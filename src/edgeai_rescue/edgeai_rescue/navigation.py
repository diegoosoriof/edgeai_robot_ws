"""Asynchronous Nav2 action/lifecycle transport for the mission FSM."""

from dataclasses import dataclass

from action_msgs.msg import GoalStatus
from lifecycle_msgs.srv import GetState
from nav2_msgs.action import BackUp, ComputePathToPose, NavigateToPose, Spin
from nav2_msgs.srv import ClearEntireCostmap, GetCostmap
import numpy as np
from rclpy.action import ActionClient

from .geometry import OccupancyGrid, shift_pose, validate_goals
from .ros_support import pose_message, steady_timer, yaw


@dataclass
class Job:
    status: str = 'PENDING'
    handle: object = None
    result: object = None
    cancel_requested: bool = False


class Navigation:
    def __init__(self, node, config, footprint, control_offset):
        self.node, self.config, self.footprint = node, config, footprint
        self.control_offset = control_offset
        self.clients, self.planners, self.spinners, self.costmaps = {}, {}, {}, {}
        self.backups = {}
        self.clear_clients, self.lifecycle, self.lifecycle_states, self.lifecycle_pending = {}, {}, {}, {}
        self.jobs, self.plan_jobs, self.clear_futures = {}, {}, []
        self.selection = None
        self.pending_selection = None
        self.generation = 0
        for robot in config['robots']:
            self.clients[robot] = ActionClient(node, NavigateToPose, f'/{robot}/navigate_to_pose')
            self.planners[robot] = ActionClient(node, ComputePathToPose, f'/{robot}/compute_path_to_pose')
            self.spinners[robot] = ActionClient(node, Spin, f'/{robot}/spin')
            self.backups[robot] = ActionClient(node, BackUp, f'/{robot}/backup')
            self.costmaps[robot] = node.create_client(GetCostmap, f'/{robot}/global_costmap/get_costmap')
            for area in ('global', 'local'):
                topic = f'/{robot}/{area}_costmap/clear_entirely_{area}_costmap'
                self.clear_clients[topic] = node.create_client(ClearEntireCostmap, topic)
            for name in ('amcl', 'planner_server', 'controller_server', 'behavior_server', 'bt_navigator', 'collision_monitor'):
                service = f'/{robot}/{name}/get_state'
                self.lifecycle[service] = node.create_client(GetState, service)
        self.lifecycle['/map_server/get_state'] = node.create_client(GetState, '/map_server/get_state')
        self.timer = steady_timer(node, .8, self.poll_lifecycle)
        self.selection_timer = steady_timer(node, .1, self.advance_selection)

    def poll_lifecycle(self):
        for service, client in self.lifecycle.items():
            old = self.lifecycle_pending.get(service)
            if client.service_is_ready() and (old is None or old.done()):
                future = client.call_async(GetState.Request())
                self.lifecycle_pending[service] = future
                future.add_done_callback(lambda done, key=service: self.record_state(key, done))

    def record_state(self, key, future):
        try:
            self.lifecycle_states[key] = future.result().current_state.id
        except Exception:
            self.lifecycle_states[key] = 0

    def ready(self):
        return (all(self.lifecycle_states.get(service) == 3 for service in self.lifecycle)
                and all(client.server_is_ready() for client in self.clients.values())
                and all(client.server_is_ready() for client in self.planners.values())
                and all(client.server_is_ready() for client in self.backups.values())
                and all(client.server_is_ready() for client in self.spinners.values())
                and all(client.service_is_ready() for client in self.costmaps.values()))

    def send(self, client, goal):
        job = Job()
        if not client.server_is_ready():
            job.status = 'FAILED'
            return job
        future = client.send_goal_async(goal)
        future.add_done_callback(lambda done: self.accepted(job, done))
        return job

    def accepted(self, job, future):
        try:
            job.handle = future.result()
            if not job.handle.accepted:
                job.status = 'REJECTED'
                return
            job.status = 'ACTIVE'
            result = job.handle.get_result_async()
            result.add_done_callback(lambda done: self.finished(job, done))
            if job.cancel_requested:
                job.handle.cancel_goal_async()
        except Exception as error:
            job.status = 'FAILED'
            self.node.get_logger().error(f'Nav2 goal request failed: {error}')

    def finished(self, job, future):
        try:
            response = future.result()
            job.result = response.result
            job.status = ('SUCCEEDED' if response.status == GoalStatus.STATUS_SUCCEEDED
                          and getattr(response.result, 'error_code', 0) == 0
                          else 'CANCELED' if response.status == GoalStatus.STATUS_CANCELED else 'FAILED')
            if job.status == 'FAILED':
                self.node.get_logger().warning(f'Nav2 action failed: {getattr(job.result, "error_msg", "")}')
        except Exception as error:
            job.status = 'FAILED'
            self.node.get_logger().error(f'Nav2 result failed: {error}')

    def navigate(self, robot, target):
        goal = NavigateToPose.Goal()
        values = shift_pose(self.config['targets'][target]['goals'][robot], self.control_offset)
        goal.pose = pose_message(values, self.node.get_clock().now().to_msg())
        self.jobs[robot] = self.send(self.clients[robot], goal)

    def spin(self, robot):
        goal = Spin.Goal()
        goal.target_yaw = float(self.config['initial_scan_angle'])
        goal.time_allowance.sec = 15
        self.jobs[robot] = self.send(self.spinners[robot], goal)

    def depart_s1(self, robot):
        goal = BackUp.Goal()
        goal.target.x = float(self.config['s1_departure_distance'])
        goal.speed = .1
        goal.time_allowance.sec = 10
        self.jobs[robot] = self.send(self.backups[robot], goal)

    def select(self, target, static_map):
        self.generation += 1
        generation = self.generation
        self.selection, self.jobs, self.plan_jobs = None, {}, {}
        try:
            if static_map is None:
                raise ValueError('Shared occupancy map has not arrived')
            validate_goals(self.config['targets'][target], static_map, self.footprint)
            # Drop previous peer trails at phase boundaries; sensors and the
            # static layer repopulate before validating the new current snapshot.
            self.clear_costmaps()
            self.pending_selection = (target, generation, self.node.get_clock().now().nanoseconds*1e-9)
        except ValueError as error:
            self.selection = False
            self.node.get_logger().warning(str(error))

    def advance_selection(self):
        if self.pending_selection is None:
            return
        target, generation, started = self.pending_selection
        if generation != self.generation:
            self.pending_selection = None
            return
        if (self.node.get_clock().now().nanoseconds*1e-9-started < .6
                or len(self.clear_futures) != len(self.clear_clients)
                or not all(future.done() for future in self.clear_futures)):
            return
        self.pending_selection = None
        if any(future.exception() for future in self.clear_futures):
            self.selection = False
            return
        for robot in self.config['targets'][target]['robots']:
            if not self.costmaps[robot].service_is_ready():
                self.selection = False
                return
            future = self.costmaps[robot].call_async(GetCostmap.Request())
            future.add_done_callback(lambda done, name=robot: self.costmap_ready(target, name, generation, done))

    def costmap_ready(self, target, robot, generation, future):
        if generation != self.generation or self.selection is False:
            return
        try:
            costmap = future.result().map
            metadata = costmap.metadata
            data = np.asarray(costmap.data, dtype='uint8').reshape(metadata.size_y, metadata.size_x)
            origin = metadata.origin
            grid = OccupancyGrid(data, metadata.resolution,
                                 (origin.position.x, origin.position.y, yaw(origin.orientation)),
                                 blocked_at=254, unknown=255)
            validate_goals(self.config['targets'][target], grid, self.footprint)
            goal = ComputePathToPose.Goal()
            values = shift_pose(self.config['targets'][target]['goals'][robot], self.control_offset)
            goal.goal = pose_message(values, self.node.get_clock().now().to_msg())
            goal.planner_id, goal.use_start = 'GridBased', False
            self.plan_jobs[robot] = self.send(self.planners[robot], goal)
        except Exception as error:
            self.selection = False
            self.node.get_logger().warning(f'Costmap goal validation failed: {error}')

    def selection_status(self, target):
        if self.selection is False:
            return False
        required = self.config['targets'][target]['robots']
        if any(job.status in ('REJECTED', 'FAILED', 'CANCELED') for job in self.plan_jobs.values()):
            return False
        if all(robot in self.plan_jobs and self.plan_jobs[robot].status == 'SUCCEEDED'
               and self.plan_jobs[robot].result.path.poses for robot in required):
            return True
        return None

    def cancel_all(self):
        self.generation += 1
        for job in list(self.jobs.values())+list(self.plan_jobs.values()):
            job.cancel_requested = True
            if job.handle and job.status == 'ACTIVE':
                job.handle.cancel_goal_async()

    def clear_costmaps(self):
        self.clear_futures = []
        for client in self.clear_clients.values():
            if client.service_is_ready():
                self.clear_futures.append(client.call_async(ClearEntireCostmap.Request()))

    def recovered(self):
        jobs = list(self.jobs.values())+list(self.plan_jobs.values())
        return (all(job.status not in ('PENDING', 'ACTIVE') for job in jobs)
                and len(self.clear_futures) == len(self.clear_clients)
                and all(future.done() and future.exception() is None for future in self.clear_futures))
