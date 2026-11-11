"""ROS adapter for the explicit rescue mission FSM."""

import json
import uuid

from geometry_msgs.msg import PoseArray, Twist
from nav_msgs.msg import OccupancyGrid as MapMessage
import numpy as np
import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, String
from std_srvs.srv import Trigger

from .fsm import Inputs, MissionFSM, State
from .geometry import OccupancyGrid
from .navigation import Navigation
from .ros_support import JsonEvidence, LATCHED, load_config, pose_message, steady_timer, yaw


class MissionNode(Node):
    def __init__(self):
        super().__init__('rescue_mission')
        self.config = load_config(self)
        self.declare_parameter('autostart_mission', True)
        self.declare_parameter('evidence_dir', 'rescue_evidence')
        self.declare_parameter('elapsed_before_part2', float(self.config['timing']['elapsed_before_part2']))
        self.declare_parameter('initial_scan_angle', float(self.config['initial_scan_angle']))
        self.declare_parameter('footprint', [-.23, -.17, .20, -.17, .20, .17, -.23, .17])
        self.declare_parameter('control_offset_x', 0.)
        self.config['timing']['elapsed_before_part2'] = self.get_parameter('elapsed_before_part2').value
        self.config['initial_scan_angle'] = self.get_parameter('initial_scan_angle').value
        if abs(self.config['initial_scan_angle']) > .5:
            raise ValueError('Optional initial scan is limited to 0.5 rad; it is collision checked by Nav2')
        self.footprint = np.asarray(self.get_parameter('footprint').value).reshape(-1, 2).tolist()
        self.fsm = MissionFSM(self.config, self.get_parameter('autostart_mission').value)
        self.run_id = uuid.uuid4().hex
        self.navigation = Navigation(self, self.config, self.footprint, self.get_parameter('control_offset_x').value)
        self.evidence = JsonEvidence(self.get_parameter('evidence_dir').value, 'mission_events.jsonl')
        self.map_grid, self.verification = None, {}
        self.last_state = None
        self.state_pub = self.create_publisher(String, '/rescue/state', LATCHED)
        self.progress_pub = self.create_publisher(String, '/rescue/progress', LATCHED)
        self.target_pub = self.create_publisher(String, '/rescue/active_target', LATCHED)
        self.goals_pub = self.create_publisher(PoseArray, '/rescue/goals', LATCHED)
        self.locks = {robot: self.create_publisher(Bool, f'/{robot}/mission_stop', 1) for robot in self.config['robots']}
        self.stops = {robot: self.create_publisher(Twist, f'/{robot}/mux_cmd_vel', 1) for robot in self.config['robots']}
        self.map_sub = self.create_subscription(MapMessage, '/map', self.on_map, LATCHED)
        self.verification_sub = self.create_subscription(String, '/rescue/verification',
            lambda message: setattr(self, 'verification', json.loads(message.data)), LATCHED)
        self.start_service = self.create_service(Trigger, '/rescue/start', self.start)
        self.stop_service = self.create_service(Trigger, '/rescue/stop', self.stop)
        self.timer = steady_timer(self, .1, self.tick)

    def on_map(self, message):
        try:
            if message.header.frame_id != self.config['map_frame']:
                raise ValueError('Occupancy map frame must be map')
            info, origin = message.info, message.info.origin
            self.map_grid = OccupancyGrid(np.asarray(message.data).reshape(info.height, info.width),
                                          info.resolution, (origin.position.x, origin.position.y, yaw(origin.orientation)))
        except ValueError as error:
            self.map_grid = None
            self.get_logger().error(str(error))

    def start(self, request, response):
        if self.fsm.state not in (State.INITIALIZING, State.FAILED, State.MISSION_COMPLETED):
            response.success, response.message = False, 'Mission already running'
            return response
        self.navigation.cancel_all()
        self.fsm, self.run_id = MissionFSM(self.config, True), uuid.uuid4().hex
        response.success, response.message = True, f'Mission requested: {self.run_id}'
        return response

    def stop(self, request, response):
        now = self.get_clock().now().nanoseconds*1e-9
        self.fsm.fail(now, 'Stopped by operator')
        self.navigation.cancel_all()
        response.success, response.message = True, 'Mission stopped; velocity locks active'
        return response

    def tick(self):
        now = self.get_clock().now().nanoseconds*1e-9
        verification = self.verification
        fresh = 0 <= now-verification.get('time', -1e6) < .5
        localized = fresh and all(verification.get('localized', {}).get(robot, False) for robot in self.config['robots'])
        inputs = Inputs(
            ready=self.map_grid is not None and self.navigation.ready(), localized=localized,
            selection=self.navigation.selection_status(),
            navigation={robot: job.status for robot, job in self.navigation.jobs.items()},
            verification=verification if fresh and verification.get('run_id') == self.run_id else {},
            recovered=self.navigation.recovered(),
        )
        effects = self.fsm.step(now, inputs)
        for kind, first, second in effects:
            if kind != 'state':
                self.evidence.write({'time': now, 'run_id': self.run_id, 'event': kind,
                                     'robot_or_target': first, 'target': second})
            if kind == 'select':
                self.navigation.select(self.fsm.selection_goals, self.map_grid)
                goals = PoseArray()
                goals.header.frame_id, goals.header.stamp = self.config['map_frame'], self.get_clock().now().to_msg()
                goals.poses = [
                    pose_message(self.config['targets'][target]['goals'][robot], goals.header.stamp).pose
                    for robot, target in self.fsm.assigned_goals.items()
                ]
                self.goals_pub.publish(goals)
            elif kind == 'navigate':
                self.navigation.navigate(first, second)
            elif kind == 'scan':
                self.navigation.spin(first)
            elif kind == 'depart_s1':
                self.navigation.depart_s1(first)
            elif kind == 'cancel_all':
                self.navigation.cancel_all()
            elif kind == 'clear_costmaps':
                self.navigation.clear_costmaps()
            elif kind == 'state':
                event = {'time': now, 'run_id': self.run_id, 'state': first, 'reason': second}
                self.evidence.write(event)
                self.get_logger().info(f'sim={now:.3f} {first}: {second}')
        state = self.fsm.state.name
        self.state_pub.publish(String(data=state))
        self.target_pub.publish(String(data=json.dumps({'run_id': self.run_id, 'target': self.fsm.active_target})))
        for robot in self.config['robots']:
            locked = robot not in self.fsm.moving_robots or not localized
            self.locks[robot].publish(Bool(data=locked))
            if locked:
                # Explicit zeros are required: dropping mux input does not stop
                # a Gazebo DiffDrive that retains its previous command.
                self.stops[robot].publish(Twist())
        progress = {'time': now, 'run_id': self.run_id, 'state': state, 'reason': self.fsm.reason,
                    'localized': verification.get('localized', {}),
                    's1_completed': self.fsm.s1_completed, 'c1_completed': self.fsm.c1_completed,
                    'targets': verification.get('targets', {}), 'recoveries': self.fsm.recoveries,
                    'moving_robots': sorted(self.fsm.moving_robots),
                    'elapsed': max(0., now-(self.fsm.start_time or now)),
                    'elapsed_before_part2': self.config['timing']['elapsed_before_part2']}
        self.progress_pub.publish(String(data=json.dumps(progress)))
        if state != self.last_state and self.fsm.state in (State.MISSION_COMPLETED, State.FAILED):
            (self.evidence.path/'mission_summary.json').write_text(json.dumps(progress, indent=2)+'\n')
        self.last_state = state


def main(args=None):
    rclpy.init(args=args)
    node = MissionNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        if rclpy.ok():
            node.navigation.cancel_all()
            for publisher in node.stops.values():
                publisher.publish(Twist())
        node.evidence.close()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
