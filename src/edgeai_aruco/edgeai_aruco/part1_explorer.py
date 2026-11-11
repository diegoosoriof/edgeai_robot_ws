"""Part 1 start gate, raw-map fallback, and ArUco camera sweeps."""

import math

from action_msgs.msg import GoalStatus
from nav2_msgs.action import Spin
import rclpy
from rclpy.action import ActionClient
from rclpy.qos import DurabilityPolicy, QoSProfile
from std_msgs.msg import Bool

from mrtsp_exploration_ros2.explorer_node import MRTSPExplorerNode
from mrtsp_exploration_ros2.occupancy_grid import OccupancyGridAdapter
from mrtsp_exploration_ros2.wfd_frontier import extract_frontiers


class Part1Explorer(MRTSPExplorerNode):
    def __init__(self):
        super().__init__()
        self.run_enabled = False
        self.scan_in_progress = False
        self.spin_client = ActionClient(self, Spin, '/spin')
        self.create_subscription(
            Bool, '/part1/run_enabled', self.on_start,
            QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL),
        )
        self.get_logger().info('Part 1 explorer ready; waiting for /part1/start')

    def on_start(self, message):
        if message.data and not self.run_enabled:
            self.run_enabled = True
            self.get_logger().info('Part 1 start received; planning frontier goals')

    def _on_timer(self):
        if not self.run_enabled or self.scan_in_progress:
            return
        super()._on_timer()

    def _compute_frontier_context(self, robot_state):
        optimized_map, frontiers, map_adapter = super()._compute_frontier_context(robot_state)
        if frontiers:
            return optimized_map, frontiers, map_adapter

        # Smoothing, free-space dilation and the normal minimum cluster size
        # can hide small unknown regions. Check the raw scan map before ending.
        frontiers = extract_frontiers(
            map_adapter.to_paper_image(),
            map_adapter,
            OccupancyGridAdapter(self.costmap_msg, occupied_threshold=self.occ_threshold),
            robot_world_position=robot_state.position,
            min_frontier_size_cells=1,
            occ_threshold=self.occ_threshold,
        )
        if frontiers:
            self._log_throttled(
                'raw_frontiers', 'info', f'Found {len(frontiers)} remaining frontiers on the raw SLAM map'
            )
        return optimized_map, frontiers, map_adapter

    def _goal_result_callback(self, future, dispatch_id, goal_kind):
        current = dispatch_id == self.current_dispatch_id
        try:
            status = future.result().status
        except Exception:
            status = None
        super()._goal_result_callback(future, dispatch_id, goal_kind)
        if current and goal_kind == 'frontier' and status == GoalStatus.STATUS_SUCCEEDED:
            self.start_camera_sweep()

    def start_camera_sweep(self):
        if not self.spin_client.server_is_ready():
            self.get_logger().warning('Spin action unavailable; continuing exploration')
            return
        goal = Spin.Goal()
        goal.target_yaw = 2.0 * math.pi
        goal.time_allowance.sec = 25
        self.scan_in_progress = True
        self.spin_client.send_goal_async(goal).add_done_callback(self.on_sweep_response)
        self.get_logger().info('Sweeping the camera after reaching a frontier')

    def on_sweep_response(self, future):
        try:
            handle = future.result()
            if handle is None or not handle.accepted:
                self.scan_in_progress = False
                return
            handle.get_result_async().add_done_callback(self.on_sweep_result)
        except Exception as error:
            self.scan_in_progress = False
            self.get_logger().warning(f'Camera sweep could not start: {error}')

    def on_sweep_result(self, future):
        self.scan_in_progress = False
        try:
            self.get_logger().info(f'Camera sweep ended with status {future.result().status}')
        except Exception as error:
            self.get_logger().warning(f'Camera sweep ended without result: {error}')


def main(args=None):
    rclpy.init(args=args)
    node = Part1Explorer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
