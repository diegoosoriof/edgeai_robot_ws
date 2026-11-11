"""Part 1 start gate and camera sweeps around the existing MRTSP explorer."""

import math

from action_msgs.msg import GoalStatus
from nav2_msgs.action import Spin
import rclpy
from rclpy.action import ActionClient
from rclpy.qos import DurabilityPolicy, QoSProfile
from std_msgs.msg import Bool

from mrtsp_exploration_ros2.explorer_node import MRTSPExplorerNode


class Part1Explorer(MRTSPExplorerNode):
    def __init__(self):
        super().__init__()
        self.run_enabled = False
        self.scan_in_progress = False
        self.failed_frontiers = []
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

    def _filter_frontiers_by_min_goal_distance(self, frontiers, robot_state):
        frontiers = super()._filter_frontiers_by_min_goal_distance(frontiers, robot_state)
        now_ns = self.get_clock().now().nanoseconds
        self.failed_frontiers = [(point, deadline) for point, deadline
                                 in self.failed_frontiers if deadline > now_ns]
        kept = [frontier for frontier in frontiers if not any(
            math.dist(frontier.center_array[:2], point) < 0.6
            for point, _ in self.failed_frontiers)]
        if not kept and frontiers:
            self._log_throttled('failed_frontiers', 'warn',
                                'Waiting for failed frontier cooldown or map growth')
        return kept

    def _goal_result_callback(self, future, dispatch_id, goal_kind):
        current = dispatch_id == self.current_dispatch_id
        target = tuple(self.active_target[:2]) if current and self.active_target is not None else None
        try:
            status = future.result().status
        except Exception:
            status = None
        super()._goal_result_callback(future, dispatch_id, goal_kind)
        if not current or goal_kind != 'frontier':
            return
        if status == GoalStatus.STATUS_SUCCEEDED:
            self.start_camera_sweep()
        elif status not in (GoalStatus.STATUS_CANCELED, None) and target is not None:
            self.failed_frontiers.append((target,
                                          self.get_clock().now().nanoseconds + 30_000_000_000))

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
