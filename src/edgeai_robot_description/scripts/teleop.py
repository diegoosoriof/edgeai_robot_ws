#!/usr/bin/env python3
"""Keyboard teleoperation for a differential-drive robot."""

import os
import select
import sys
import termios
import time
import tty

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node


class KeyboardTeleop(Node):
    def __init__(self):
        super().__init__("keyboard_teleop")
        self.declare_parameter("linear_speed", 0.3)
        self.declare_parameter("angular_speed", 1.0)
        self.declare_parameter("cmd_vel_topic", "/cmd_vel")
        self.declare_parameter("key_timeout", 0.5)

        self.linear_speed = float(self.get_parameter("linear_speed").value)
        self.angular_speed = float(self.get_parameter("angular_speed").value)
        self.key_timeout = float(self.get_parameter("key_timeout").value)
        topic = str(self.get_parameter("cmd_vel_topic").value)
        self.publisher = self.create_publisher(Twist, topic, 10)
        self.command = Twist()
        self.last_key_time = 0.0
        self.running = True
        self.timer = self.create_timer(0.01, self._on_timer)

        self._stdin_fd = sys.stdin.fileno()
        self._terminal_settings = termios.tcgetattr(self._stdin_fd)
        tty.setraw(self._stdin_fd)
        self.get_logger().info(
            f"Keyboard teleop publishing to {topic}. Use arrow keys; "
            "space stops, q quits."
        )

    def _read_key(self):
        if not select.select([sys.stdin], [], [], 0)[0]:
            return None

        key = os.read(self._stdin_fd, 1).decode(errors="ignore")
        if key != "\x1b":
            return key

        # Arrow keys arrive as ESC [ A/B/C/D. Read the rest without blocking.
        sequence = key
        for _ in range(2):
            ready, _, _ = select.select([sys.stdin], [], [], 0.01)
            if not ready:
                break
            sequence += os.read(self._stdin_fd, 1).decode(errors="ignore")
        return sequence

    def _on_timer(self):
        key = self._read_key()
        if key is not None:
            if key == "q" or key == "\x03":
                self.running = False
                return
            if key == "\x1b[A":  # up
                self.command.linear.x = self.linear_speed
                self.command.angular.z = 0.0
            elif key == "\x1b[B":  # down
                self.command.linear.x = -self.linear_speed
                self.command.angular.z = 0.0
            elif key == "\x1b[C":  # right
                self.command.linear.x = 0.0
                self.command.angular.z = -self.angular_speed
            elif key == "\x1b[D":  # left
                self.command.linear.x = 0.0
                self.command.angular.z = self.angular_speed
            elif key == " ":
                self.command = Twist()
            else:
                return
            self.last_key_time = time.monotonic()

        if time.monotonic() - self.last_key_time > self.key_timeout:
            self.command = Twist()
        self.publisher.publish(self.command)

    def stop(self):
        self.publisher.publish(Twist())
        termios.tcsetattr(self._stdin_fd, termios.TCSADRAIN, self._terminal_settings)


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = KeyboardTeleop()
        while rclpy.ok() and node.running:
            rclpy.spin_once(node, timeout_sec=0.01)
    except (KeyboardInterrupt, EOFError):
        pass
    finally:
        if node is not None:
            node.stop()
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
