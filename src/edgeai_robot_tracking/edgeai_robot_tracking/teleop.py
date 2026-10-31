"""Terminal keyboard control with a wall-time input watchdog and explicit stops."""

import argparse
import math
import select
import signal
import sys
import termios
import time
import tty

from geometry_msgs.msg import Twist
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.signals import SignalHandlerOptions
from rclpy.utilities import remove_ros_args


def key_velocity(key, linear, angular):
    """A key sets one command; only repeated input keeps motion alive."""
    return {
        'w': (linear, 0.), 's': (-linear, 0.),
        'a': (0., angular), 'd': (0., -angular),
        'j': (linear, angular), 'l': (linear, -angular),
    }.get(key.lower(), (0., 0.))


def active_velocity(command, last_key, now, timeout):
    return command if now - last_key < timeout else (0., 0.)


def parse_args(args=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--namespace', default='robot2', help='Robot namespace (default robot2)')
    parser.add_argument('--topic', default='', help='Exact Twist topic; overrides namespace')
    parser.add_argument('--linear-speed', type=float, default=0.15, help='m/s')
    parser.add_argument('--angular-speed', type=float, default=0.6, help='rad/s')
    parser.add_argument('--input-timeout', type=float, default=0.25, help='Wall seconds')
    parser.add_argument('--rate', type=float, default=20.0, help='Publishing rate in wall Hz')
    options = parser.parse_args(remove_ros_args(args=args)[1:])
    for name in ('linear_speed', 'angular_speed', 'input_timeout', 'rate'):
        value = getattr(options, name)
        if not math.isfinite(value) or value <= 0:
            parser.error(f'--{name.replace("_", "-")} must be positive')
    return options


def main(args=None):
    args = sys.argv if args is None else args
    options = parse_args(args)
    if not sys.stdin.isatty():
        print('Keyboard control needs its own interactive terminal. Run: '
              'ros2 run edgeai_robot_tracking keyboard_teleop --namespace robot2', file=sys.stderr)
        return 1
    topic = options.topic or '/' + '/'.join(
        item for item in (options.namespace.strip('/'), 'cmd_vel') if item)
    # Own the signals so the ROS context remains alive for the final zero command.
    rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO)
    node = Node('keyboard_teleop')
    publisher = node.create_publisher(Twist, topic, 10)
    terminal = termios.tcgetattr(sys.stdin)
    old_term_handler = signal.getsignal(signal.SIGTERM)
    quitting = False

    def request_quit(signum, frame):
        nonlocal quitting
        quitting = True

    def publish(velocity):
        message = Twist()
        message.linear.x, message.angular.z = velocity
        publisher.publish(message)

    command, last_key = (0., 0.), -math.inf
    print(f'Publishing geometry_msgs/Twist to {topic}\n'
          'w/s: forward/back | a/d: turn left/right | j/l: forward arcs\n'
          'Space or x: STOP | q: quit | Ctrl+C: stop and quit\n'
          f'Input expires after {options.input_timeout:.2f} wall seconds. '
          'Use repeated key presses; keyboard repeat delay may cause a brief stop.\n'
          'Stop any other cmd_vel publishers before driving.')
    try:
        signal.signal(signal.SIGTERM, request_quit)
        tty.setcbreak(sys.stdin.fileno())
        # Start stopped; allow DDS discovery while sending zeros.
        for _ in range(5):
            publish((0., 0.))
            rclpy.spin_once(node, timeout_sec=0.04)
        while rclpy.ok() and not quitting:
            available, _, _ = select.select([sys.stdin], [], [], 1.0 / options.rate)
            if available:
                key = sys.stdin.read(1)
                if key in ('q', 'Q', '\x03', ''):
                    break
                command = key_velocity(key, options.linear_speed, options.angular_speed)
                last_key = time.monotonic()
            publish(active_velocity(command, last_key, time.monotonic(), options.input_timeout))
            rclpy.spin_once(node, timeout_sec=0.)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        # Restore the terminal even if final publication raises during ROS shutdown.
        try:
            if rclpy.ok():
                for _ in range(3):
                    publish((0., 0.))
                    rclpy.spin_once(node, timeout_sec=0.03)
        finally:
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, terminal)
            signal.signal(signal.SIGTERM, old_term_handler)
            node.destroy_node()
            if rclpy.ok():
                rclpy.shutdown()
    return 0
