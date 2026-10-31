"""Exercise the real keyboard executable through a PTY and an isolated Twist topic."""

import os
import pty
import signal
import subprocess
import sys
import termios
import time

from geometry_msgs.msg import Twist
import pytest
import rclpy
from rclpy.context import Context
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node


@pytest.mark.skipif(os.environ.get('RUN_ROS_TESTS') != '1', reason='opt-in DDS and PTY test')
@pytest.mark.parametrize('exit_method', ['quit', 'ctrl_c'])
def test_real_teleop_stops_and_restores_terminal(exit_method):
    context = Context()
    rclpy.init(context=context)
    source = Node('teleop_stop_probe', context=context)
    executor = SingleThreadedExecutor(context=context)
    executor.add_node(source)
    topic = f'/teleop_test_{os.getpid()}/cmd_vel'
    commands = []
    subscription = source.create_subscription(Twist, topic, commands.append, 10)
    master, slave = pty.openpty()
    original = termios.tcgetattr(slave)
    process = subprocess.Popen([
        sys.executable, '-c',
        'import sys; from edgeai_robot_tracking.teleop import main; sys.exit(main())',
        '--topic', topic, '--input-timeout', '0.15',
    ], stdin=slave, stdout=slave, stderr=slave, start_new_session=True)

    def spin_until(predicate, timeout=3.):
        deadline = time.monotonic() + timeout
        while not predicate() and time.monotonic() < deadline:
            executor.spin_once(timeout_sec=.01)
        assert predicate(), 'Keyboard process did not publish the expected command'

    try:
        spin_until(lambda: bool(commands))
        commands.clear()
        os.write(master, b'w')
        spin_until(lambda: any(m.linear.x > 0 for m in commands))
        commands.clear()
        spin_until(lambda: bool(commands) and commands[-1].linear.x == 0)
        os.write(master, b'w')
        spin_until(lambda: commands[-1].linear.x > 0)
        commands.clear()
        os.write(master, b' ')
        spin_until(lambda: bool(commands) and commands[-1].linear.x == 0)
        os.write(master, b'w')
        spin_until(lambda: commands[-1].linear.x > 0)
        commands.clear()
        if exit_method == 'ctrl_c':
            process.send_signal(signal.SIGINT)
        else:
            os.write(master, b'q')
        spin_until(lambda: process.poll() is not None)
        for _ in range(5):
            executor.spin_once(timeout_sec=.02)
        assert process.returncode == 0
        assert commands and commands[-1].linear.x == 0 and commands[-1].angular.z == 0
        assert termios.tcgetattr(slave) == original
        assert subscription
    finally:
        if process.poll() is None:
            process.terminate()
            try:
                process.wait(timeout=3)
            except subprocess.TimeoutExpired:
                process.kill()
                process.wait(timeout=3)
        os.close(master)
        os.close(slave)
        executor.shutdown()
        source.destroy_node()
        context.try_shutdown()
