from edgeai_robot_tracking.teleop import active_velocity, key_velocity


def test_keyboard_commands_stop_on_expiry_or_unknown_key():
    command = key_velocity('w', .15, .6)
    assert command == (.15, 0.)
    assert active_velocity(command, 10., 10.1, .25) == command
    assert active_velocity(command, 10., 10.25, .25) == (0., 0.)
    assert key_velocity('s', .15, .6) == (-.15, 0.)
    assert key_velocity('a', .15, .6) == (0., .6)
    assert key_velocity('d', .15, .6) == (0., -.6)
    assert key_velocity(' ', .15, .6) == (0., 0.)
    assert key_velocity('x', .15, .6) == (0., 0.)
