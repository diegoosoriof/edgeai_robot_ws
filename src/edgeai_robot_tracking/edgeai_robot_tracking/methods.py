"""Startup method contract shared by the launch file and ROS node."""

DETECTION_METHODS = ('green_seeds', 'geometry')


def validate_detection_method(value):
    if value not in DETECTION_METHODS:
        raise ValueError(f'Unsupported detection_method {value!r}; choose '
                         + ' or '.join(DETECTION_METHODS))
    return value
