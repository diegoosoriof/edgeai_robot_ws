from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    params = os.path.join(get_package_share_directory('edgeai_robot_description'),
                          'config', 'map_alignment.yaml')
    return LaunchDescription([
        Node(package='edgeai_robot_description', executable='map_alignment_node',
             name='map_alignment', output='screen', parameters=[params]),
    ])
