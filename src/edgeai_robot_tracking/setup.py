from glob import glob

from setuptools import find_packages, setup


setup(
    name='edgeai_robot_tracking',
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/edgeai_robot_tracking']),
        ('share/edgeai_robot_tracking', ['package.xml']),
        ('share/edgeai_robot_tracking/launch', glob('launch/*.launch.py')),
        ('share/edgeai_robot_tracking/config', glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    extras_require={'test': ['pytest']},
    zip_safe=True,
    maintainer='Jingwei Dong',
    maintainer_email='jingweid@kth.se',
    description='Green wheel RGB-D detection and temporal tracking',
    license='Apache-2.0',
    entry_points={'console_scripts': [
        'rgbd_tracker = edgeai_robot_tracking.detector_node:main',
        'keyboard_teleop = edgeai_robot_tracking.teleop:main',
    ]},
)
