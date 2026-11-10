from glob import glob

from setuptools import find_packages, setup

setup(
    name='edgeai_aruco',
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/edgeai_aruco']),
        ('share/edgeai_aruco', ['package.xml', 'README.md']),
        ('share/edgeai_aruco/config', glob('config/*.yaml')),
        ('share/edgeai_aruco/launch', glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools', 'numpy'],
    zip_safe=True,
    maintainer='Jingwei Dong',
    maintainer_email='jingweid@kth.se',
    description='ArUco perception for the EdgeAI robot',
    license='Apache-2.0',
    extras_require={'test': ['pytest']},
    entry_points={'console_scripts': [
        'aruco_detector = edgeai_aruco.node:main',
        'generate_marker = edgeai_aruco.marker_model:main',
    ]},
)
