from pathlib import Path

from setuptools import find_packages, setup


resources = [
    ('share/ament_index/resource_index/packages', ['resource/edgeai_rescue']),
    ('share/edgeai_rescue', ['package.xml', 'README.md']),
]
for directory in ('config', 'launch', 'rviz', 'maps'):
    for path in Path(directory).rglob('*'):
        if path.is_file() and '__pycache__' not in path.parts:
            resources.append((f'share/edgeai_rescue/{path.parent}', [str(path)]))

setup(
    name='edgeai_rescue', version='0.1.0', packages=find_packages(exclude=['test']),
    data_files=resources, install_requires=['setuptools', 'numpy', 'PyYAML'],
    zip_safe=True, maintainer='EdgeAI team', maintainer_email='jingweid@kth.se',
    description='Part 2: localized Nav2 rescue with an asynchronous mission FSM',
    license='Apache-2.0', extras_require={'test': ['pytest']},
    entry_points={'console_scripts': [
        'mission = edgeai_rescue.mission_node:main',
        'verifier = edgeai_rescue.verifier_node:main',
        'peer_obstacles = edgeai_rescue.peer_node:main',
        'generate_reference_map = edgeai_rescue.reference_map:main',
    ]},
)
