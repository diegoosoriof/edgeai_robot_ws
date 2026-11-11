"""Reuse the existing URDF, adapting only the Part 2 instances in memory."""

from itertools import product
import math
import xml.etree.ElementTree as ET

import numpy as np
import xacro


def rotation(rpy):
    r, p, y = rpy
    cr, cp, cy = np.cos(rpy)
    sr, sp, sy = np.sin(rpy)
    return np.array([[cy*cp, cy*sp*sr-sy*cr, cy*sp*cr+sy*sr],
                     [sy*cp, sy*sp*sr+cy*cr, sy*sp*cr-cy*sr], [-sp, cp*sr, cp*cr]])


def transform(xyz=(0, 0, 0), rpy=(0, 0, 0)):
    result = np.eye(4)
    result[:3, :3] = rotation(rpy)
    result[:3, 3] = xyz
    return result


def urdf_origin(element):
    return (np.eye(4) if element is None else transform(
        np.fromstring(element.get('xyz', '0 0 0'), sep=' '),
        np.fromstring(element.get('rpy', '0 0 0'), sep=' ')))


def derive_footprint(root):
    """Envelope of actual collision bodies, including sampled caster swivels."""
    points = []
    for swivel in np.linspace(-math.pi, math.pi, 17):
        frames = {'base_footprint': np.eye(4)}
        pending = list(root.findall('joint'))
        while pending:
            ready = [joint for joint in pending if joint.find('parent').get('link') in frames]
            if not ready:
                raise ValueError('Robot joint tree is not connected to base_footprint')
            for joint in ready:
                offset = urdf_origin(joint.find('origin'))
                if 'caster_swivel' in joint.get('name'):
                    offset = offset @ transform(rpy=(0, 0, swivel))
                frames[joint.find('child').get('link')] = frames[joint.find('parent').get('link')] @ offset
                pending.remove(joint)
        for link in root.findall('link'):
            for collision in link.findall('collision'):
                geometry = collision.find('geometry')
                if geometry.find('box') is not None:
                    half = np.fromstring(geometry.find('box').get('size'), sep=' ')/2
                elif geometry.find('cylinder') is not None:
                    cylinder = geometry.find('cylinder')
                    half = np.array([float(cylinder.get('radius'))]*2 + [float(cylinder.get('length'))/2])
                else:
                    raise ValueError('Unsupported robot collision shape')
                frame = frames[link.get('name')] @ urdf_origin(collision.find('origin'))
                for signs in product((-1, 1), repeat=3):
                    points.append((frame @ np.r_[half*np.array(signs), 1])[:2])
    # Extra 5 mm covers caster sampling and rounds outwards to centimetres.
    lo = np.floor((np.min(points, axis=0)-.005)*100)/100
    hi = np.ceil((np.max(points, axis=0)+.005)*100)/100
    return [[float(lo[0]), float(lo[1])], [float(hi[0]), float(lo[1])],
            [float(hi[0]), float(hi[1])], [float(lo[0]), float(hi[1])]]


def axle_offset(root):
    wheels = [root.find(f"joint[@name='{side}_wheel_joint']") for side in ('left', 'right')]
    if any(joint.find('parent').get('link') != 'base_link' for joint in wheels):
        raise ValueError('Expected the existing drive-wheel joints on base_link')
    center = sum(urdf_origin(joint.find('origin'))[:2, 3] for joint in wheels)/2
    center += urdf_origin(root.find("joint[@name='base_footprint_joint']/origin"))[:2, 3]
    if abs(center[1]) > 1e-6:
        raise ValueError('Drive axle must be centered laterally')
    return float(center[0])


def drive_center_x(xacro_file):
    return axle_offset(ET.fromstring(xacro.process_file(str(xacro_file)).toxml()))


def describe_robot(xacro_file, name, enable_camera=False):
    root = ET.fromstring(xacro.process_file(str(xacro_file)).toxml())
    footprint = derive_footprint(root)
    # DiffDrive velocities/odometry refer to the wheel axle. Keep the original
    # base_footprint and all body geometry as a static child of that control frame.
    offset = axle_offset(root)
    ET.SubElement(root, 'link', name='drive_frame')
    joint = ET.SubElement(root, 'joint', name='drive_frame_joint', type='fixed')
    ET.SubElement(joint, 'parent', link='drive_frame')
    ET.SubElement(joint, 'child', link='base_footprint')
    ET.SubElement(joint, 'origin', xyz=f'{-offset} 0 0', rpy='0 0 0')
    prefix = f'/{name}'
    for gazebo in root.findall('gazebo'):
        for sensor in list(gazebo.findall('sensor')):
            if sensor.get('type') == 'rgbd_camera':
                if not enable_camera:
                    gazebo.remove(sensor)
                    continue
                sensor.find('topic').text = f'{prefix}/camera'
                # Native cloud XYZ is X-forward. Do not relabel it optical.
                for parent in (sensor, sensor.find('camera')):
                    if parent is not None:
                        for tag in list(parent.findall('optical_frame_id')):
                            parent.remove(tag)
            elif sensor.get('type') == 'gpu_lidar':
                sensor.find('topic').text = f'{prefix}/scan'
                sensor.find('gz_frame_id').text = f'{name}/lidar_link'
        for plugin in gazebo.findall('plugin'):
            if plugin.get('name', '').endswith('DiffDrive'):
                for key, value in {'topic': f'{prefix}/cmd_vel', 'odom_topic': f'{prefix}/odom',
                                   'tf_topic': f'{prefix}/tf', 'frame_id': f'{name}/odom',
                                   'child_frame_id': f'{name}/drive_frame'}.items():
                    plugin.find(key).text = value
            elif plugin.get('name', '').endswith('JointStatePublisher'):
                plugin.find('topic').text = f'{prefix}/joint_states'
    colors = ('0.15 0.55 0.95 1', '1.0 0.45 0.1 1')
    for material in root.findall("link[@name='base_link']/visual/material/color"):
        material.set('rgba', colors[0 if name == 'robot1' else 1])
    return ET.tostring(root, encoding='unicode'), footprint
