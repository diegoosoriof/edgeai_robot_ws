"""ROS 2 transport for timestamped ArUco observations."""

import cv2
from cv_bridge import CvBridge, CvBridgeError
from geometry_msgs.msg import Pose, PoseArray, TransformStamped
from rcl_interfaces.msg import ParameterDescriptor
import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, Image
from std_msgs.msg import Header, Int32MultiArray
from tf2_ros import TransformBroadcaster
from vision_msgs.msg import Detection3D, Detection3DArray, ObjectHypothesisWithPose

from .detection import Calibration, Detector, positive_float, rotation_quaternion


class ArucoDetectorNode(Node):
    def __init__(self, **kwargs):
        super().__init__('aruco_detector', **kwargs)
        defaults = {
            'dictionary': 'DICT_4X4_50', 'marker_size': 0.20,
            'image_topic': 'camera/image', 'camera_info_topic': 'camera/camera_info',
            'detections_topic': 'aruco/detections', 'ids_topic': 'aruco/ids',
            'poses_topic': 'aruco/poses', 'debug_image_topic': 'aruco/debug_image',
            'input_frame': 'camera_optical_frame', 'optical_frame': 'camera_optical_frame',
            'publish_debug_image': True,
            'publish_tf': False, 'tf_child_prefix': 'aruco_marker_',
            'axis_length': 0.10, 'max_reprojection_error': 3.0,
        }
        for name, value in defaults.items():
            self.declare_parameter(name, value, ParameterDescriptor(read_only=True))
        self.declare_parameter('marker_size_ids', Parameter.Type.INTEGER_ARRAY,
                               ParameterDescriptor(read_only=True))
        self.declare_parameter('marker_sizes', Parameter.Type.DOUBLE_ARRAY,
                               ParameterDescriptor(read_only=True))
        self.settings = {name: self.get_parameter(name).value for name in defaults}
        marker_ids = self.get_parameter_or('marker_size_ids').value or []
        marker_sizes = self.get_parameter_or('marker_sizes').value or []
        if len(marker_ids) != len(marker_sizes) or len(set(marker_ids)) != len(marker_ids):
            raise ValueError('marker_size_ids and marker_sizes must be equally sized with unique IDs')
        self.detector = Detector(
            self.settings['dictionary'], self.settings['marker_size'],
            dict(zip(marker_ids, marker_sizes)), self.settings['max_reprojection_error'])
        positive_float(self.settings['axis_length'], 'axis_length')
        if not self.settings['input_frame'] or not self.settings['optical_frame']:
            raise ValueError('input_frame and optical_frame must name the source and optical TF frames')
        self.bridge = CvBridge()
        self.calibration = None
        self.info_frame = ''
        self.detections_pub = self.create_publisher(
            Detection3DArray, self.settings['detections_topic'], 1)
        self.ids_pub = self.create_publisher(Int32MultiArray, self.settings['ids_topic'], 1)
        self.poses_pub = self.create_publisher(PoseArray, self.settings['poses_topic'], 1)
        self.debug_pub = self.create_publisher(
            Image, self.settings['debug_image_topic'], qos_profile_sensor_data
        ) if self.settings['publish_debug_image'] else None
        self.tf_broadcaster = TransformBroadcaster(self) if self.settings['publish_tf'] else None
        self.info_sub = self.create_subscription(
            CameraInfo, self.settings['camera_info_topic'], self.on_camera_info,
            qos_profile_sensor_data)
        self.image_sub = self.create_subscription(
            Image, self.settings['image_topic'], self.on_image, qos_profile_sensor_data)
        self.get_logger().info(
            f"Detecting {self.settings['dictionary']} on {self.image_sub.topic_name}; "
            f"calibration: {self.info_sub.topic_name}; input frame: {self.settings['input_frame']}; "
            f"optical frame: {self.settings['optical_frame']}")

    def warn(self, message):
        self.get_logger().warning(message, throttle_duration_sec=5.0)

    def on_camera_info(self, message):
        # Invalidate the previous calibration on any bad update, rather than
        # accidentally reusing it after a camera mode or calibration change.
        self.calibration = None
        self.info_frame = message.header.frame_id
        try:
            if (message.binning_x > 1 or message.binning_y > 1
                    or message.roi.x_offset or message.roi.y_offset
                    or message.roi.width not in (0, message.width)
                    or message.roi.height not in (0, message.height)):
                raise ValueError('Binned or cropped CameraInfo is unsupported; use full-frame RGB')
            self.calibration = Calibration.from_values(
                message.k, message.d, message.width, message.height, message.distortion_model)
        except ValueError as error:
            self.warn(str(error))

    def on_image(self, message):
        calibration = self.calibration
        reason = None
        if calibration is None:
            reason = 'Waiting for valid CameraInfo; publishing empty pose observations'
        elif (message.header.frame_id != self.settings['input_frame']
                or message.header.frame_id != self.info_frame):
            reason = 'Image and CameraInfo must both use the configured input_frame'
        elif (message.width, message.height) != (calibration.width, calibration.height):
            reason = 'Image size differs from CameraInfo; publishing empty pose observations'
        if reason:
            self.warn(reason)
            calibration = None
        observations = []
        try:
            image = self.bridge.imgmsg_to_cv2(message, desired_encoding='bgr8')
            corners, ids, observations = self.detector.detect(image, calibration)
            if self.debug_pub is not None:
                debug = image.copy()
                if ids is not None:
                    cv2.aruco.drawDetectedMarkers(debug, corners, ids)
                for observation in observations:
                    cv2.drawFrameAxes(
                        debug, calibration.matrix, calibration.distortion,
                        observation.rvec, observation.tvec, self.settings['axis_length'])
                if reason:
                    cv2.putText(debug, 'Pose unavailable: calibration/frame/size', (8, 22),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 1)
                debug_message = self.bridge.cv2_to_imgmsg(debug, encoding='bgr8')
                debug_message.header = self.optical_header(message.header)
                self.debug_pub.publish(debug_message)
        except (CvBridgeError, cv2.error, ValueError) as error:
            self.warn(f'Cannot process camera image: {error}')
            observations = []
        self.publish_observations(message.header, observations)

    def optical_header(self, source_header):
        # OpenCV's pinhole projection and solvePnP always use optical axes,
        # independently of the frame label carried by the source 2D image.
        # Gazebo Harmonic labels RGB, calibration, and native X-forward points
        # with the sensor frame. Do not change that shared sensor setting.
        return Header(stamp=source_header.stamp, frame_id=self.settings['optical_frame'])

    def publish_observations(self, header, observations):
        header = self.optical_header(header)
        detections = Detection3DArray(header=header)
        poses = PoseArray(header=header)
        ids = Int32MultiArray()
        transforms = []
        for observation in observations:
            pose = Pose()
            pose.position.x, pose.position.y, pose.position.z = map(float, observation.tvec.ravel())
            quaternion = rotation_quaternion(observation.rvec)
            (pose.orientation.x, pose.orientation.y,
             pose.orientation.z, pose.orientation.w) = map(float, quaternion)
            result = ObjectHypothesisWithPose()
            result.hypothesis.class_id = str(observation.marker_id)
            result.hypothesis.score = 1.0  # Successful code decode; not a pose confidence.
            result.pose.pose = pose
            detection = Detection3D(header=header, results=[result])
            detection.id = f"{self.settings['dictionary']}/{observation.marker_id}"
            detection.bbox.center = pose
            detection.bbox.size.x = observation.size
            detection.bbox.size.y = observation.size
            detections.detections.append(detection)
            poses.poses.append(pose)
            ids.data.append(observation.marker_id)
            if self.tf_broadcaster is not None:
                transform = TransformStamped(header=header)
                transform.child_frame_id = f"{self.settings['tf_child_prefix']}{observation.marker_id}"
                transform.transform.translation.x = pose.position.x
                transform.transform.translation.y = pose.position.y
                transform.transform.translation.z = pose.position.z
                transform.transform.rotation = pose.orientation
                transforms.append(transform)
        # Every image produces a fresh observation, including an empty one when
        # the marker is lost. Never retain detections from a previous image.
        self.detections_pub.publish(detections)
        self.ids_pub.publish(ids)
        self.poses_pub.publish(poses)
        if transforms:
            self.tf_broadcaster.sendTransform(transforms)


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = ArucoDetectorNode()
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
