#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import yaml
import sys
from ament_index_python.packages import get_package_share_directory
import os

def detect_available_cameras(max_devices=10):
    available = []
    for i in range(max_devices):
        dev = f'/dev/video{i}'
        cap = cv2.VideoCapture(dev)
        if cap.isOpened():
            available.append(dev)
            cap.release()
    return available

class CameraPublisher(Node):
    def __init__(self):
        super().__init__('camera_publisher')

        pkg_share = get_package_share_directory('camera_viewer')
        config_path = os.path.join(pkg_share, 'config', 'cameras.yaml')
        self.get_logger().info(f"Loading config from: {config_path}") # DEBUG

        with open(config_path, 'r') as file:
            config = yaml.safe_load(file)
        self.get_logger().info(f"YAML content loaded: {config}") # DEBUG

        params = config.get('camera_viewer', {}).get('ros__parameters', {})

        default_index = params.get('selected_camera', 0)
        self.declare_parameter('selected_camera', default_index)
        selected_index = self.get_parameter('selected_camera').get_parameter_value().integer_value

        self.camera_sources = detect_available_cameras()
        self.get_logger().info(f"Available cameras: {self.camera_sources}")

        if not self.camera_sources:
            self.get_logger().error('No working camera devices found.')
            sys.exit(1)

        if selected_index >= len(self.camera_sources):
            self.get_logger().error('Selected camera index out of range.')
            sys.exit(1)

        self.selected_camera = self.camera_sources[selected_index]
        self.get_logger().info(f'Using camera: {self.selected_camera}')

        # Open camera
        self.cap = cv2.VideoCapture(self.selected_camera)
        if not self.cap.isOpened():
            self.get_logger().error(f'Failed to open camera: {self.selected_camera}')
            sys.exit(1)

        self.bridge = CvBridge()
        self.publisher = self.create_publisher(Image, 'image_raw', 10)
        self.timer = self.create_timer(0.03, self.publish_frame)  # ~30 FPS

    def publish_frame(self):
        ret, frame = self.cap.read()
        if not ret:
            self.get_logger().warning('Failed to read frame from camera')
            return

        msg = self.bridge.cv2_to_imgmsg(frame, encoding='bgr8')
        self.publisher.publish(msg)

    def destroy_node(self):
        self.cap.release()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = CameraPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
