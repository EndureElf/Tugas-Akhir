#!/usr/bin/env python3
"""
Script untuk melihat gambar dari topic kamera
Subscribe dari topic: /mipi_camera/image_raw/compressed atau topic lainnya
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage
import cv2
from cv_bridge import CvBridge
import numpy as np


class CameraViewer(Node):
    """ROS2 Node untuk melihat gambar dari camera topic"""

    def __init__(self):
        super().__init__('camera_viewer')

        # Declare parameters
        self.declare_parameter('topic', '/mipi_camera/image_raw/compressed')
        self.declare_parameter('show_window', True)
        self.declare_parameter('save_images', False)
        self.declare_parameter('save_path', '/tmp/')

        # Get parameters
        topic = self.get_parameter('topic').value
        self.show_window = self.get_parameter('show_window').value
        self.save_images = self.get_parameter('save_images').value
        self.save_path = self.get_parameter('save_path').value

        # Create subscriber
        self.subscription = self.create_subscription(
            CompressedImage,
            topic,
            self.image_callback,
            10
        )

        # Create CV Bridge
        self.bridge = CvBridge()
        self.frame_count = 0

        self.get_logger().info(f'Subscribing to: {topic}')
        if self.show_window:
            self.get_logger().info('Press "q" to quit, "s" to save screenshot')

    def image_callback(self, msg):
        """Callback when image is received"""
        try:
            # Convert compressed image to OpenCV format
            np_arr = np.frombuffer(msg.data, np.uint8)
            image = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)

            if image is None:
                self.get_logger().warn('Failed to decode image')
                return

            self.frame_count += 1

            # Add info text on image
            info_text = f'Frame: {self.frame_count} | Size: {image.shape[1]}x{image.shape[0]}'
            cv2.putText(image, info_text, (10, 30),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)

            # Show window if enabled
            if self.show_window:
                cv2.imshow('MIPI Camera Viewer', image)
                key = cv2.waitKey(1) & 0xFF

                # Press 'q' to quit
                if key == ord('q'):
                    self.get_logger().info('Quitting...')
                    rclpy.shutdown()

                # Press 's' to save screenshot
                elif key == ord('s'):
                    filename = f'{self.save_path}/screenshot_{self.frame_count}.jpg'
                    cv2.imwrite(filename, image)
                    self.get_logger().info(f'Screenshot saved: {filename}')

            # Auto save if enabled
            if self.save_images:
                if self.frame_count % 30 == 0:  # Save every 30 frames
                    filename = f'{self.save_path}/frame_{self.frame_count}.jpg'
                    cv2.imwrite(filename, image)
                    self.get_logger().info(f'Frame saved: {filename}')

            # Log info every 30 frames
            if self.frame_count % 30 == 0:
                self.get_logger().info(f'Received {self.frame_count} frames')

        except Exception as e:
            self.get_logger().error(f'Error processing image: {e}')

    def destroy_node(self):
        """Cleanup when node is destroyed"""
        cv2.destroyAllWindows()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)

    try:
        node = CameraViewer()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            node.destroy_node()
            rclpy.shutdown()


if __name__ == '__main__':
    main()
