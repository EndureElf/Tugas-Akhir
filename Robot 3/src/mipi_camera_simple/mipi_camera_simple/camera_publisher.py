#!/usr/bin/env python3
"""
Script untuk mempublikasikan gambar dari kamera MIPI RDK
Publisher ke topic: /mipi_camera/image_raw/compressed
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage
import cv2
from cv_bridge import CvBridge
import numpy as np

try:
    from hobot_vio import libsrcampy as srcampy
    HOBOT_AVAILABLE = True
except ImportError:
    print("WARNING: hobot_vio not found! Using dummy mode.")
    HOBOT_AVAILABLE = False


class MipiCamera:
    """Class untuk mengakses kamera MIPI pada RDK"""

    def __init__(self, width=640, height=480, debug=False):
        self.debug = debug
        self.state = False
        self.width = width
        self.height = height

        if not HOBOT_AVAILABLE:
            if self.debug:
                print("Using dummy camera (hobot_vio not available)")
            self.state = False
            return

        self.camera = srcampy.Camera()
        # open_cam(pipe_id, video_index, fps, width, height)
        # pipe_id=1 untuk MIPI camera
        error = self.camera.open_cam(1, -1, 30, self.width, self.height)

        if error == 0:
            self.state = True
            if self.debug:
                print(f"Camera opened successfully! Resolution: {self.width}x{self.height}")
        else:
            self.state = False
            if self.debug:
                print(f"Failed to open camera! Error code: {error}")

    def __del__(self):
        self.release()
        if self.debug:
            print("Camera released")

    def nv12_to_bgr(self, image):
        """Convert NV12 format to BGR"""
        frame = np.frombuffer(image, dtype=np.uint8)
        # RDK X3 camera output is 1920x1080 in NV12 format
        img_bgr = cv2.cvtColor(frame.reshape(1620, 1920), cv2.COLOR_YUV2BGR_NV12)
        # Resize to desired resolution
        if img_bgr.shape[0] != self.height or img_bgr.shape[1] != self.width:
            img_bgr = cv2.resize(img_bgr, (self.width, self.height))
        return img_bgr

    def isOpened(self):
        """Check if camera is opened"""
        return self.state

    def read(self):
        """Read a frame from camera"""
        if not self.state:
            return False, None

        try:
            # Mode 2: get image in NV12 format
            image = self.camera.get_img(2)
            if image is None:
                return False, None

            image_bgr = self.nv12_to_bgr(image)
            return True, image_bgr
        except Exception as e:
            if self.debug:
                print(f"Error reading frame: {e}")
            return False, None

    def release(self):
        """Release camera"""
        if self.state and HOBOT_AVAILABLE:
            self.camera.close_cam()
            self.state = False


class CameraPublisher(Node):
    """ROS2 Node untuk mempublikasikan gambar dari kamera MIPI"""

    def __init__(self):
        super().__init__('mipi_camera_publisher')

        # Declare parameters
        self.declare_parameter('width', 640)
        self.declare_parameter('height', 480)
        self.declare_parameter('fps', 30)
        self.declare_parameter('debug', True)

        # Get parameters
        width = self.get_parameter('width').value
        height = self.get_parameter('height').value
        fps = self.get_parameter('fps').value
        debug = self.get_parameter('debug').value

        # Create publisher
        self.publisher = self.create_publisher(
            CompressedImage,
            '/mipi_camera/image_raw/compressed',
            10
        )

        # Create CV Bridge
        self.bridge = CvBridge()

        # Initialize camera
        self.camera = MipiCamera(width=width, height=height, debug=debug)

        if not self.camera.isOpened():
            self.get_logger().error('Failed to open camera!')
            return

        self.get_logger().info(f'Camera opened! Publishing at {fps} FPS to /mipi_camera/image_raw/compressed')

        # Create timer to publish images
        timer_period = 1.0 / fps
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.frame_count = 0

    def timer_callback(self):
        """Callback to publish camera frame"""
        ret, frame = self.camera.read()

        if ret and frame is not None:
            try:
                # Convert to compressed image message
                compressed_msg = self.bridge.cv2_to_compressed_imgmsg(frame, dst_format='jpeg')
                compressed_msg.header.stamp = self.get_clock().now().to_msg()
                compressed_msg.header.frame_id = 'camera_frame'

                # Publish
                self.publisher.publish(compressed_msg)

                self.frame_count += 1
                if self.frame_count % 30 == 0:
                    self.get_logger().info(f'Published {self.frame_count} frames')

            except Exception as e:
                self.get_logger().error(f'Error publishing frame: {e}')
        else:
            self.get_logger().warn('Failed to read frame from camera')

    def destroy_node(self):
        """Cleanup when node is destroyed"""
        self.camera.release()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)

    try:
        node = CameraPublisher()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            node.destroy_node()
            rclpy.shutdown()


if __name__ == '__main__':
    main()
