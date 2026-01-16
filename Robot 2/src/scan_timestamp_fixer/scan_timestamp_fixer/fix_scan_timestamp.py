import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan

class ScanTimestampFixer(Node):
    def __init__(self):
        super().__init__('scan_timestamp_fixer')
        
        # Input dari driver lidar
        self.sub = self.create_subscription(
            LaserScan,
            '/robot2/scan',
            self.scan_callback,
            10)
        
        # Output scan yang sudah diperbaiki
        self.pub = self.create_publisher(
            LaserScan,
            '/robot2/scan_fixed',
            10)
        
        self.get_logger().info("Scan Timestamp Fixer aktif.")

    def scan_callback(self, msg):
        # Perbaiki timestamp ke ROS time
        msg.header.stamp = self.get_clock().now().to_msg()

        # Publish ulang
        self.pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = ScanTimestampFixer()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
