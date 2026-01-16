#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseWithCovarianceStamped
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy

class TestSub(Node):
    def __init__(self):
        super().__init__('test_sub')

        qos = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL
        )

        self.create_subscription(
            PoseWithCovarianceStamped,
            '/robot2/amcl_pose',
            self.cb,
            qos
        )

        self.get_logger().info("Test subscriber started (QoS matched AMCL)")

    def cb(self, msg):
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        self.get_logger().info(f"AMCL Pose -> x={x:.3f}, y={y:.3f}")

def main():
    rclpy.init()
    node = TestSub()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
