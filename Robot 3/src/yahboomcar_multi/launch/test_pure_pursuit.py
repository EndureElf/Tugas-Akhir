#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, Twist, PoseWithCovarianceStamped
import math
import time

class TestPurePursuit(Node):
    def __init__(self):
        super().__init__('test_pure_pursuit')
        
        # ========== TEST VARIABLES ==========
        self.test_goal = None
        self.test_pose = None
        
        # ========== PUBLISHERS ==========
        self.pose_pub = self.create_publisher(PoseWithCovarianceStamped, '/robot2/amcl_pose', 10)
        self.goal_pub = self.create_publisher(PoseStamped, '/robot2/goal_pose', 10)
        self.cmd_vel_pub = self.create_publisher(Twist, '/robot2/cmd_vel', 10)
        
        # ========== TIMERS ==========
        # Publish test pose (robot2 at origin)
        self.create_timer(0.1, self.publish_test_pose)
        
        # Publish test goal (2m ahead)
        self.create_timer(3.0, self.publish_test_goal)
        
        # Monitor
        self.create_timer(1.0, self.monitor)
        
        self.get_logger().info("🧪 TEST PURE PURSUIT: Publishing test data...")
        
    def publish_test_pose(self):
        """Publish fake robot2 pose at (0, 0)"""
        msg = PoseWithCovarianceStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'map'
        
        # Robot2 di origin
        msg.pose.pose.position.x = 0.0
        msg.pose.pose.position.y = 0.0
        msg.pose.pose.position.z = 0.0
        
        # Facing positive X
        msg.pose.pose.orientation.w = 1.0
        
        self.pose_pub.publish(msg)
        self.test_pose = msg
        
    def publish_test_goal(self):
        """Publish test goal 2m di depan"""
        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'map'
        
        # Goal 2m di depan robot2
        msg.pose.position.x = 2.0
        msg.pose.position.y = 0.0
        msg.pose.position.z = 0.0
        msg.pose.orientation.w = 1.0
        
        self.goal_pub.publish(msg)
        self.test_goal = msg
        self.get_logger().info(f"📤 Published test goal: ({msg.pose.position.x}, {msg.pose.position.y})")
        
    def monitor(self):
        """Monitor status"""
        if self.test_pose and self.test_goal:
            dx = self.test_goal.pose.position.x - self.test_pose.pose.pose.position.x
            dy = self.test_goal.pose.position.y - self.test_pose.pose.pose.position.y
            distance = math.sqrt(dx*dx + dy*dy)
            
            self.get_logger().info(f"📊 Test: Robot2 at (0,0), Goal at (2,0), Distance: {distance:.2f}m")
            
            # Juga publish cmd_vel manual untuk test
            cmd = Twist()
            cmd.linear.x = 0.15
            cmd.angular.z = 0.0
            self.cmd_vel_pub.publish(cmd)

def main():
    rclpy.init()
    node = TestPurePursuit()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()