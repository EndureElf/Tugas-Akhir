#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
import socket
import json
import threading

class Robot2GoalReceiver(Node):
    def __init__(self):
        super().__init__('robot2_goal_receiver')
        
        # Publisher untuk goal pose (ke navigation stack Robot 2)
        self.goal_publisher = self.create_publisher(
            PoseStamped,
            '/robot2/goal_pose',  # ✅ TOPIC YANG DIPAKAI NAVIGATION ROBOT 2
            10
        )
        
        # UDP socket untuk receive dari Laptop
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.bind(('0.0.0.0', 8891))  # Port berbeda dari Robot 1
        
        # Thread untuk listen UDP
        self.udp_thread = threading.Thread(target=self.udp_listener)
        self.udp_thread.daemon = True
        self.udp_thread.start()
        
        self.get_logger().info("🤖 ROBOT2 GOAL RECEIVER: Listening on port 8891...")

    def udp_listener(self):
        """Receive goals from Laptop"""
        while rclpy.ok():
            try:
                data, addr = self.sock.recvfrom(1024)
                goal_data = json.loads(data.decode())
                
                # Convert to ROS message
                goal_msg = PoseStamped()
                goal_msg.header.stamp = self.get_clock().now().to_msg()
                goal_msg.header.frame_id = goal_data['frame_id']
                
                # Position
                goal_msg.pose.position.x = goal_data['goal_pose']['position']['x']
                goal_msg.pose.position.y = goal_data['goal_pose']['position']['y']
                goal_msg.pose.position.z = goal_data['goal_pose']['position']['z']
                
                # Orientation
                goal_msg.pose.orientation.x = goal_data['goal_pose']['orientation']['x']
                goal_msg.pose.orientation.y = goal_data['goal_pose']['orientation']['y']
                goal_msg.pose.orientation.z = goal_data['goal_pose']['orientation']['z']
                goal_msg.pose.orientation.w = goal_data['goal_pose']['orientation']['w']
                
                # Publish to navigation stack Robot 2
                self.goal_publisher.publish(goal_msg)
                
                self.get_logger().info(
                    f'🎯 Goal Received: x={goal_msg.pose.position.x:.2f}, y={goal_msg.pose.position.y:.2f} from {addr[0]}'
                )
                
            except Exception as e:
                self.get_logger().error(f'UDP error: {str(e)}')

def main():
    rclpy.init()
    node = Robot2GoalReceiver()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
