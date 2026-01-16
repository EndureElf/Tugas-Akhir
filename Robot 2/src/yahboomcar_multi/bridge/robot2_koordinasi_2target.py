#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
import socket
import json
import threading

class Robot2Bridge(Node):
    def __init__(self):
        super().__init__('robot2_bridge')
        
        # UDP socket untuk TERIMA dari Laptop
        self.receive_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.receive_sock.bind(('0.0.0.0', 8891))  # ⬅️ PORT BERBEDA: 8891
        
        # Publisher untuk goal ke Nav2 Robot 2
        self.goal_pub = self.create_publisher(
            PoseStamped,
            '/robot2/goal_pose',  # ⬅️ TOPIC UNTUK NAV2 ROBOT 2
            10
        )
        
        # Thread untuk listen dari laptop
        self.receive_thread = threading.Thread(target=self.receive_from_laptop)
        self.receive_thread.daemon = True
        self.receive_thread.start()
        
        self.get_logger().info('🤖 ROBOT 2 BRIDGE READY')
        self.get_logger().info('  - Listening on UDP port 8891')
        self.get_logger().info('  - Publishing to /robot2/goal_pose')
        self.get_logger().info('  - Waiting for goals from Laptop...')

    def receive_from_laptop(self):
        """Terima goal dari laptop dan publish ke Nav2 Robot 2"""
        while rclpy.ok():
            try:
                data, addr = self.receive_sock.recvfrom(1024)
                self.get_logger().info(f'📥 From Laptop: {len(data)} bytes')
                
                goal_data = json.loads(data.decode())
                
                # Convert to ROS message
                goal_msg = PoseStamped()
                goal_msg.header.stamp = self.get_clock().now().to_msg()
                goal_msg.header.frame_id = goal_data['frame_id']
                
                # Position
                pos = goal_data['goal_pose']['position']
                goal_msg.pose.position.x = pos['x']
                goal_msg.pose.position.y = pos['y']
                goal_msg.pose.position.z = pos['z']
                
                # Orientation
                orient = goal_data['goal_pose']['orientation']
                goal_msg.pose.orientation.x = orient['x']
                goal_msg.pose.orientation.y = orient['y']
                goal_msg.pose.orientation.z = orient['z']
                goal_msg.pose.orientation.w = orient['w']
                
                # Publish to Nav2 Robot 2
                self.goal_pub.publish(goal_msg)
                
                self.get_logger().info(f'🎯 To Nav2 Robot2: ({goal_msg.pose.position.x:.2f}, {goal_msg.pose.position.y:.2f})')
                self.get_logger().info('🤖 ROBOT 2 SHOULD MOVE NOW!')
                
            except Exception as e:
                self.get_logger().error(f'Error: {str(e)}')

def main():
    rclpy.init()
    node = Robot2Bridge()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()