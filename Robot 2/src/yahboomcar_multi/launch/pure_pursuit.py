#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, Twist
import math
import time

class SimplePurePursuit(Node):
    def __init__(self):
        super().__init__('robot2_simple_pure_pursuit')
        
        # Parameters
        self.max_linear_vel = 0.25
        self.max_angular_vel = 0.8
        self.goal_tolerance = 0.5
        self.k_p_angular = 1.5
        
        # State
        self.robot_x = 0.0
        self.robot_y = 0.0
        self.robot_yaw = 0.0
        self.target_x = None
        self.target_y = None
        self.has_pose = False
        
        # Publisher
        self.cmd_vel_pub = self.create_publisher(Twist, '/robot2/cmd_vel', 10)
        
        # === FIX: Subscribe dengan cara yang berbeda ===
        # 1. Untuk goal (sudah bekerja)
        self.goal_sub = self.create_subscription(
            PoseStamped,
            '/robot2/goal_pose',
            self.goal_callback,
            10
        )
        
        # 2. Untuk pose - coba ODOMETRY karena AMCL tidak bekerja
        from nav_msgs.msg import Odometry
        self.odom_sub = self.create_subscription(
            Odometry,
            '/robot2/odom_raw',  # Coba ganti ke /robot2/odometry/filtered jika tidak ada
            self.odom_callback,
            10
        )
        self.get_logger().info("Using /robot2/odom_raw for robot position")
        
        # Control loop
        self.create_timer(0.1, self.control_loop)
        
        # Status timer
        self.create_timer(1.0, self.print_status)
        
        self.get_logger().info("🤖 SIMPLE PURE PURSUIT: Ready!")
        
        # Force move setelah 3 detik jika tidak ada data
        self.create_timer(3.0, self.force_move_if_no_data)
    
    def odom_callback(self, msg):
        """Get robot position from odometry"""
        self.robot_x = msg.pose.pose.position.x
        self.robot_y = msg.pose.pose.position.y
        
        # Get yaw from quaternion
        q = msg.pose.pose.orientation
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        self.robot_yaw = math.atan2(siny_cosp, cosy_cosp)
        
        self.has_pose = True
        
        if not hasattr(self, 'pose_received'):
            self.pose_received = True
            self.get_logger().info(f"✅ ROBOT POSITION: ({self.robot_x:.2f}, {self.robot_y:.2f})")
    
    def goal_callback(self, msg):
        """Get target position"""
        self.target_x = msg.pose.position.x
        self.target_y = msg.pose.position.y
        
        self.get_logger().info(f"🎯 TARGET: ({self.target_x:.2f}, {self.target_y:.2f})")
        
        if self.has_pose:
            distance = math.sqrt((self.target_x - self.robot_x)**2 + 
                               (self.target_y - self.robot_y)**2)
            self.get_logger().info(f"  Distance: {distance:.2f}m")
    
    def control_loop(self):
        """Simple go-to-goal controller"""
        if not self.has_pose or self.target_x is None:
            return
        
        # Calculate distance to target
        dx = self.target_x - self.robot_x
        dy = self.target_y - self.robot_y
        distance = math.sqrt(dx*dx + dy*dy)
        
        # Check if reached
        if distance < self.goal_tolerance:
            self.stop_robot()
            return
        
        # Calculate angle error
        target_angle = math.atan2(dy, dx)
        angle_error = target_angle - self.robot_yaw
        
        # Normalize angle
        while angle_error > math.pi:
            angle_error -= 2.0 * math.pi
        while angle_error < -math.pi:
            angle_error += 2.0 * math.pi
        
        # Calculate velocities
        # Linear: based on distance
        if distance > 1.0:
            linear_vel = self.max_linear_vel
        elif distance > 0.3:
            linear_vel = self.max_linear_vel * 0.7
        else:
            linear_vel = 0.1
        
        # Angular: proportional control
        angular_vel = self.k_p_angular * angle_error
        
        # Limit angular velocity
        if angular_vel > self.max_angular_vel:
            angular_vel = self.max_angular_vel
        elif angular_vel < -self.max_angular_vel:
            angular_vel = -self.max_angular_vel
        
        # Reduce linear speed if turning sharply
        if abs(angle_error) > 0.5:
            linear_vel *= 0.5
        
        # Publish command
        cmd_vel = Twist()
        cmd_vel.linear.x = linear_vel
        cmd_vel.angular.z = angular_vel
        
        self.cmd_vel_pub.publish(cmd_vel)
        
        # Log every 0.5 seconds
        if not hasattr(self, 'last_log') or time.time() - self.last_log > 0.5:
            self.get_logger().info(
                f"⚡ V={linear_vel:.2f}, ω={angular_vel:.2f}, "
                f"Dist={distance:.2f}m, Err={math.degrees(angle_error):.1f}°"
            )
            self.last_log = time.time()
    
    def force_move_if_no_data(self):
        """Force move jika tidak ada data"""
        if not self.has_pose:
            self.get_logger().warn("⚠️ No pose data - FORCING MOVEMENT!")
            
            # Move toward last known target or just forward
            cmd_vel = Twist()
            if self.target_x is not None:
                # Try to go toward target even without pose
                cmd_vel.linear.x = 0.15
                cmd_vel.angular.z = 0.0
            else:
                # Just move forward
                cmd_vel.linear.x = 0.1
                cmd_vel.angular.z = 0.0
            
            self.cmd_vel_pub.publish(cmd_vel)
            self.get_logger().info(f"🚀 FORCE MOVE: linear={cmd_vel.linear.x:.2f}")
    
    def print_status(self):
        """Print simple status"""
        if self.has_pose:
            pose_info = f"📍 Robot: ({self.robot_x:.2f}, {self.robot_y:.2f})"
        else:
            pose_info = "📍 No pose"
        
        if self.target_x is not None:
            goal_info = f"🎯 Target: ({self.target_x:.2f}, {self.target_y:.2f})"
        else:
            goal_info = "🎯 No target"
        
        self.get_logger().info(f"{pose_info} | {goal_info}")
    
    def stop_robot(self):
        """Stop robot"""
        cmd_vel = Twist()
        self.cmd_vel_pub.publish(cmd_vel)

def main():
    rclpy.init()
    node = SimplePurePursuit()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()