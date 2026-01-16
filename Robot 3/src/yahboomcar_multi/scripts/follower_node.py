import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
import math
import numpy as np

def yaw_from_quaternion(q):
    """Convert a quaternion to a yaw angle."""
    siny_cosp = 2 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)

class Follower(Node):
    def __init__(self):
        super().__init__('follower_node')

        # Declare parameters
        self.declare_parameter('leader_odom_topic', '/robot1/odom')
        self.declare_parameter('own_odom_topic', '/robot2/odom')
        self.declare_parameter('cmd_vel_topic', '/robot2/cmd_vel')
        self.declare_parameter('offset_x', -0.5)
        self.declare_parameter('offset_y', 0.3)
        self.declare_parameter('kp_linear_x', 1.0)
        self.declare_parameter('kp_linear_y', 1.0)
        self.declare_parameter('kp_angular_z', 1.5)
        self.declare_parameter('control_frequency', 10.0)

        # Get parameters
        leader_odom_topic = self.get_parameter('leader_odom_topic').get_parameter_value().string_value
        own_odom_topic = self.get_parameter('own_odom_topic').get_parameter_value().string_value
        cmd_vel_topic = self.get_parameter('cmd_vel_topic').get_parameter_value().string_value
        self.offset_x_ = self.get_parameter('offset_x').get_parameter_value().double_value
        self.offset_y_ = self.get_parameter('offset_y').get_parameter_value().double_value
        self.kp_linear_x_ = self.get_parameter('kp_linear_x').get_parameter_value().double_value
        self.kp_linear_y_ = self.get_parameter('kp_linear_y').get_parameter_value().double_value
        self.kp_angular_z_ = self.get_parameter('kp_angular_z').get_parameter_value().double_value
        control_frequency = self.get_parameter('control_frequency').get_parameter_value().double_value

        # Subscriptions
        self.sub_leader_odom_ = self.create_subscription(Odometry, leader_odom_topic, self.leader_odom_callback, 10)
        self.sub_own_odom_ = self.create_subscription(Odometry, own_odom_topic, self.own_odom_callback, 10)
        
        # Publisher
        self.pub_cmd_ = self.create_publisher(Twist, cmd_vel_topic, 10)

        # State
        self.leader_odom_ = None
        self.own_odom_ = None

        # Control loop timer
        self.timer_ = self.create_timer(1.0 / control_frequency, self.control_loop)
        
        self.get_logger().info(f"Follower node initialized. Following '{leader_odom_topic}' with offset ({self.offset_x_}, {self.offset_y_})")

    def leader_odom_callback(self, msg):
        self.leader_odom_ = msg

    def own_odom_callback(self, msg):
        self.own_odom_ = msg

    def control_loop(self):
        if self.leader_odom_ is None or self.own_odom_ is None:
            self.get_logger().warn("Waiting for odometry from leader and self...", throttle_duration_sec=5)
            return

        # Leader's pose
        leader_x = self.leader_odom_.pose.pose.position.x
        leader_y = self.leader_odom_.pose.pose.position.y
        leader_yaw = yaw_from_quaternion(self.leader_odom_.pose.pose.orientation)

        # Calculate target pose in the global frame
        # Rotate the offset by the leader's yaw and then add it to the leader's position
        target_x = leader_x + self.offset_x_ * math.cos(leader_yaw) - self.offset_y_ * math.sin(leader_yaw)
        target_y = leader_y + self.offset_x_ * math.sin(leader_yaw) + self.offset_y_ * math.cos(leader_yaw)
        target_yaw = leader_yaw # Follower should have the same orientation as the leader

        # Follower's current pose
        own_x = self.own_odom_.pose.pose.position.x
        own_y = self.own_odom_.pose.pose.position.y
        own_yaw = yaw_from_quaternion(self.own_odom_.pose.pose.orientation)

        # Calculate error in the follower's frame for better control
        error_global_x = target_x - own_x
        error_global_y = target_y - own_y
        
        # Rotate global error into the follower's local frame
        error_local_x = error_global_x * math.cos(-own_yaw) - error_global_y * math.sin(-own_yaw)
        error_local_y = error_global_x * math.sin(-own_yaw) + error_global_y * math.cos(-own_yaw)

        # Calculate yaw error
        error_yaw = target_yaw - own_yaw
        # Normalize yaw error to [-pi, pi]
        if error_yaw > math.pi:
            error_yaw -= 2 * math.pi
        if error_yaw < -math.pi:
            error_yaw += 2 * math.pi

        # Proportional control
        cmd_vel = Twist()
        cmd_vel.linear.x = self.kp_linear_x_ * error_local_x
        cmd_vel.linear.y = self.kp_linear_y_ * error_local_y  # For mecanum drive
        cmd_vel.angular.z = self.kp_angular_z_ * error_yaw

        self.pub_cmd_.publish(cmd_vel)

def main(args=None):
    rclpy.init(args=args)
    follower_node = Follower()
    rclpy.spin(follower_node)
    follower_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
