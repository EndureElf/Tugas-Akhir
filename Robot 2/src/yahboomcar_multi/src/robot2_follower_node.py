#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, PoseWithCovarianceStamped
import math

class Robot2Follower(Node):
    def __init__(self):
        super().__init__("robot2_follower")

        self.declare_parameter("follow_distance", 0.6)
        self.declare_parameter("linear_gain", 0.6)
        self.declare_parameter("angular_gain", 1.0)

        self.follow_distance = self.get_parameter("follow_distance").value
        self.linear_gain = self.get_parameter("linear_gain").value
        self.angular_gain = self.get_parameter("angular_gain").value

        self.leader_pose = None
        self.my_pose = None

        self.cmd_pub = self.create_publisher(Twist, "/robot2/cmd_vel", 10)

        # Subscribe posisi leader (robot1)
        self.create_subscription(
            PoseWithCovarianceStamped,
            "/robot1/amcl_pose",
            self.leader_pose_callback,
            10
        )

        # Subscribe posisi robot2 sendiri
        self.create_subscription(
            PoseWithCovarianceStamped,
            "/robot2/amcl_pose",
            self.my_pose_callback,
            10
        )

        # Loop kontrol
        self.timer = self.create_timer(0.05, self.control_loop)
        self.get_logger().info("Robot2 follower node started")

    def leader_pose_callback(self, msg):
        self.leader_pose = msg.pose.pose

    def my_pose_callback(self, msg):
        self.my_pose = msg.pose.pose

    def control_loop(self):
        if self.leader_pose is None or self.my_pose is None:
            return

        dx = self.leader_pose.position.x - self.my_pose.position.x
        dy = self.leader_pose.position.y - self.my_pose.position.y

        distance = math.sqrt(dx*dx + dy*dy)
        angle_to_leader = math.atan2(dy, dx)

        # Buat Twist command
        cmd = Twist()

        # Jarak maju
        cmd.linear.x = self.linear_gain * (distance - self.follow_distance)

        # Putaran arah ke leader
        cmd.angular.z = self.angular_gain * angle_to_leader

        # Batasi max speed
        cmd.linear.x = max(min(cmd.linear.x, 0.3), -0.3)
        cmd.angular.z = max(min(cmd.angular.z, 1.2), -1.2)

        self.cmd_pub.publish(cmd)

def main(args=None):
    rclpy.init(args=args)
    node = Robot2Follower()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
