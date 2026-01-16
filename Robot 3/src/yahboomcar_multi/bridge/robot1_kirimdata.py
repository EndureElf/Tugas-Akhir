#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseWithCovarianceStamped, Twist
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
import socket
import json
import math


class Robot1UDPSender(Node):

    def __init__(self):
        super().__init__('robot1_udp_sender')

        self.robot2_ip = '192.168.0.88'   # GANTI IP ROBOT2
        self.robot2_port = 8891

        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

        self.leader_pose = None
        self.leader_cmd = None

        amcl_qos = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL
        )        

        self.create_subscription(
            PoseWithCovarianceStamped,
            '/robot1/amcl_pose',
            self.amcl_cb,
            amcl_qos
        )

        self.create_subscription(
            Twist,
            '/robot1/cmd_vel',
            self.cmd_cb,
            10
        )

        self.timer = self.create_timer(0.02, self.send_udp)  # 50 Hz

        self.get_logger().info("Robot1 UDP sender started")

    def amcl_cb(self, msg):
        pose = msg.pose.pose
        yaw = self.yaw_from_q(pose.orientation)

        self.leader_pose = {
            "x": pose.position.x,
            "y": pose.position.y,
            "yaw": yaw
        }

    def cmd_cb(self, msg):
        self.leader_cmd = {
            "vx": msg.linear.x,
            "vy": msg.linear.y,
            "w":  msg.angular.z
        }

    def send_udp(self):

        if self.leader_pose is None:
            return

        data = {
            "leader_pose": self.leader_pose
        }

        # feedforward OPTIONAL
        if self.leader_cmd is not None:
            data["leader_cmd_vel"] = self.leader_cmd

        msg = json.dumps(data).encode('utf-8')
        self.sock.sendto(msg, (self.robot2_ip, self.robot2_port))

    def yaw_from_q(self, q):
        return math.atan2(
            2.0 * (q.w * q.z),
            1.0 - 2.0 * (q.z * q.z)
        )


def main():
    rclpy.init()
    node = Robot1UDPSender()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
