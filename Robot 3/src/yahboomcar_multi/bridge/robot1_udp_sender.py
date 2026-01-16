#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

from geometry_msgs.msg import PoseWithCovarianceStamped, PoseStamped, Twist
from nav_msgs.msg import Path

from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy

import socket
import json
import math


# =========================
# UTIL
# =========================
def quat_to_yaw(q):
    return math.atan2(
        2.0 * (q.w * q.z + q.x * q.y),
        1.0 - 2.0 * (q.y*q.y + q.z*q.z)
    )


# =========================
# NODE
# =========================
class Robot1UDPSender(Node):
    def __init__(self):
        super().__init__('robot1_udp_sender')

        # =========================
        # UDP CONFIG
        # =========================
        self.udp_ip   = '192.168.0.88'   # IP ROBOT 2 (UBAH JIKA PERLU)
        self.udp_port = 5005
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

        # =========================
        # DATA CACHE
        # =========================
        self.leader_pose = None
        self.leader_cmd  = None
        self.goal_pose   = None
        self.waypoint    = None

        self.global_path = None
        self.path_sent   = False   # hanya kirim PATH PERTAMA

        # =========================
        # QoS PROFILES
        # =========================
        amcl_qos = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL
        )

        goal_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE
        )

        plan_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE
        )

        # =========================
        # SUBSCRIBERS
        # =========================
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

        self.create_subscription(
            PoseStamped,
            '/robot1/goal_pose',
            self.goal_cb,
            goal_qos
        )

        self.create_subscription(
            PoseStamped,
            '/robot3/goal_pose',
            self.waypoint_cb,
            goal_qos
        )

        self.create_subscription(
            Path,
            '/robot1/plan',
            self.plan_cb,
            plan_qos
        )

        # =========================
        # TIMER (UDP SEND)
        # =========================
        self.create_timer(0.05, self.send_udp)  # 20 Hz

        self.get_logger().info('Robot1 UDP Sender READY')

    # =========================
    # CALLBACKS
    # =========================
    def amcl_cb(self, msg):
        p = msg.pose.pose
        self.leader_pose = {
            't': msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9,
            'x': p.position.x,
            'y': p.position.y,
            'th': quat_to_yaw(p.orientation)
        }

    def cmd_cb(self, msg):
        self.leader_cmd = {
            'vx': msg.linear.x,
            'vy': msg.linear.y,
            'w' : msg.angular.z
        }

    def goal_cb(self, msg):
        p = msg.pose
        self.goal_pose = {
            'x': p.position.x,
            'y': p.position.y,
            'th': quat_to_yaw(p.orientation)
        }

    def waypoint_cb(self, msg):
        p = msg.pose
        self.waypoint = {
            'x': p.position.x,
            'y': p.position.y,
            'th': quat_to_yaw(p.orientation)
        }

    def plan_cb(self, msg):
        # SIMPAN & KIRIM GLOBAL PATH PERTAMA SAJA
        if self.path_sent:
            return

        path = []
        for i, pose in enumerate(msg.poses):
            path.append({
                'idx': i,
                'x': pose.pose.position.x,
                'y': pose.pose.position.y
            })

        self.global_path = path
        self.path_sent = True

        self.get_logger().info(
            f'Global path initial received ({len(path)} points)'
        )

    # =========================
    # UDP SEND
    # =========================
    def send_udp(self):
        if self.leader_pose is None:
            return

        packet = {
            'pose': self.leader_pose,
            'cmd': self.leader_cmd,
            'goal': self.goal_pose,
            'waypoint': self.waypoint,
            'path': self.global_path   # None kalau belum diterima
        }

        try:
            self.sock.sendto(
                json.dumps(packet).encode(),
                (self.udp_ip, self.udp_port)
            )
        except Exception as e:
            self.get_logger().warn(f'UDP send failed: {e}')


# =========================
# MAIN
# =========================
def main():
    rclpy.init()
    node = Robot1UDPSender()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
