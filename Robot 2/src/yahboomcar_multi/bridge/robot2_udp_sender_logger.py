#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

from geometry_msgs.msg import PoseWithCovarianceStamped, Twist
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy

import socket
import json
import csv
import math
import os


# =========================
# UTIL
# =========================
def quat_to_yaw(q):
    return math.atan2(
        2.0 * (q.w*q.z + q.x*q.y),
        1.0 - 2.0 * (q.y*q.y + q.z*q.z)
    )


# =========================
# NODE
# =========================
class Robot2UDPSenderLogger(Node):

    def __init__(self):
        super().__init__('robot2_udp_sender_logger')

        # =========================
        # UDP CONFIG
        # =========================
        self.udp_port = 5005
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.bind(('', self.udp_port))
        self.sock.setblocking(False)

        # =========================
        # DATA CACHE
        # =========================
        self.leader_pose = None
        self.leader_cmd  = None

        self.follower_pose = None
        self.follower_cmd  = None

        self.global_path   = None   # LIST of poses[0..N]
        self.path_saved    = False  # KUNCI: hanya sekali

        self.goal_pose = None
        self.waypoint  = None

        # =========================
        # LOG DIRECTORY
        # =========================
        self.log_dir = os.path.expanduser('~/ros2_ws/log_data')
        os.makedirs(self.log_dir, exist_ok=True)

        # =========================
        # CSV FILES
        # =========================
        self.csv_sync = open(
            os.path.join(self.log_dir, 'leader_follower_sync.csv'),
            'w', newline=''
        )
        self.sync_writer = csv.writer(self.csv_sync)
        self.sync_writer.writerow([
            't',
            'x_lead','y_lead','th_lead',
            'x_fol','y_fol','th_fol',
            'vx_lead','vy_lead','w_lead',
            'vx_fol','vy_fol','w_fol'
        ])
    
        self.csv_path = open(
            os.path.join(self.log_dir, 'robot1_global_path_initial.csv'),
            'w', newline=''
        )
        self.path_writer = csv.writer(self.csv_path)
        self.path_writer.writerow(['idx','x','y'])

        self.csv_goal = open(
            os.path.join(self.log_dir, 'robot1_goal_pose.csv'),
            'w', newline=''
        )
        self.goal_writer = csv.writer(self.csv_goal)
        self.goal_writer.writerow(['x','y','th'])

        self.csv_wp = open(
            os.path.join(self.log_dir, 'robot3_waypoint.csv'),
            'w', newline=''
        )
        self.wp_writer = csv.writer(self.csv_wp)
        self.wp_writer.writerow(['x','y','th'])

        # =========================
        # QoS AMCL
        # =========================
        amcl_qos = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL
        )

        # =========================
        # SUBSCRIBERS (ROBOT 2)
        # =========================
        self.create_subscription(
            PoseWithCovarianceStamped,
            '/robot2/amcl_pose',
            self.follower_pose_cb,
            amcl_qos
        )

        self.create_subscription(
            Twist,
            '/robot2/cmd_vel',
            self.follower_cmd_cb,
            10
        )

        # =========================
        # TIMER
        # =========================
        self.create_timer(0.05, self.update)  # 20 Hz

        self.get_logger().info('Robot2 UDP Sender Logger READY')

    # =========================
    # CALLBACKS
    # =========================
    def follower_pose_cb(self, msg):
        p = msg.pose.pose
        self.follower_pose = {
            't': msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9,
            'x': p.position.x,
            'y': p.position.y,
            'th': quat_to_yaw(p.orientation)
        }

    def follower_cmd_cb(self, msg):
        self.follower_cmd = {
            'vx': msg.linear.x,
            'vy': msg.linear.y,
            'w' : msg.angular.z
        }

    # =========================
    # MAIN LOOP
    # =========================
    def update(self):

        # ===== UDP RECEIVE =====
        try:
            data, _ = self.sock.recvfrom(65536)
            pkt = json.loads(data.decode())

            self.leader_pose = pkt.get('pose', None)
            self.leader_cmd  = pkt.get('cmd', None)
            self.goal_pose   = pkt.get('goal', None)
            self.waypoint    = pkt.get('waypoint', None)

            # PATH = LIST poses[0..N]
            if pkt.get('path', None) and not self.path_saved:
                self.global_path = pkt['path']

        except BlockingIOError:
            pass

        # ===== SIMPAN GLOBAL PATH PENUH (SEKALI) =====
        if self.global_path and not self.path_saved:
            for p in self.global_path:
                self.path_writer.writerow([
                    p['idx'],
                    p['x'],
                    p['y']
                ])
            self.path_saved = True
            self.get_logger().info(
                f'Global path initial SAVED ({len(self.global_path)} points)'
            )

        # ===== SIMPAN GOAL & WAYPOINT =====
        if self.goal_pose:
            self.goal_writer.writerow([
                self.goal_pose['x'],
                self.goal_pose['y'],
                self.goal_pose['th']
            ])

        if self.waypoint:
            self.wp_writer.writerow([
                self.waypoint['x'],
                self.waypoint['y'],
                self.waypoint['th']
            ])

        # ===== SINKRON FORMASI =====
        if not (self.leader_pose and self.follower_pose):
            return

        self.sync_writer.writerow([
            self.follower_pose['t'],

            self.leader_pose['x'],
            self.leader_pose['y'],
            self.leader_pose['th'],

            self.follower_pose['x'],
            self.follower_pose['y'],
            self.follower_pose['th'],

            self.leader_cmd['vx'] if self.leader_cmd else 0.0,
            self.leader_cmd['vy'] if self.leader_cmd else 0.0,
            self.leader_cmd['w']  if self.leader_cmd else 0.0,

            self.follower_cmd['vx'] if self.follower_cmd else 0.0,
            self.follower_cmd['vy'] if self.follower_cmd else 0.0,
            self.follower_cmd['w']  if self.follower_cmd else 0.0
        ])

    # =========================
    # CLEAN SHUTDOWN
    # =========================
    def destroy_node(self):
        self.csv_sync.close()
        self.csv_path.close()
        self.csv_goal.close()
        self.csv_wp.close()
        super().destroy_node()


# =========================
# MAIN
# =========================
def main():
    rclpy.init()
    node = Robot2UDPSenderLogger()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
