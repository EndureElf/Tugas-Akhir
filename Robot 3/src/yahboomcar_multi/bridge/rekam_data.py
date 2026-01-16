#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

from geometry_msgs.msg import PoseWithCovarianceStamped, PoseStamped, Twist
from nav_msgs.msg import Path
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy

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
class Robot1Logger(Node):

    def __init__(self):
        super().__init__('robot1_logger')

        # =========================
        # LOG DIRECTORY
        # =========================
        self.log_dir = os.path.expanduser('~/ros2_ws/log_data')
        os.makedirs(self.log_dir, exist_ok=True)

        # =========================
        # CSV FILES
        # =========================
        # Trajectory
        self.csv_traj = open(
            os.path.join(self.log_dir, 'robot1_trajectory.csv'),
            'w', newline=''
        )
        self.traj_writer = csv.writer(self.csv_traj)
        self.traj_writer.writerow(['t', 'x', 'y', 'th'])

        # cmd_vel
        self.csv_cmd = open(
            os.path.join(self.log_dir, 'robot1_cmd_vel.csv'),
            'w', newline=''
        )
        self.cmd_writer = csv.writer(self.csv_cmd)
        self.cmd_writer.writerow(['t', 'vx', 'vy', 'w'])

        # Global Path
        self.csv_path = open(
            os.path.join(self.log_dir, 'robot1_global_path_initial.csv'),
            'w', newline=''
        )
        self.path_writer = csv.writer(self.csv_path)
        self.path_writer.writerow(['idx', 'x', 'y'])

        # Goal Pose
        self.csv_goal = open(
            os.path.join(self.log_dir, 'robot1_goal_pose.csv'),
            'w', newline=''
        )
        self.goal_writer = csv.writer(self.csv_goal)
        self.goal_writer.writerow(['x', 'y', 'th'])

        # Waypoint
        self.csv_wp = open(
            os.path.join(self.log_dir, 'robot1_waypoint.csv'),
            'w', newline=''
        )
        self.wp_writer = csv.writer(self.csv_wp)
        self.wp_writer.writerow(['x', 'y', 'th'])

        self.path_saved = False

        # =========================
        # QoS
        # =========================
        amcl_qos = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL
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
            self.cmd_vel_cb,
            10
        )

        self.create_subscription(
            Path,
            '/robot1/plan',
            self.global_path_cb,
            10
        )

        self.create_subscription(
            PoseStamped,
            '/robot1/goal_pose',
            self.goal_cb,
            10
        )

        # Optional waypoint
        self.create_subscription(
            PoseStamped,
            '/robot3/goal_pose',
            self.waypoint_cb,
            10
        )

        self.get_logger().info('Robot1 Logger READY')

    # =========================
    # CALLBACKS
    # =========================
    def amcl_cb(self, msg):
        p = msg.pose.pose
        t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9

        self.traj_writer.writerow([
            t,
            p.position.x,
            p.position.y,
            quat_to_yaw(p.orientation)
        ])

    def cmd_vel_cb(self, msg):
        t = self.get_clock().now().nanoseconds * 1e-9
        self.cmd_writer.writerow([
            t,
            msg.linear.x,
            msg.linear.y,
            msg.angular.z
        ])

    def global_path_cb(self, msg):
        if self.path_saved:
            return

        for i, pose in enumerate(msg.poses):
            p = pose.pose.position
            self.path_writer.writerow([i, p.x, p.y])

        self.path_saved = True
        self.get_logger().info(
            f'Global path SAVED ({len(msg.poses)} points)'
        )

    def goal_cb(self, msg):
        p = msg.pose
        self.goal_writer.writerow([
            p.position.x,
            p.position.y,
            quat_to_yaw(p.orientation)
        ])

    def waypoint_cb(self, msg):
        p = msg.pose
        self.wp_writer.writerow([
            p.position.x,
            p.position.y,
            quat_to_yaw(p.orientation)
        ])

    # =========================
    # CLEAN SHUTDOWN
    # =========================
    def destroy_node(self):
        self.csv_traj.close()
        self.csv_cmd.close()
        self.csv_path.close()
        self.csv_goal.close()
        self.csv_wp.close()
        super().destroy_node()


# =========================
# MAIN
# =========================
def main():
    rclpy.init()
    node = Robot1Logger()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
