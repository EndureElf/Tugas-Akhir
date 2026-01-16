import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from nav_msgs.msg import Odometry
import socket
import json
import math
import threading
import time


def quat_to_yaw(q):
    """Convert quaternion to yaw angle"""
    return math.atan2(
        2.0 * (q.w * q.z + q.x * q.y),
        1.0 - 2.0 * (q.y*q.y + q.z*q.z)
    )


def yaw_to_quat(yaw):
    """Convert yaw angle to quaternion"""
    return (
        0.0,
        0.0,
        math.sin(yaw * 0.5),
        math.cos(yaw * 0.5)
    )


class Robot2Follower(Node):
    def __init__(self):
        super().__init__('robot2_nav2_follower')

        # ===============================
        # JADWAL FORMASI - EDIT DI SINI!
        # ===============================
        # Format: [waktu_mulai, nama_formasi, offset_x, offset_y]
        self.formation_schedule = [
            [0,   "follow_center",    -0.5,  0.0],   # 0-6 detik: belakang tengah
            [7,   "follow_left",      -0.5,  0.3],   # 7-13 detik: belakang kiri
            [14,  "follow_right",     -0.5, -0.3],   # 14-20 detik: belakang kanan
            [21,  "diagonal_left",    -0.6,  0.4],   # 21-27 detik: diagonal kiri
            [28,  "diagonal_right",   -0.6, -0.4],   # 28-34 detik: diagonal kanan
        ]
        
        self.loop_duration = 35.0
        self.start_time = time.time()
        
        # Current formation
        self.current_formation_name = self.formation_schedule[0][1]
        self.offset_x = self.formation_schedule[0][2]
        self.offset_y = self.formation_schedule[0][3]

        # ===============================
        # PARAMETERS
        # ===============================
        self.min_safe_distance = 0.4
        self.emergency_stop_distance = 0.25
        
        # Goal update policy
        self.goal_change_threshold = 0.2      # Update jika goal leader berubah >0.2m
        self.goal_update_min_interval = 0.5   # Minimal 0.5 detik antar update

        self.leader_goal = None
        self.leader_pose = None
        self.follower_pose = None

        self.last_published_goal = None
        self.last_leader_goal = None
        self.last_goal_time = 0.0
        self.udp_last_received = 0.0

        # Tracking
        self.udp_count = 0
        self.amcl_count = 0
        self.goal_sent_count = 0
        self.goal_skipped_count = 0
        self.emergency_stops = 0

        # ===============================
        # SUBSCRIBE TO ROBOT2 POSE
        # ===============================
        amcl_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )
        
        self.amcl_sub = self.create_subscription(
            PoseWithCovarianceStamped,
            '/robot2/amcl_pose',
            self.amcl_cb,
            amcl_qos
        )
        
        self.odom_sub = self.create_subscription(
            Odometry,
            '/robot2/odom_raw',
            self.odom_cb,
            10
        )

        # ===============================
        # NAV2 GOAL PUBLISHER
        # ===============================
        self.goal_pub = self.create_publisher(
            PoseStamped,
            '/robot2/goal_pose',
            10
        )

        # ===============================
        # UDP RECEIVE
        # ===============================
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.bind(('0.0.0.0', 8891))
        self.sock.settimeout(1.0)
        threading.Thread(target=self.udp_listener, daemon=True).start()

        # Timers
        self.timer = self.create_timer(0.2, self.process)
        self.formation_timer = self.create_timer(1.0, self.update_formation_by_time)
        self.debug_timer = self.create_timer(3.0, self.print_debug_status)

        self.get_logger().info("🤖 Robot2 GOAL-BASED Dynamic Formation STARTED")
        self.get_logger().info("📅 Formation Schedule:")
        for schedule in self.formation_schedule:
            t, name, x, y = schedule
            self.get_logger().info(f"   {t:3d}s → {name:20s} (x={x:+.1f}, y={y:+.1f})")
        self.get_logger().info(f"   🔁 Loop every {self.loop_duration}s")
        self.get_logger().info(f"   🎯 Following leader's GOAL (not real-time pose)")

    def amcl_cb(self, msg):
        try:
            p = msg.pose.pose.position
            q = msg.pose.pose.orientation
            self.follower_pose = {
                "x": p.x,
                "y": p.y,
                "yaw": quat_to_yaw(q)
            }
            self.amcl_count += 1
        except Exception as e:
            self.get_logger().error(f"❌ Error in amcl_cb: {e}")

    def odom_cb(self, msg):
        try:
            if self.amcl_count > 0:
                return
            p = msg.pose.pose.position
            q = msg.pose.pose.orientation
            self.follower_pose = {
                "x": p.x,
                "y": p.y,
                "yaw": quat_to_yaw(q)
            }
        except Exception as e:
            self.get_logger().error(f"❌ Error in odom_cb: {e}")

    def update_formation_by_time(self):
        """Update formasi berdasarkan waktu yang sudah berjalan"""
        elapsed_time = time.time() - self.start_time
        current_time = elapsed_time % self.loop_duration
        
        selected_formation = self.formation_schedule[0]
        
        for schedule in self.formation_schedule:
            if current_time >= schedule[0]:
                selected_formation = schedule
            else:
                break
        
        new_name = selected_formation[1]
        if new_name != self.current_formation_name:
            self.current_formation_name = new_name
            self.offset_x = selected_formation[2]
            self.offset_y = selected_formation[3]
            
            self.get_logger().info(
                f"🔄 [{current_time:.0f}s] Formation: {self.current_formation_name} "
                f"(x={self.offset_x:+.1f}, y={self.offset_y:+.1f})"
            )
            
            # Force update goal saat formasi berubah
            self.last_leader_goal = None

    def udp_listener(self):
        """Thread untuk menerima data UDP dari leader"""
        self.get_logger().info("📡 UDP Listener started on port 8891")
        
        while rclpy.ok():
            try:
                data, addr = self.sock.recvfrom(2048)
                msg = json.loads(data.decode())
                
                self.leader_goal = msg["leader_goal"]
                self.leader_pose = msg["leader_pose"]
                
                self.udp_last_received = self.get_clock().now().nanoseconds * 1e-9
                self.udp_count += 1
                
                if self.udp_count == 1:
                    self.get_logger().info(f"✅ First UDP packet received from {addr}")
                    
            except socket.timeout:
                continue
            except Exception as e:
                self.get_logger().error(f"UDP receive error: {e}")

    def print_debug_status(self):
        """Print debug information"""
        elapsed = time.time() - self.start_time
        current_time = elapsed % self.loop_duration
        
        status = []
        status.append(f"T:{current_time:.0f}s")
        status.append(f"Form:{self.current_formation_name}")
        status.append(f"Sent:{self.goal_sent_count}")
        status.append(f"Skip:{self.goal_skipped_count}")
        
        if self.leader_pose and self.follower_pose:
            dx = self.follower_pose["x"] - self.leader_pose["x"]
            dy = self.follower_pose["y"] - self.leader_pose["y"]
            dist = math.hypot(dx, dy)
            
            if dist < self.emergency_stop_distance:
                status.append(f"🔴Dist:{dist:.2f}m")
            elif dist < self.min_safe_distance:
                status.append(f"🟡Dist:{dist:.2f}m")
            else:
                status.append(f"🟢Dist:{dist:.2f}m")
        
        if self.emergency_stops > 0:
            status.append(f"🚨Stops:{self.emergency_stops}")
        
        self.get_logger().info(" | ".join(status))

    def process(self):
        """Main processing loop - mengikuti GOAL leader"""
        if self.leader_goal is None or self.leader_pose is None or self.follower_pose is None:
            return

        now = self.get_clock().now().nanoseconds * 1e-9
        if now - self.udp_last_received > 3.0:
            return

        # ===============================
        # SAFETY CHECK: JARAK KE LEADER
        # ===============================
        dx_leader = self.follower_pose["x"] - self.leader_pose["x"]
        dy_leader = self.follower_pose["y"] - self.leader_pose["y"]
        dist_to_leader = math.hypot(dx_leader, dy_leader)

        # EMERGENCY STOP
        if dist_to_leader < self.emergency_stop_distance:
            self.emergency_stops += 1
            self.get_logger().error(
                f"🚨 EMERGENCY STOP! Distance: {dist_to_leader:.2f}m",
                throttle_duration_sec=1.0
            )
            return

        # ===============================
        # CEK APAKAH LEADER GOAL BERUBAH
        # ===============================
        leader_goal_changed = False
        
        if self.last_leader_goal is None:
            # Pertama kali
            leader_goal_changed = True
        else:
            # Cek perubahan goal leader
            dx = self.leader_goal["x"] - self.last_leader_goal["x"]
            dy = self.leader_goal["y"] - self.last_leader_goal["y"]
            goal_change = math.hypot(dx, dy)
            
            dyaw = self.leader_goal["yaw"] - self.last_leader_goal["yaw"]
            dyaw = math.atan2(math.sin(dyaw), math.cos(dyaw))
            
            if goal_change > self.goal_change_threshold or abs(dyaw) > math.radians(20):
                leader_goal_changed = True

        # Rate limiting
        if now - self.last_goal_time < self.goal_update_min_interval:
            leader_goal_changed = False

        if not leader_goal_changed:
            self.goal_skipped_count += 1
            return

        # ===============================
        # HITUNG TARGET FORMASI BERDASARKAN LEADER GOAL
        # ===============================
        xg = self.leader_goal["x"]
        yg = self.leader_goal["y"]
        yaw_goal = self.leader_goal["yaw"]

        # Transform offset ke koordinat global berdasarkan orientasi GOAL leader
        x_target = xg + self.offset_x * math.cos(yaw_goal) \
                      - self.offset_y * math.sin(yaw_goal)
        y_target = yg + self.offset_x * math.sin(yaw_goal) \
                      + self.offset_y * math.cos(yaw_goal)

        yaw_target = yaw_goal

        # ===============================
        # SAFETY: CEK JARAK TARGET KE LEADER POSE (bukan goal)
        # ===============================
        dx_target_leader = x_target - self.leader_pose["x"]
        dy_target_leader = y_target - self.leader_pose["y"]
        target_to_leader = math.hypot(dx_target_leader, dy_target_leader)

        # Jika target terlalu dekat dengan posisi aktual leader, adjust
        if target_to_leader < self.min_safe_distance:
            self.get_logger().warn(
                f"⚠️  Target too close to leader pose: {target_to_leader:.2f}m, adjusting...",
                throttle_duration_sec=2.0
            )
            direction = math.atan2(dy_target_leader, dx_target_leader)
            x_target = self.leader_pose["x"] + self.min_safe_distance * 1.2 * math.cos(direction)
            y_target = self.leader_pose["y"] + self.min_safe_distance * 1.2 * math.sin(direction)

        # ===============================
        # PUBLISH NAV2 GOAL
        # ===============================
        qx, qy, qz, qw = yaw_to_quat(yaw_target)

        goal = PoseStamped()
        goal.header.frame_id = "map"
        goal.header.stamp = self.get_clock().now().to_msg()

        goal.pose.position.x = x_target
        goal.pose.position.y = y_target
        goal.pose.position.z = 0.0
        goal.pose.orientation.x = qx
        goal.pose.orientation.y = qy
        goal.pose.orientation.z = qz
        goal.pose.orientation.w = qw

        self.goal_pub.publish(goal)

        # Update tracking
        self.last_published_goal = (x_target, y_target, yaw_target)
        self.last_leader_goal = self.leader_goal.copy()
        self.last_goal_time = now
        self.goal_sent_count += 1

        self.get_logger().info(
            f"🎯 Goal #{self.goal_sent_count} [{self.current_formation_name}] → "
            f"({x_target:.2f}, {y_target:.2f}) yaw={math.degrees(yaw_target):.0f}° | "
            f"LeaderGoal:({xg:.2f},{yg:.2f}) | d_leader={dist_to_leader:.2f}m"
        )


def main():
    rclpy.init()
    node = Robot2Follower()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()