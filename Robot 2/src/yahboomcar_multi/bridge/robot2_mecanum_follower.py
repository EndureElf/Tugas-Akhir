import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from nav_msgs.msg import Odometry
import socket
import json
import math
import threading


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


def normalize_angle(angle):
    """Normalize angle to [-pi, pi]"""
    while angle > math.pi:
        angle -= 2.0 * math.pi
    while angle < -math.pi:
        angle += 2.0 * math.pi
    return angle


class Robot2Follower(Node):
    def __init__(self):
        super().__init__('robot2_nav2_follower')

        # ===============================
        # OFFSET FORMASI (UBAH DI SINI)
        # ===============================
        # KOORDINAT BODY-FRAME LEADER:
        #     +X = DEPAN leader
        #     +Y = KIRI leader
        #     -X = BELAKANG leader  
        #     -Y = KANAN leader
        #
        # Contoh formasi:
        #   Belakang Kiri:  offset_x=-0.5, offset_y=+0.5
        #   Belakang Kanan: offset_x=-0.5, offset_y=-0.5
        #   Depan Kiri:     offset_x=+0.5, offset_y=+0.5
        #   Samping Kiri:   offset_x=0.0,  offset_y=+0.5
        
        self.offset_x = -0.5   # BELAKANG (negatif)
        self.offset_y =  0.5   # KIRI (positif)
        
        # ⚠️ JIKA POSISI TERBALIK, COBA GANTI TANDA:
        # self.offset_x = 0.5   # Ganti tanda
        # self.offset_y = -0.5  # Ganti tanda

        # ===============================
        # PARAMETER TUNING
        # ===============================
        self.min_dist_to_leader = 0.2      # Jarak minimum safety ke leader (m)
        self.max_dist_to_target = 0.2      # Tolerance arrival ke target (m)
        self.pos_update_threshold = 0.08   # Update goal jika berubah > 8cm
        self.yaw_update_threshold = math.radians(8)  # Update goal jika orientasi berubah > 8 deg
        self.goal_update_rate = 0.25       # Minimum waktu antar update goal (s)

        self.leader_goal = None
        self.leader_pose = None
        self.follower_pose = None

        self.last_goal_sent = None   # (x, y, yaw, timestamp)
        self.goal_sent_count = 0

        # Debug counters
        self.udp_count = 0
        self.amcl_count = 0
        self.odom_count = 0
        self.udp_last_received = 0.0

        # ===============================
        # SUBSCRIBE TO ROBOT2 POSE
        # ===============================
        self.get_logger().info("🔍 Creating subscriptions...")
        
        # QoS profile untuk AMCL
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
        self.get_logger().info("   ✓ Subscribed to /robot2/amcl_pose")
        
        # Fallback ke odom_raw
        self.odom_sub = self.create_subscription(
            Odometry,
            '/robot2/odom_raw',
            self.odom_cb,
            10
        )
        self.get_logger().info("   ✓ Subscribed to /robot2/odom_raw (fallback)")

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

        # Processing timer
        self.timer = self.create_timer(0.2, self.process)

        # Debug timer
        self.debug_timer = self.create_timer(3.0, self.print_debug_status)

        self.get_logger().info("🤖 Robot2 Nav2 Follower STARTED (FIXED)")
        self.get_logger().info(f"   Formation: {self.offset_x:.2f}m back, {self.offset_y:.2f}m left")

    def amcl_cb(self, msg):
        """Callback untuk AMCL pose"""
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
            self.get_logger().error(f"Error in amcl_cb: {e}")

    def odom_cb(self, msg):
        """Callback untuk Odometry (fallback)"""
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
            self.odom_count += 1
            
            if self.odom_count == 1:
                self.get_logger().warn("⚠️  Using ODOM fallback")
                
        except Exception as e:
            self.get_logger().error(f"Error in odom_cb: {e}")

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
                    self.get_logger().info(f"✅ UDP connected from {addr}")
                    
            except socket.timeout:
                continue
            except Exception as e:
                self.get_logger().error(f"UDP error: {e}")

    def print_debug_status(self):
        """Print debug information"""
        status_parts = []
        status_parts.append(f"UDP:{self.udp_count}")
        status_parts.append(f"AMCL:{self.amcl_count}")
        status_parts.append(f"Goals:{self.goal_sent_count}")
        
        if self.follower_pose:
            fx = self.follower_pose['x']
            fy = self.follower_pose['y']
            fyaw = math.degrees(self.follower_pose['yaw'])
            status_parts.append(f"Fol:({fx:.1f},{fy:.1f},{fyaw:.0f}°)")
        
        if self.leader_pose:
            lx = self.leader_pose['x']
            ly = self.leader_pose['y']
            lyaw = math.degrees(self.leader_pose['yaw'])
            status_parts.append(f"Lead:({lx:.1f},{ly:.1f},{lyaw:.0f}°)")
        
        self.get_logger().info(" | ".join(status_parts))

    def compute_formation_target(self):
        """
        Hitung target posisi formasi
        Returns: (x_target, y_target, yaw_target) atau None jika tidak valid
        """
        # ⚠️ PERBAIKAN CRITICAL: Gunakan POSE leader (bukan GOAL) sebagai base!
        # leader_goal = tujuan akhir (bisa jauh)
        # leader_pose = posisi sekarang (real-time)
        x_base = self.leader_pose["x"]  # GANTI dari leader_goal ke leader_pose
        y_base = self.leader_pose["y"]  # GANTI dari leader_goal ke leader_pose
        yaw_base = self.leader_pose["yaw"]
        
        # Transform offset dari body-frame ke world-frame
        cos_yaw = math.cos(yaw_base)
        sin_yaw = math.sin(yaw_base)
        
        # FORMULA TRANSFORMASI:
        x_target = x_base + (self.offset_x * cos_yaw - self.offset_y * sin_yaw)
        y_target = y_base + (self.offset_x * sin_yaw + self.offset_y * cos_yaw)
        
        # DEBUG: Log transformasi (setiap 20 goal)
        if self.goal_sent_count % 20 == 0 or self.goal_sent_count < 5:
            self.get_logger().info(
                f"🔍 FORMATION: "
                f"LeaderNOW@({x_base:.2f},{y_base:.2f},{math.degrees(yaw_base):.0f}°) + "
                f"Offset({self.offset_x},{self.offset_y}) = "
                f"Target@({x_target:.2f},{y_target:.2f})"
            )
        
        # Orientasi target = ikuti leader
        yaw_target = self.leader_pose["yaw"]
        
        return (x_target, y_target, yaw_target)

    def should_update_goal(self, x_target, y_target, yaw_target, now):
        """
        Cek apakah perlu update goal
        Returns: (should_update, reason)
        """
        # Selalu kirim goal pertama
        if self.last_goal_sent is None:
            return (True, "first_goal")
        
        last_x, last_y, last_yaw, last_time = self.last_goal_sent
        
        # Rate limiting
        if now - last_time < self.goal_update_rate:
            return (False, "rate_limit")
        
        # Cek perubahan posisi
        dx = x_target - last_x
        dy = y_target - last_y
        pos_change = math.hypot(dx, dy)
        
        if pos_change > self.pos_update_threshold:
            return (True, f"pos_change:{pos_change:.2f}m")
        
        # Cek perubahan orientasi
        yaw_change = normalize_angle(yaw_target - last_yaw)
        
        if abs(yaw_change) > self.yaw_update_threshold:
            return (True, f"yaw_change:{math.degrees(yaw_change):.1f}°")
        
        return (False, "no_significant_change")

    def process(self):
        """Main processing loop"""
        # Check data availability
        if None in (self.leader_goal, self.leader_pose, self.follower_pose):
            return

        # Check UDP connection
        now = self.get_clock().now().nanoseconds * 1e-9
        if now - self.udp_last_received > 2.0:
            return

        # ===============================
        # COMPUTE FORMATION TARGET
        # ===============================
        x_target, y_target, yaw_target = self.compute_formation_target()

        # ===============================
        # COMPUTE DISTANCES
        # ===============================
        # Jarak follower ke leader
        dx_leader = self.follower_pose["x"] - self.leader_pose["x"]
        dy_leader = self.follower_pose["y"] - self.leader_pose["y"]
        dist_to_leader = math.hypot(dx_leader, dy_leader)
        
        # Jarak follower ke target
        dx_target = x_target - self.follower_pose["x"]
        dy_target = y_target - self.follower_pose["y"]
        dist_to_target = math.hypot(dx_target, dy_target)

        # ===============================
        # SAFETY CHECKS
        # ===============================
        # Jangan kirim goal jika terlalu dekat dengan leader DAN sudah di posisi
        if dist_to_leader < self.min_dist_to_leader and dist_to_target < self.max_dist_to_target:
            return

        # ===============================
        # DECIDE IF SHOULD UPDATE GOAL
        # ===============================
        should_update, reason = self.should_update_goal(x_target, y_target, yaw_target, now)
        
        if not should_update:
            return

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
        self.last_goal_sent = (x_target, y_target, yaw_target, now)
        self.goal_sent_count += 1

        # Logging
        self.get_logger().info(
            f"🎯 Goal#{self.goal_sent_count} → ({x_target:.2f},{y_target:.2f}) "
            f"yaw={math.degrees(yaw_target):.0f}° | "
            f"dLeader={dist_to_leader:.2f}m dTarget={dist_to_target:.2f}m | "
            f"reason={reason}"
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