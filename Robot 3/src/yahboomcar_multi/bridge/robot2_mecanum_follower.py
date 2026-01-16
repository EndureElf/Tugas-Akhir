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


class Robot2Follower(Node):
    def __init__(self):
        super().__init__('robot2_nav2_follower')

        # ===============================
        # OFFSET FORMASI (UBAH DI SINI)
        # ===============================
        self.offset_x = -0.5   # belakang leader (negatif = belakang)
        self.offset_y =  0.5   # kiri leader (positif = kiri)

        # ===============================
        # NAV2 SAFETY PARAM
        # ===============================
        self.min_dist = 0.2          # jarak minimum ke leader
        self.pos_threshold = 0.2     # threshold posisi untuk update goal
        self.update_period = 0.2     # minimum waktu antar update goal
        self.yaw_threshold = math.radians(15)  # threshold orientasi

        self.leader_goal = None
        self.leader_pose = None
        self.follower_pose = None

        self.last_goal = None   # (x, y, yaw)
        self.last_time = 0.0
        self.udp_last_received = 0.0

        # Debug counters
        self.udp_count = 0
        self.amcl_count = 0
        self.odom_count = 0
        self.goal_sent_count = 0

        # ===============================
        # SUBSCRIBE TO ROBOT2 POSE
        # ===============================
        self.get_logger().info("🔍 Creating subscriptions...")
        
        # QoS profile untuk AMCL (biasanya menggunakan transient_local)
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
            amcl_qos  # Menggunakan QoS yang sesuai
        )
        self.get_logger().info("   ✓ Subscribed to /robot2/amcl_pose (with TRANSIENT_LOCAL QoS)")
        
        # Fallback ke odom_raw (bukan odom biasa)
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

        # Timer untuk processing
        self.timer = self.create_timer(0.2, self.process)

        # Timer untuk debug status
        self.debug_timer = self.create_timer(2.0, self.print_debug_status)

        self.get_logger().info("🤖 Robot2 Nav2 Follower STARTED")
        self.get_logger().info(f"   Formation offset: x={self.offset_x}, y={self.offset_y}")
        self.get_logger().info("   Listening on UDP port 8891...")

    def amcl_cb(self, msg):
        """Callback untuk AMCL pose"""
        self.get_logger().info("🔔 AMCL callback triggered!")  # DEBUG
        try:
            p = msg.pose.pose.position
            q = msg.pose.pose.orientation
            
            self.get_logger().info(f"   Parsing: x={p.x:.2f}, y={p.y:.2f}")  # DEBUG
            
            self.follower_pose = {
                "x": p.x,
                "y": p.y,
                "yaw": quat_to_yaw(q)
            }
            self.amcl_count += 1
            
            self.get_logger().info(f"✅ AMCL pose set! Count: {self.amcl_count}")
                
        except Exception as e:
            self.get_logger().error(f"❌ Error in amcl_cb: {e}")
            import traceback
            self.get_logger().error(traceback.format_exc())

    def odom_cb(self, msg):
        """Callback untuk Odometry (fallback)"""
        self.get_logger().info("🔔 ODOM callback triggered!")  # DEBUG
        try:
            # Hanya gunakan odom jika AMCL tidak aktif
            if self.amcl_count > 0:
                self.get_logger().info("   Skipping (AMCL active)")
                return
                
            p = msg.pose.pose.position
            q = msg.pose.pose.orientation
            
            self.get_logger().info(f"   Parsing: x={p.x:.2f}, y={p.y:.2f}")  # DEBUG
            
            self.follower_pose = {
                "x": p.x,
                "y": p.y,
                "yaw": quat_to_yaw(q)
            }
            self.odom_count += 1
            
            self.get_logger().info(f"✅ ODOM pose set! Count: {self.odom_count}")
                
        except Exception as e:
            self.get_logger().error(f"❌ Error in odom_cb: {e}")
            import traceback
            self.get_logger().error(traceback.format_exc())

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
                    self.get_logger().info(f"   Leader goal: x={self.leader_goal['x']:.2f}, y={self.leader_goal['y']:.2f}")
                    
            except socket.timeout:
                continue
            except Exception as e:
                self.get_logger().error(f"UDP receive error: {e}")

    def print_debug_status(self):
        """Print debug information"""
        status = []
        status.append(f"UDP: {self.udp_count}")
        status.append(f"AMCL: {self.amcl_count}")
        status.append(f"ODOM: {self.odom_count}")
        status.append(f"Goals: {self.goal_sent_count}")
        
        if self.leader_goal is not None:
            status.append(f"L_goal✅")
        else:
            status.append(f"L_goal❌")
            
        if self.leader_pose is not None:
            status.append(f"L_pose✅")
        else:
            status.append(f"L_pose❌")
            
        if self.follower_pose is not None:
            fx = self.follower_pose['x']
            fy = self.follower_pose['y']
            status.append(f"F_pose✅({fx:.1f},{fy:.1f})")
        else:
            status.append(f"F_pose❌")
        
        self.get_logger().info(" | ".join(status))

    def process(self):
        """Main processing loop untuk menghitung dan publish goal"""
        # Check if all data available
        if self.leader_goal is None:
            return
            
        if self.leader_pose is None:
            return
            
        if self.follower_pose is None:
            return

        # Check UDP connection
        now = self.get_clock().now().nanoseconds * 1e-9
        if now - self.udp_last_received > 3.0:
            self.get_logger().warn("⚠️  No UDP data from leader!", throttle_duration_sec=2.0)
            return

        # ===============================
        # HITUNG TARGET FORMASI
        # ===============================
        xg = self.leader_goal["x"]
        yg = self.leader_goal["y"]
        yaw_leader = self.leader_pose["yaw"]

        # Transform offset ke koordinat global
        x_target = xg + self.offset_x * math.cos(yaw_leader) \
                      - self.offset_y * math.sin(yaw_leader)
        y_target = yg + self.offset_x * math.sin(yaw_leader) \
                      + self.offset_y * math.cos(yaw_leader)

        # Orientasi target = orientasi goal leader
        yaw_target = self.leader_goal["yaw"]

        # ===============================
        # SAFETY: JARAK KE LEADER
        # ===============================
        dx = self.follower_pose["x"] - self.leader_pose["x"]
        dy = self.follower_pose["y"] - self.leader_pose["y"]
        dist_to_leader = math.hypot(dx, dy)

        # Jarak ke target
        dx_target = x_target - self.follower_pose["x"]
        dy_target = y_target - self.follower_pose["y"]
        dist_to_target = math.hypot(dx_target, dy_target)

        # Jangan block jika sudah di posisi
        if dist_to_leader < self.min_dist and dist_to_target < 0.1:
            return

        # ===============================
        # FILTER UPDATE GOAL
        # ===============================
        should_send = True
        
        if self.last_goal is not None:
            dx_goal = x_target - self.last_goal[0]
            dy_goal = y_target - self.last_goal[1]
            pos_change = math.hypot(dx_goal, dy_goal)
            
            # Cek perubahan orientasi
            yaw_err = yaw_target - self.last_goal[2]
            yaw_err = math.atan2(math.sin(yaw_err), math.cos(yaw_err))
            
            # Skip jika tidak ada perubahan signifikan
            if pos_change < self.pos_threshold and abs(yaw_err) < self.yaw_threshold:
                should_send = False

        # Rate limiting
        if now - self.last_time < self.update_period:
            should_send = False

        if not should_send:
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
        self.last_goal = (x_target, y_target, yaw_target)
        self.last_time = now
        self.goal_sent_count += 1

        # Logging
        self.get_logger().info(
            f"🎯 Goal #{self.goal_sent_count} → ({x_target:.2f}, {y_target:.2f}) "
            f"yaw={math.degrees(yaw_target):.1f}° | "
            f"dist_leader={dist_to_leader:.2f}m | "
            f"dist_target={dist_to_target:.2f}m"
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