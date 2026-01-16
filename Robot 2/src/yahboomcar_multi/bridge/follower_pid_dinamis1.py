#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, PoseWithCovarianceStamped
import numpy as np
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
import math
import socket
import json
import threading


class Robot2FollowerPID(Node):

    def __init__(self):
        super().__init__('robot2_follower_pid_mecanum')

        # Mode operasi - AWALNYA KITA ANGGAP LEADER DIAM
        self.leader_moving = False  # Awalnya false, anggap leader diam
        self.movement_threshold = 0.05  # Threshold kecepatan untuk menentukan leader bergerak (m/s)
        self.min_stopped_time = 0.5  # Minimal waktu diam untuk ganti formasi waktu
        self.min_moving_time = 0.5  # Minimal waktu bergerak untuk ganti formasi jarak
        
        # Formasi berbasis jarak (saat leader bergerak)
        self.formation_states_distance = [
            {"distance": 0.0, "offset_x": -0.5, "offset_y": -0.5, "name": "belakang-kanan"},
            {"distance": 2.0, "offset_x": -0.5, "offset_y": 0.0, "name": "belakang"},
            {"distance": 4.0, "offset_x": -0.5, "offset_y": 0.5, "name": "belakang-kiri"},
            {"distance": 7.0, "offset_x": 0.0, "offset_y": 0.5, "name": "kiri-pas"},
        ]
        
        # Formasi berbasis waktu (saat leader diam)
        self.formation_states_time = [
            {"time": 0.0, "offset_x": 0.0, "offset_y": 0.5, "name": "kiri-pas"},
            {"time": 4.0, "offset_x": -0.5, "offset_y": 0.5, "name": "belakang-kiri"},
            {"time": 7.0, "offset_x": -0.5, "offset_y": 0.0, "name": "belakang"},
            {"time": 10.0, "offset_x": -0.5, "offset_y": -0.5, "name": "belakang-kanan"},
        ]
        
        # Formasi offset saat ini - AWALNYA KIRI-PAS (sesuai formasi waktu pertama)
        self.offset_x = 0.0   # kiri-pas (default awal)
        self.offset_y = 0.5   # kiri-pas (default awal)
        
        # Tracking untuk formasi berbasis jarak
        self.current_formation_idx_distance = 0
        self.total_distance = 0.0
        self.last_leader_position = None
        self.distance_traveled_since_last_formation = 0.0
        
        # Tracking untuk formasi berbasis waktu
        self.current_formation_idx_time = 0
        self.stopped_start_time = self.get_clock().now()  # Mulai timer waktu dari sekarang
        self.formation_change_time = None
        
        # Untuk deteksi pergerakan yang lebih stabil
        self.moving_start_time = None
        self.stopped_start_time_detection = None
        self.last_leader_speed = 0.0
        self.last_moving_state = False
        
        # Kecepatan maksimum
        self.vx_max = 0.35
        self.vy_max = 0.35
        self.w_max = 1.0

        # PID gains
        self.Kp = np.diag([1.9, 1.9, 1.0])
        self.Ki = np.diag([0.1, 0.1, 0.1])
        self.Kd = np.diag([0.001, 0.001, 0.001])
        self.alpha_ff = np.diag([0.5, 0.5, 0.3])

        # State variables
        self.leader_pose = None
        self.leader_cmd = None
        self.follower_pose = None
        self.last_leader_pose = None
        self.global_leader_pose = None
        
        self.last_leader_update = self.get_clock().now()
        
        # PID control variables
        self.errSum = np.zeros((3, 1))
        self.lastErr = np.zeros((3, 1))
        self.prev_time = self.get_clock().now()

        # Counter untuk mengurangi frekuensi logging
        self.iteration_count = 0
        self.last_log_time = self.get_clock().now()
        
        # Deadbands
        self.vel_deadband = 0.02
        self.pos_deadband = 0.05
        self.yaw_deadband = 0.08
        
        # Timeouts
        self.amcl_timeout = 1.0
        
        # QoS untuk AMCL
        amcl_qos = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL
        )

        # Subscriber untuk pose follower sendiri
        self.create_subscription(
            PoseWithCovarianceStamped,
            '/robot2/amcl_pose',
            self.follower_cb,
            amcl_qos,
        )

        # Publisher untuk cmd_vel
        self.cmd_pub = self.create_publisher(
            Twist,
            '/robot2/cmd_vel',
            10
        )

        # Setup UDP socket
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.bind(('0.0.0.0', 8891))
        self.sock.settimeout(0.1)
        threading.Thread(target=self.udp_listener, daemon=True).start()

        # Timer untuk control loop (20 Hz)
        self.timer = self.create_timer(0.05, self.control_loop)

        self.get_logger().info("Robot2 Follower PID Start - Formasi Dinamis Dua Mode")
        self.get_logger().info("Mode: Berbasis Waktu (leader diam) / Berbasis Jarak (leader bergerak)")
        self.get_logger().info(f"Offset awal: kiri-pas ({self.offset_x}, {self.offset_y})")

    def follower_cb(self, msg):
        """Callback untuk pose follower sendiri"""
        self.follower_pose = msg.pose.pose

    def udp_listener(self):
        """Thread untuk menerima data UDP dari leader"""
        while rclpy.ok():
            try:
                data, addr = self.sock.recvfrom(2048)
                msg = json.loads(data.decode())
                
                # Simpan data dari leader
                new_pose = msg["leader_pose"]
                self.leader_cmd = msg.get("leader_cmd_vel", None)
                self.last_leader_update = self.get_clock().now()
                
                # Hitung jarak yang ditempuh leader
                if self.last_leader_pose is not None and self.leader_moving:
                    dx = new_pose["x"] - self.last_leader_pose["x"]
                    dy = new_pose["y"] - self.last_leader_pose["y"]
                    distance_step = math.sqrt(dx**2 + dy**2)
                    
                    # Update total jarak
                    self.total_distance += distance_step
                    self.distance_traveled_since_last_formation += distance_step
                
                # Update pose leader
                if self.last_leader_pose is None:
                    self.leader_pose = new_pose
                    self.global_leader_pose = new_pose.copy()
                    self.last_leader_pose = new_pose.copy()
                    self.get_logger().info(f"Pose leader pertama: ({new_pose['x']:.2f}, {new_pose['y']:.2f})")
                else:
                    self.leader_pose = new_pose
                    self.last_leader_pose = new_pose.copy()
                
            except socket.timeout:
                continue
            except Exception as e:
                self.get_logger().error(f"UDP error: {e}")

    def check_leader_moving(self):
        """Cek apakah leader sedang bergerak dengan lebih akurat"""
        if self.leader_cmd is None:
            # Jika belum ada data cmd_vel, kita anggap leader diam
            return False
            
        now = self.get_clock().now()
        
        # Hitung kecepatan leader
        vx = self.leader_cmd.get("vx", 0)
        vy = self.leader_cmd.get("vy", 0)
        w = self.leader_cmd.get("w", 0)
        
        linear_speed = math.sqrt(vx**2 + vy**2)
        angular_speed = abs(w)
        total_speed = max(linear_speed, angular_speed)
        
        # Gunakan hysteresis untuk deteksi pergerakan
        is_moving_now = total_speed > self.movement_threshold
        
        # Simpan state sebelumnya
        was_moving = self.leader_moving
        
        if is_moving_now:
            # Leader sedang bergerak
            if not was_moving:
                # Baru mulai bergerak
                if self.moving_start_time is None:
                    self.moving_start_time = now
                else:
                    moving_time = (now - self.moving_start_time).nanoseconds * 1e-9
                    if moving_time > self.min_moving_time:
                        self.leader_moving = True
                        self.stopped_start_time = None
                        self.moving_start_time = None
                        self.stopped_start_time_detection = None
                        self.get_logger().info("🚀 Leader mulai bergerak - Mode: Berbasis Jarak")
                        self.reset_for_moving_mode()
            else:
                # Sudah bergerak, reset timer diam
                self.moving_start_time = None
                self.stopped_start_time_detection = None
        else:
            # Leader diam
            if was_moving:
                # Baru berhenti
                if self.stopped_start_time_detection is None:
                    self.stopped_start_time_detection = now
                else:
                    stopped_time = (now - self.stopped_start_time_detection).nanoseconds * 1e-9
                    if stopped_time > self.min_stopped_time:
                        self.leader_moving = False
                        self.stopped_start_time = now
                        self.stopped_start_time_detection = None
                        self.moving_start_time = None
                        self.get_logger().info("🛑 Leader berhenti - Mode: Berbasis Waktu")
                        self.reset_for_stopped_mode()
            else:
                # Sudah diam, reset timer bergerak
                self.stopped_start_time_detection = None
                self.moving_start_time = None
        
        # Log jika state berubah
        if was_moving != self.leader_moving:
            self.last_moving_state = self.leader_moving
        
        return self.leader_moving

    def reset_for_moving_mode(self):
        """Reset untuk mode bergerak (berbasis jarak)"""
        # Cari formasi jarak yang sesuai dengan jarak tempuh saat ini
        idx = 0
        for i, state in enumerate(self.formation_states_distance):
            if self.total_distance >= state["distance"]:
                idx = i
        
        self.current_formation_idx_distance = idx
        self.apply_formation_distance(idx)
        self.errSum = np.zeros((3, 1))
        self.get_logger().info(f"Reset ke mode bergerak, formasi: {self.formation_states_distance[idx]['name']}")

    def reset_for_stopped_mode(self):
        """Reset untuk mode diam (berbasis waktu)"""
        self.current_formation_idx_time = 0
        self.apply_formation_time(0)
        self.errSum = np.zeros((3, 1))
        self.get_logger().info("Reset ke mode diam, formasi: kiri-pas")

    def apply_formation_time(self, idx):
        """Apply formasi berbasis waktu berdasarkan index"""
        state = self.formation_states_time[idx]
        
        # Update offset
        self.offset_x = state["offset_x"]
        self.offset_y = state["offset_y"]
        
        self.formation_change_time = self.get_clock().now()
        
        # Logging
        if self.stopped_start_time is not None:
            elapsed_time = (self.get_clock().now() - self.stopped_start_time).nanoseconds * 1e-9
            self.get_logger().info(f"⏰ Formasi waktu: {state['name']} (t={elapsed_time:.1f}s)")
            self.get_logger().info(f"   Offset: ({self.offset_x}, {self.offset_y})")

    def apply_formation_distance(self, idx):
        """Apply formasi berbasis jarak berdasarkan index"""
        state = self.formation_states_distance[idx]
        
        # Update offset
        self.offset_x = state["offset_x"]
        self.offset_y = state["offset_y"]
        
        self.get_logger().info(f"📏 Formasi jarak: {state['name']} (jarak: {self.total_distance:.2f}m)")
        self.get_logger().info(f"   Offset: ({self.offset_x}, {self.offset_y})")

    def update_formation_time(self):
        """Update formasi berdasarkan waktu (saat leader diam)"""
        if self.stopped_start_time is None:
            return
            
        # Hitung waktu yang telah berlalu sejak leader diam
        elapsed_time = (self.get_clock().now() - self.stopped_start_time).nanoseconds * 1e-9
        
        # Cari formasi yang sesuai berdasarkan waktu
        new_idx = 0
        for i, state in enumerate(self.formation_states_time):
            if elapsed_time >= state["time"]:
                new_idx = i
        
        # Jika formasi berubah
        if new_idx != self.current_formation_idx_time:
            self.current_formation_idx_time = new_idx
            self.apply_formation_time(new_idx)

    def check_formation_change_distance(self):
        """Cek apakah perlu mengubah formasi berdasarkan jarak (saat leader bergerak)"""
        for i, state in enumerate(self.formation_states_distance):
            if i <= self.current_formation_idx_distance:
                continue
                
            if self.total_distance >= state["distance"]:
                # Ganti formasi
                self.current_formation_idx_distance = i
                self.distance_traveled_since_last_formation = 0.0
                
                # Apply formasi baru
                self.apply_formation_distance(i)
                
                # Reset integral term saat ganti formasi
                self.errSum = np.zeros((3, 1))
                break

    def update_global_leader_pose(self, dt):
        """Update global leader pose dengan dead reckoning"""
        if self.global_leader_pose is None:
            return None
            
        # Jika ada cmd_vel dan global_leader_pose sudah diinisialisasi
        if self.leader_cmd is not None and self.leader_moving:
            # Dead reckoning: global_leader_pose += cmd_vel * dt
            vx = self.leader_cmd["vx"]
            vy = self.leader_cmd["vy"]
            w = self.leader_cmd["w"]
            
            # Transformasi kecepatan ke koordinat global
            theta = self.global_leader_pose["yaw"]
            
            # Integrasi posisi (Euler method)
            self.global_leader_pose["x"] += (vx * math.cos(theta) - vy * math.sin(theta)) * dt
            self.global_leader_pose["y"] += (vx * math.sin(theta) + vy * math.cos(theta)) * dt
            self.global_leader_pose["yaw"] += w * dt
            
            # Normalisasi yaw
            self.global_leader_pose["yaw"] = self.normalize_angle(self.global_leader_pose["yaw"])
            
            # Hitung jarak yang ditempuh dari dead reckoning
            if self.last_leader_position is not None:
                dx = (vx * math.cos(theta) - vy * math.sin(theta)) * dt
                dy = (vx * math.sin(theta) + vy * math.cos(theta)) * dt
                distance_step = math.sqrt(dx**2 + dy**2)
                
                # Update total jarak
                self.total_distance += distance_step
                self.distance_traveled_since_last_formation += distance_step
            
            # Update last position untuk perhitungan jarak berikutnya
            self.last_leader_position = (self.global_leader_pose["x"], self.global_leader_pose["y"])
        
        return self.global_leader_pose

    def control_loop(self):
        """Main control loop"""
        # Validasi data dasar
        if self.global_leader_pose is None or self.follower_pose is None:
            self.get_logger().warn("Waiting for initial data...")
            return
        
        # Cek timeout data leader
        now = self.get_clock().now()
        time_since_update = (now - self.last_leader_update).nanoseconds * 1e-9
        if time_since_update > self.amcl_timeout:
            self.get_logger().warn(f"Leader data stale ({time_since_update:.1f}s)! Stopping...")
            self.publish_zero_vel()
            return
        
        # Cek apakah leader bergerak
        self.check_leader_moving()
        
        # Update formasi berdasarkan mode
        if self.leader_moving:
            # Mode: Berbasis Jarak
            self.check_formation_change_distance()
        else:
            # Mode: Berbasis Waktu
            self.update_formation_time()
        
        # Hitung dt untuk kontrol dan dead reckoning
        dt = (now - self.prev_time).nanoseconds * 1e-9
        self.prev_time = now
        if dt <= 0.0:
            return
        
        # Update global leader pose
        leader_pose = self.update_global_leader_pose(dt)
        
        # Hitung target position dengan offset
        xl = leader_pose["x"]
        yl = leader_pose["y"]
        thetal = leader_pose["yaw"]
        
        # Transformasi offset ke koordinat global
        xt = xl + self.offset_x * math.cos(thetal) - self.offset_y * math.sin(thetal)
        yt = yl + self.offset_x * math.sin(thetal) + self.offset_y * math.cos(thetal)
        thetat = thetal
        
        # Dapatkan pose follower saat ini
        xf = self.follower_pose.position.x
        yf = self.follower_pose.position.y
        thetaf = self.yaw_from_q(self.follower_pose.orientation)
        
        # Hitung error dalam koordinat follower
        e_map = np.array([[xt - xf],
                          [yt - yf]])
        
        R = np.array([
            [math.cos(thetaf), math.sin(thetaf)],
            [-math.sin(thetaf), math.cos(thetaf)]
        ])
        
        e_base = R @ e_map
        ex = e_base[0, 0]
        ey = e_base[1, 0]
        eyaw = self.normalize_angle(thetat - thetaf)
        
        # Jika error kecil, maka robot berhenti
        distance_error = math.sqrt(ex**2 + ey**2)
        
        if distance_error < self.pos_deadband and abs(eyaw) < self.yaw_deadband:
            mode_str = "JARAK" if self.leader_moving else "WAKTU"
            if self.iteration_count % 40 == 0:  # Log hanya setiap 2 detik
                self.get_logger().debug(f"Target tercapai! Mode: {mode_str}, Error: {distance_error:.3f}m")
            self.publish_zero_vel()
            return
        
        # PID Control
        e = np.array([[ex], [ey], [eyaw]])
        
        # Integral term dengan anti-windup
        self.errSum += e * dt
        self.errSum = np.clip(self.errSum, -0.5, 0.5)
        
        # Derivative term
        dErr = (e - self.lastErr) / dt
        self.lastErr = e
        
        # PID output
        u_pid = self.Kp @ e + self.Ki @ self.errSum + self.Kd @ dErr
        
        # Feedforward
        ff_term = np.zeros((3, 1))
        if self.leader_cmd is not None and self.leader_moving:
            vx_leader = self.leader_cmd["vx"]
            vy_leader = self.leader_cmd["vy"]
            w_leader = self.leader_cmd["w"]

            # Transformasi kecepatan leader ke koordinat follower
            vx_offset = vx_leader - w_leader * self.offset_y
            vy_offset = vy_leader + w_leader * self.offset_x

            # Rotasi ke koordinat follower
            theta_rel = thetaf - thetal
            vx_ff = vx_offset * math.cos(theta_rel) + vy_offset * math.sin(theta_rel)
            vy_ff = -vx_offset * math.sin(theta_rel) + vy_offset * math.cos(theta_rel)
            w_ff = w_leader
            
            ff_term = np.array([[vx_ff], [vy_ff], [w_ff]])
        
        # Total output PID + Feedforward
        u = u_pid + self.alpha_ff @ ff_term

        # Clamp velocity dengan batas maksimum
        vx = self.clamp(u[0, 0], self.vx_max)
        vy = self.clamp(u[1, 0], self.vy_max)
        w = self.clamp(u[2, 0], self.w_max)
        
        # Apply velocity deadband
        if abs(vx) < self.vel_deadband:
            vx = 0.0
        if abs(vy) < self.vel_deadband:
            vy = 0.0
        if abs(w) < self.vel_deadband:
            w = 0.0
        
        # Publish command
        cmd = Twist()
        cmd.linear.x = vx
        cmd.linear.y = vy
        cmd.angular.z = w
        self.cmd_pub.publish(cmd)
        
        # Logging periodik (maksimal 1 Hz)
        current_time = self.get_clock().now()
        time_since_last_log = (current_time - self.last_log_time).nanoseconds * 1e-9
        
        if time_since_last_log >= 1.0:
            mode_str = "JARAK" if self.leader_moving else "WAKTU"
            formation_name = ""
            elapsed_time = 0.0
            
            if self.leader_moving:
                formation_name = self.formation_states_distance[self.current_formation_idx_distance]["name"]
            else:
                formation_name = self.formation_states_time[self.current_formation_idx_time]["name"]
                if self.stopped_start_time is not None:
                    elapsed_time = (self.get_clock().now() - self.stopped_start_time).nanoseconds * 1e-9
            
            self.get_logger().info(
                f"Mode: {mode_str} | "
                f"Formasi: {formation_name} | "
                f"{'Jarak: ' + f'{self.total_distance:.1f}' + 'm' if self.leader_moving else 'Waktu: ' + f'{elapsed_time:.1f}' + 's'} | "
                f"Error: {distance_error:.2f}m, {math.degrees(eyaw):.0f}° | "
                f"CMD: ({vx:.2f}, {vy:.2f}, {w:.2f}) | "
                f"Offset: ({self.offset_x:.1f}, {self.offset_y:.1f})"
            )
            self.last_log_time = current_time
        
        self.iteration_count += 1

    def publish_zero_vel(self):
        """Mengirim kecepatan nol dan reset integral"""
        cmd = Twist()
        cmd.linear.x = 0.0
        cmd.linear.y = 0.0
        cmd.angular.z = 0.0
        self.cmd_pub.publish(cmd)
        
        # Reset integral term saat berhenti
        self.errSum = np.zeros((3, 1))

    def yaw_from_q(self, q):
        """Ekstrak yaw dari quaternion"""
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        return math.atan2(siny_cosp, cosy_cosp)

    def normalize_angle(self, a):
        """Normalisasi sudut ke range [-pi, pi]"""
        while a > math.pi:
            a -= 2.0 * math.pi
        while a < -math.pi:
            a += 2.0 * math.pi
        return a

    def clamp(self, v, vmax):
        """Clamp nilai antara -vmax dan vmax"""
        return max(min(v, vmax), -vmax)


def main():
    rclpy.init()
    node = Robot2FollowerPID()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()