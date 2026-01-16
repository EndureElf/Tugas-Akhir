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

        # Formasi dinamis berdasarkan jarak
        self.formation_states = [
            {"distance": 0.0, "offset_x": -0.5, "offset_y": -0.5, "name": "belakang-kanan"},
            {"distance": 2.0, "offset_x": -0.5, "offset_y": 0.0, "name": "belakang"},
            {"distance": 4.0, "offset_x": -0.5, "offset_y": 0.5, "name": "belakang-kiri"},
            {"distance": 7.0, "offset_x": 0.0, "offset_y": 0.5, "name": "kiri-pas"},
        ]
        
        # Formasi offset saat ini
        self.offset_x = -0.5   # belakang leader (m)
        self.offset_y = -0.5    # kanan leader (m)
        self.current_formation_idx = 0
        self.formation_changed = False
        
        # Nilai Kp, Ki, Kd hasil tuning
        self.Kp = np.diag([1.9, 1.9, 1.0]) 
        self.Ki = np.diag([0.1, 0.1, 0.1])
        self.Kd = np.diag([0.0001, 0.0001, 0.001])
        
        # Feedforward gain tetap sama
        self.alpha_ff = np.diag([0.5, 0.5, 0.3])

        # Kecepatan maksimum untuk formasi berbeda
        self.vx_max_belakang_kanan = 0.3   # Normal
        self.vy_max_belakang_kanan = 0.3
        self.w_max_belakang_kanan = 1.0
        
        self.vx_max_belakang = 0.28        # Lebih lambat
        self.vy_max_belakang = 0.28
        self.w_max_belakang = 0.8
        
        self.vx_max_belakang_kiri = 0.3    # Normal
        self.vy_max_belakang_kiri = 0.3
        self.w_max_belakang_kiri = 1.0
        
        self.vx_max_kiri_pas = 0.42       # Lebih cepat (mengejar leader)
        self.vy_max_kiri_pas = 0.42
        self.w_max_kiri_pas = 1.0
        
        # Default kecepatan maksimum
        self.vx_max = 0.3
        self.vy_max = 0.3
        self.w_max = 1.0

        # Tracking jarak lintasan
        self.total_distance = 0.0
        self.last_leader_position = None
        self.distance_traveled_since_last_formation = 0.0

        self.leader_pose = None      # AMCL pose terbaru dari leader
        self.leader_cmd = None       # Cmd_vel terbaru dari leader
        self.follower_pose = None    # Pose follower sendiri

        self.last_leader_pose = None     # Leader pose sebelumnya untuk komparasi
        self.global_leader_pose = None   # Posisi leader (AMCL atau dead reckoning)
        
        self.last_leader_update = self.get_clock().now()
        
        self.errSum = np.zeros((3, 1))
        self.lastErr = np.zeros((3, 1))
        self.prev_time = self.get_clock().now()

        self.vel_deadband = 0.01          # Deadband kecepatan
        self.pos_deadband = 0.03          # Deadband posisi (3 cm)
        self.yaw_deadband = 0.05          # Deadband orientasi (3 derajat)
        
        # Timeouts
        self.amcl_timeout = 0.5           # Timeout AMCL data
        
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

        # Setup UDP socket untuk menerima data dari leader
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.bind(('0.0.0.0', 8891))
        self.sock.settimeout(0.1)
        threading.Thread(target=self.udp_listener, daemon=True).start()

        # Timer untuk control loop (20 Hz)
        self.timer = self.create_timer(0.05, self.control_loop)

        self.get_logger().info("Robot2 Follower PID Start - Formasi Dinamis")
        self.get_logger().info(f"Gain PID: Kp={np.diag(self.Kp)}, Ki={np.diag(self.Ki)}, Kd={np.diag(self.Kd)}")

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
                if self.last_leader_pose is not None:
                    dx = new_pose["x"] - self.last_leader_pose["x"]
                    dy = new_pose["y"] - self.last_leader_pose["y"]
                    distance_step = math.sqrt(dx**2 + dy**2)
                    
                    # Update total jarak
                    self.total_distance += distance_step
                    self.distance_traveled_since_last_formation += distance_step
                    
                    # Cek apakah perlu ganti formasi
                    self.check_formation_change()
                
                # Jika lokasi AMCL != last lokasi AMCL
                if self.last_leader_pose is None:
                    self.leader_pose = new_pose
                    self.global_leader_pose = new_pose.copy()
                    self.last_leader_pose = new_pose.copy()
                    self.get_logger().info(f"First pose: ({new_pose['x']:.2f}, {new_pose['y']:.2f})")
                else:
                    # Bandingkan dengan pose sebelumnya
                    dx = abs(new_pose["x"] - self.last_leader_pose["x"])
                    dy = abs(new_pose["y"] - self.last_leader_pose["y"])
                    dyaw = abs(new_pose["yaw"] - self.last_leader_pose["yaw"])
                    
                    # Threshold untuk menentukan "berbeda" - 1 cm
                    threshold = 0.01
                    
                    if dx > threshold or dy > threshold or dyaw > threshold:
                        # Jika AMCL berbeda, maka menggunakan AMCL langsung
                        self.leader_pose = new_pose
                        self.global_leader_pose = new_pose.copy()
                        self.last_leader_pose = new_pose.copy()
                        
                        self.get_logger().debug(f"AMCL changed: Using new AMCL")
                    else:
                        # Jika AMCL sama, maka akan menuju dead reckoning 
                        self.leader_pose = new_pose
                        self.last_leader_pose = new_pose.copy()
                        # global_leader_pose TIDAK di-update dari AMCL
                        # (akan lanjut dead reckoning di control loop)
                        
                        self.get_logger().debug(f"AMCL same: Continuing dead reckoning")
                
            except socket.timeout:
                continue
            except Exception as e:
                self.get_logger().error(f"UDP error: {e}")

    def check_formation_change(self):
        """Cek apakah perlu mengubah formasi berdasarkan jarak yang ditempuh"""
        for i, state in enumerate(self.formation_states):
            if i <= self.current_formation_idx:
                continue
                
            if self.total_distance >= state["distance"]:
                # Ganti formasi
                self.current_formation_idx = i
                self.offset_x = state["offset_x"]
                self.offset_y = state["offset_y"]
                self.formation_changed = True
                self.distance_traveled_since_last_formation = 0.0
                
                # Reset integral term saat ganti formasi
                self.errSum = np.zeros((3, 1))
                
                # Set kecepatan maksimum berdasarkan formasi spesifik
                if state["name"] == "belakang-kanan":
                    self.vx_max = self.vx_max_belakang_kanan
                    self.vy_max = self.vy_max_belakang_kanan
                    self.w_max = self.w_max_belakang_kanan
                elif state["name"] == "belakang":
                    self.vx_max = self.vx_max_belakang
                    self.vy_max = self.vy_max_belakang
                    self.w_max = self.w_max_belakang
                elif state["name"] == "belakang-kiri":
                    self.vx_max = self.vx_max_belakang_kiri
                    self.vy_max = self.vy_max_belakang_kiri
                    self.w_max = self.w_max_belakang_kiri
                elif state["name"] == "kiri-pas":
                    self.vx_max = self.vx_max_kiri_pas
                    self.vy_max = self.vy_max_kiri_pas
                    self.w_max = self.w_max_kiri_pas
                else:
                    # Fallback ke normal
                    self.vx_max = 0.3
                    self.vy_max = 0.3
                    self.w_max = 1.0
                
                self.get_logger().info(f"✅ Formasi berubah ke: {state['name']} (jarak: {self.total_distance:.2f}m)")
                self.get_logger().info(f"   Offset: ({self.offset_x}, {self.offset_y})")
                self.get_logger().info(f"   Max Vel: ({self.vx_max:.2f}, {self.vy_max:.2f}, {self.w_max:.2f})")
                break

    def update_global_leader_pose(self, dt):
        """
        Update global leader pose dengan dead reckoning jika tidak ada update AMCL
        atau jika AMCL sama dengan sebelumnya
        """
        if self.global_leader_pose is None:
            return None
            
        # Jika ada cmd_vel dan global_leader_pose sudah diinisialisasi
        if self.leader_cmd is not None:
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
                
                # Cek apakah perlu ganti formasi
                self.check_formation_change()
            
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
        
        # Hitung dt untuk kontrol dan dead reckoning
        dt = (now - self.prev_time).nanoseconds * 1e-9
        self.prev_time = now
        if dt <= 0.0:
            return
        
        # Update global leader pose dengan dead reckoning
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
        
        # Jika baru ganti formasi, beri waktu adaptasi
        if self.formation_changed:
            self.get_logger().info(f"Adaptasi formasi baru... Jarak ke target: {math.sqrt((xt-xf)**2 + (yt-yf)**2):.2f}m")
            self.formation_changed = False
        
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
            self.get_logger().debug(f"✓ Target reached! Error: {distance_error:.3f}m, {math.degrees(eyaw):.1f}°")
            self.publish_zero_vel()
            return  # Keluar dari control loop, robot berhenti
        
        # PID Control dengan gain yang sudah di-tuning
        e = np.array([[ex], [ey], [eyaw]])
        
        # Integral term dengan anti-windup
        self.errSum += e * dt
        self.errSum = np.clip(self.errSum, -1.0, 1.0)
        
        # Derivative term
        dErr = (e - self.lastErr) / dt
        self.lastErr = e
        
        # PID output dengan gain yang tetap sama
        u_pid = self.Kp @ e + self.Ki @ self.errSum + self.Kd @ dErr
        
        # Feedforward
        ff_term = np.zeros((3, 1))
        if self.leader_cmd is not None:
            # Ambil kecepatan leader dari cmd_vel
            vx_leader = self.leader_cmd["vx"]
            vy_leader = self.leader_cmd["vy"]
            w_leader = self.leader_cmd["w"]

            # Transformasi kecepatan leader ke koordinat follower
            vx_offset = vx_leader - w_leader * self.offset_y
            vy_offset = vy_leader + w_leader * self.offset_x

            # Rotasi ke koordinat follower
            theta_rel = thetaf - thetal  # Sudut relatif follower terhadap leader
            vx_ff = vx_offset * math.cos(theta_rel) + vy_offset * math.sin(theta_rel)
            vy_ff = -vx_offset * math.sin(theta_rel) + vy_offset * math.cos(theta_rel)
            w_ff = w_leader
            
            ff_term = np.array([[vx_ff], [vy_ff], [w_ff]])
        
        # Total output PID + Feedforward
        u = u_pid + self.alpha_ff @ ff_term

        # Clamp velocity dengan batas maksimum formasi
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
        
        # Logging periodik
        formation_name = self.formation_states[self.current_formation_idx]["name"]
        self.get_logger().info(
            f"Formasi: {formation_name} | "
            f"Jarak tempuh: {self.total_distance:.2f}m | "
            f"Target: ({xt:.2f}, {yt:.2f}) | "
            f"Current: ({xf:.2f}, {yf:.2f}) | "
            f"Error: {distance_error:.3f}m, {math.degrees(eyaw):.1f}° | "
            f"CMD: ({vx:.2f}, {vy:.2f}, {w:.2f})"
        )

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