#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, PoseWithCovarianceStamped, PoseStamped
from nav_msgs.msg import Path
from sensor_msgs.msg import LaserScan
import numpy as np
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
import math
import socket
import json
import threading
import time


class Robot2DualModeController(Node):

    def __init__(self):
        super().__init__('robot2_dual_mode_controller')

        # ========== MODE OPERASI ==========
        self.MODE_FOLLOWER = 1
        self.MODE_DWA = 2
        self.current_mode = self.MODE_FOLLOWER
        self.transition_threshold = 10.0  # detik (transisi saat ETA_elapsed ≥ 4s)
        self.transition_triggered = False
        self.transition_complete = False
        
        # Flag untuk goal processing
        self.goal_received = False
        self.goal_published = False

        # ========== OFFSET & PARAMETERS ==========
        # Formasi offset untuk follower mode
        self.offset_x = 0.0   # belakang leader (m)
        self.offset_y = -0.4  # kiri leader (m)
        
        # Kecepatan follower mode (5/3 dari leader)
        self.follower_speed_multiplier = 5.0/3.0  # 5/3 = 1.666x
        self.leader_max_vel_x = 0.8  # Default, akan diupdate dari leader
        self.leader_max_vel_y = 0.2  # Default, akan diupdate dari leader
        self.last_leader_speed_update = 0.0  # Timestamp update terakhir
        self.speed_update_interval = 1.0  # Update kecepatan setiap 1 detik
        
        # Offset untuk goal pose transformasi
        self.goal_distance = 0.3   # jarak di depan leader (m)
        
        # Goal pose yang ditransformasi
        self.transformed_goal_pose = None
        self.leader_goal_pose = None  # Goal asli dari leader
        
        # ========== PARAMETERS FOLLOWER PID ==========
        self.Kp = np.diag([1.9, 1.9, 1.0])
        self.Ki = np.diag([0.06, 0.06, 0.06])
        self.Kd = np.diag([0.0001, 0.0001, 0.001])
        self.alpha_ff = np.diag([0.5, 0.5, 0.3])
        
        # Velocity limits (default, akan diupdate berdasarkan leader)
        self.vx_max = 0.5
        self.vy_max = 0.5
        self.w_max = 1.0
        
        # Deadbands
        self.vel_deadband = 0.01
        self.pos_deadband = 0.03
        self.yaw_deadband = 0.05
        self.amcl_timeout = 0.5
        
        # ========== PARAMETERS DWA ==========
        # Motion limits untuk DWA (DEFAULT - akan diupdate oleh ETA)
        self.dwa_max_vel_x = 0.3    # Forward/backward
        self.dwa_max_vel_y = 0.2    # Sideways
        self.dwa_max_rot_vel = 0.5  # Rotation
        
        # ETA Parameters (sama dengan leader)
        self.target_time = 30.0  # Default, akan diupdate dari leader
        self.min_speed = 0.1
        self.max_speed = 0.8
        self.stop_time_threshold = 0.8
        self.safety_margin = 1.1
        self.scaling_factor = 1.2  # Sama dengan leader (1.2)
        
        # Mission timing (sinkron dengan leader)
        self.mission_start = None
        self.time_elapsed = 0.0
        self.time_remaining = self.target_time
        self.mission_active = False
        self.mission_stopped = False
        
        # Path tracking untuk ETA
        self.total_path_length = 0.0
        self.remaining_path_length = 0.0
        self.current_speed = 0.0
        self.needed_speed = 0.0
        
        # Path tracking parameters
        self.dwa_lookahead_distance = 0.5
        self.dwa_goal_tolerance = 0.15
        self.dwa_path_tracking_tolerance = 0.3
        
        # Heading alignment parameters
        self.dwa_heading_alignment_tolerance = 0.2  # rad (~11.5°)
        self.dwa_min_heading_error_for_sideways = 0.3  # rad (~17°)
        self.dwa_max_sideways_ratio = 0.3
        
        # DWA Parameters
        self.dwa_prediction_time = 1.5
        self.dwa_dt = 0.1
        self.dwa_vx_samples = 9
        self.dwa_vy_samples = 7
        self.dwa_w_samples = 11
        
        # DWA Behavior states
        self.DWA_STATE_ALIGNING = 1
        self.DWA_STATE_TRACKING = 2
        self.DWA_STATE_APPROACHING = 3
        self.DWA_STATE_FINAL_ALIGNING = 4
        self.DWA_STATE_IDLE = 5
        self.dwa_state = self.DWA_STATE_ALIGNING
        
        # ========== STATE VARIABLES ==========
        # Follower state
        self.leader_pose = None
        self.leader_cmd = None
        self.follower_pose = None
        self.last_leader_pose = None
        self.global_leader_pose = None
        self.last_leader_update = self.get_clock().now()
        
        # PID terms
        self.errSum = np.zeros((3, 1))
        self.lastErr = np.zeros((3, 1))
        self.prev_time = self.get_clock().now()
        
        # Leader data untuk logging
        self.leader_eta = None
        self.leader_dwa_state = None
        self.leader_path_info = None
        self.leader_velocity_params = None
        
        # DWA state
        self.robot_vel = [0.0, 0.0, 0.0]
        self.laser_data = None
        self.global_path = []  # Path untuk DWA (dari global planner)
        self.current_path_index = 0
        self.progress_along_segment = 0.0
        self.target_point = None
        self.target_heading = 0.0
        self.position_reached = False
        self.final_goal_pose = None
        
        # DWA performance tracking
        self.last_target_update = time.time()
        self.stuck_check_interval = 2.0  # detik
        self.last_robot_position = None
        self.stuck_counter = 0
        self.max_stuck_count = 5
        
        # Statistics
        self.commands_sent = 0
        self.max_speed_commands = 0
        self.stop_commands_sent = 0
        
        # Flags untuk mengurangi logging spam
        self.initial_waiting_logged = False
        self.stale_data_warning_logged = False
        self.speed_update_count = 0
        self.last_log_time = 0.0
        self.dwa_state_logged = {}
        
        # ========== PUBLISHERS & SUBSCRIBERS ==========
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
        
        # Subscriber untuk laser scan (DWA mode)
        self.create_subscription(
            LaserScan,
            '/robot2/scan',
            self.scan_callback,
            10
        )
        
        # SUBSCRIBER BARU: Global path dari path planner
        self.create_subscription(
            Path,
            '/robot2/plan',
            self.path_callback,
            10
        )
        
        # Publisher untuk cmd_vel (digunakan oleh kedua mode)
        self.cmd_pub = self.create_publisher(Twist, '/robot2/cmd_vel', 10)
        
        # Publisher untuk goal pose yang ditransformasi (hanya sekali saat menerima goal)
        self.goal_pub = self.create_publisher(PoseStamped, '/robot2/goal_pose', 10)
        
        # Publisher untuk local plan visualization (DWA mode)
        self.local_plan_pub = self.create_publisher(Path, '/robot2/local_plan', 10)
        
        # ========== UDP COMMUNICATION ==========
        # Setup UDP socket untuk menerima data dari leader
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.bind(('0.0.0.0', 8891))
        self.sock.settimeout(0.1)
        threading.Thread(target=self.udp_listener, daemon=True).start()
        
        # ========== TIMERS ==========
        # Timer untuk control loop (20 Hz)
        self.timer = self.create_timer(0.05, self.control_loop)
        
        # Timer untuk logging status (2 Hz)
        self.status_timer = self.create_timer(0.5, self.log_status)
        
        # Timer untuk ETA loop (5 Hz)
        self.eta_timer = self.create_timer(0.2, self.eta_loop)
        
        self.get_logger().info("🤖 Robot2 Dual Mode Controller Started")
        self.get_logger().info(f"📍 Current mode: FOLLOWER")
        self.get_logger().info(f"⏰ Transition at: ETA_elapsed ≥ {self.transition_threshold}s")
        self.get_logger().info(f"🎯 Goal will be published ONCE when received")
        self.get_logger().info(f"🚀 Follower speed multiplier: {self.follower_speed_multiplier:.2f}x")

    # ========== CALLBACK FUNCTIONS ==========
    def follower_cb(self, msg):
        """Callback untuk pose follower sendiri"""
        self.follower_pose = msg.pose.pose

    def scan_callback(self, msg):
        """Callback untuk laser scan (DWA mode)"""
        self.laser_data = msg

    def path_callback(self, msg):
        """CALLBACK BARU: Menerima global path dari path planner"""
        if msg.poses:
            new_path = []
            
            for pose in msg.poses:
                x = pose.pose.position.x
                y = pose.pose.position.y
                
                # Extract orientation dari path
                q = pose.pose.orientation
                theta = self.yaw_from_q(q)
                
                new_path.append([x, y, theta])
            
            if len(new_path) >= 2:
                self.global_path = new_path
                self.get_logger().info(f"📈 Received global path: {len(self.global_path)} points")
                
                # Reset tracking state
                self.current_path_index = 0
                self.progress_along_segment = 0.0
                self.position_reached = False
                
                # Hitung path length untuk ETA
                self.calculate_path_length()
                
                # Log path info
                if len(self.global_path) > 0:
                    start_x, start_y, _ = self.global_path[0]
                    goal_x, goal_y, _ = self.global_path[-1]
                    self.get_logger().info(
                        f"   Start: ({start_x:.2f}, {start_y:.2f}) → "
                        f"Goal: ({goal_x:.2f}, {goal_y:.2f}) | "
                        f"Length: {self.total_path_length:.2f}m"
                    )

    def udp_listener(self):
        """Thread untuk menerima data UDP dari leader"""
        while rclpy.ok():
            try:
                data, addr = self.sock.recvfrom(2048)
                msg = json.loads(data.decode())
                
                # 1. AMCL Pose dari leader
                if "amcl_pose" in msg:
                    amcl_data = msg["amcl_pose"]
                    position = amcl_data["position"]
                    orientation = amcl_data["orientation"]
                    
                    yaw = self.quaternion_to_yaw(orientation)
                    
                    new_pose = {
                        "x": position["x"],
                        "y": position["y"],
                        "yaw": yaw
                    }
                    
                    self.last_leader_update = self.get_clock().now()
                    
                    if self.last_leader_pose is None:
                        self.leader_pose = new_pose
                        self.global_leader_pose = new_pose.copy()
                        self.last_leader_pose = new_pose.copy()
                    else:
                        dx = abs(new_pose["x"] - self.last_leader_pose["x"])
                        dy = abs(new_pose["y"] - self.last_leader_pose["y"])
                        dyaw = abs(new_pose["yaw"] - self.last_leader_pose["yaw"])
                        
                        threshold = 0.01
                        
                        if dx > threshold or dy > threshold or dyaw > threshold:
                            self.leader_pose = new_pose
                            self.global_leader_pose = new_pose.copy()
                            self.last_leader_pose = new_pose.copy()
                        else:
                            self.leader_pose = new_pose
                            self.last_leader_pose = new_pose.copy()
                
                # 2. Cmd Vel dari leader
                if "cmd_vel" in msg:
                    cmd_data = msg["cmd_vel"]
                    linear = cmd_data["linear"]
                    angular = cmd_data["angular"]
                    
                    self.leader_cmd = {
                        "vx": linear["x"],
                        "vy": linear["y"],
                        "w": angular["z"]
                    }
                
                # 3. ETA Information - DISINKRONKAN!
                if "eta" in msg:
                    eta_data = msg["eta"]
                    # SIMPAN waktu dari leader (TIDAK menghitung ulang)
                    self.time_elapsed = eta_data.get('time_elapsed', 0)
                    self.time_remaining = eta_data.get('time_remaining', 0)
                    self.target_time = eta_data.get('target_time', 30.0)
                    
                    # Sinkronkan mission state dengan leader
                    if "mission_active" in msg:
                        self.mission_active = msg["mission_active"]
                    if "mission_stopped" in msg:
                        self.mission_stopped = msg["mission_stopped"]
                    
                    # Simpan juga untuk transition check
                    self.leader_eta = eta_data
                
                # 4. Goal Pose dari leader - DIPROSES LANGSUNG!
                if "goal_pose" in msg:
                    if not self.goal_received:  # Hanya proses pertama kali
                        goal_data = msg["goal_pose"]
                        position = goal_data["position"]
                        orientation = goal_data["orientation"]
                        
                        yaw = self.quaternion_to_yaw(orientation)
                        
                        self.leader_goal_pose = {
                            "x": position["x"],
                            "y": position["y"],
                            "yaw": yaw
                        }
                        
                        self.get_logger().info(
                            f"📋 Received leader goal: "
                            f"({self.leader_goal_pose['x']:.2f}, {self.leader_goal_pose['y']:.2f}, "
                            f"{math.degrees(self.leader_goal_pose['yaw']):.1f}°)"
                        )
                        
                        # TRANSFORM & PUBLISH SEKARANG JUGA!
                        self.process_leader_goal()
                        self.goal_received = True
                
                # 5. Path Information (untuk ETA calculation)
                if "path_info" in msg:
                    path_data = msg["path_info"]
                    self.total_path_length = path_data.get('total_length', 0.0)
                    self.remaining_path_length = path_data.get('remaining_length', 0.0)
                    self.needed_speed = path_data.get('needed_speed', 0.0)
                    self.current_speed = path_data.get('current_speed', 0.0)
                    self.leader_path_info = path_data
                
                # 6. Velocity Parameters (untuk sinkronisasi kecepatan)
                if "velocity" in msg:
                    velocity_data = msg["velocity"]
                    # Update leader velocity parameters (untuk follower mode)
                    new_leader_max_vel_x = velocity_data.get('max_vel_x', 0.8)
                    new_leader_max_vel_y = velocity_data.get('max_vel_y', 0.2)
                    
                    # Update DWA parameters berdasarkan leader
                    self.dwa_max_vel_x = velocity_data.get('max_vel_x', 0.3)
                    self.dwa_max_vel_y = velocity_data.get('max_vel_y', 0.2)
                    self.dwa_max_rot_vel = velocity_data.get('max_rot_vel', 0.5)
                    self.leader_velocity_params = velocity_data
                    
                    # Update kecepatan follower (5/3 dari leader) dengan interval
                    current_time = time.time()
                    if (current_time - self.last_leader_speed_update >= self.speed_update_interval or
                        abs(new_leader_max_vel_x - self.leader_max_vel_x) > 0.05 or
                        abs(new_leader_max_vel_y - self.leader_max_vel_y) > 0.02):
                        
                        # Update leader speed
                        self.leader_max_vel_x = new_leader_max_vel_x
                        self.leader_max_vel_y = new_leader_max_vel_y
                        
                        # Update follower speed (5/3 dari leader)
                        self.vx_max = self.leader_max_vel_x * self.follower_speed_multiplier
                        self.vy_max = self.leader_max_vel_y * self.follower_speed_multiplier
                        
                        self.last_leader_speed_update = current_time
                        self.speed_update_count += 1
                        
                        # Log hanya setiap 5 update
                        if self.speed_update_count % 5 == 0:
                            self.get_logger().info(
                                f"⚡ Follower speed updated #{self.speed_update_count}: "
                                f"Leader max_vel_x={self.leader_max_vel_x:.2f}m/s → "
                                f"Follower max_vel_x={self.vx_max:.2f}m/s "
                                f"({self.follower_speed_multiplier:.2f}x)"
                            )
                
                # 7. DWA State
                if "dwa_state" in msg:
                    self.leader_dwa_state = msg["dwa_state"]
                
            except socket.timeout:
                continue
            except json.JSONDecodeError as e:
                self.get_logger().error(f"JSON decode error: {e}")
                continue
            except Exception as e:
                self.get_logger().error(f"UDP error: {e}")
                continue

    def process_leader_goal(self):
        """PROSES GOAL POSE SAAT DITERIMA: Transform + Publish SEKALI"""
        if self.leader_goal_pose is None:
            return
        
        # 1. Transform goal pose (offset 2m depan + rotasi 180°)
        success = self.transform_goal_pose()
        if not success:
            return
        
        # 2. Publish ke /robot2/goal_pose SEKALI
        success = self.publish_transformed_goal_once()
        if not success:
            return
        
        self.get_logger().info("✅ Goal processed: Transformed and published to /robot2/goal_pose")
        
        # 3. Simpan untuk DWA (global path akan dibuat saat transisi)
        if self.transformed_goal_pose is not None:
            self.final_goal_pose = [
                self.transformed_goal_pose["x"],
                self.transformed_goal_pose["y"],
                self.transformed_goal_pose["yaw"]
            ]
            
            self.get_logger().info(
                f"🎯 DWA Goal ready for transition: "
                f"({self.final_goal_pose[0]:.2f}, {self.final_goal_pose[1]:.2f}, "
                f"{math.degrees(self.final_goal_pose[2]):.1f}°)"
            )

    def transform_goal_pose(self):
        """
        Transform goal pose dari leader dengan offset di depan dan rotasi 180 derajat
        """
        if self.leader_goal_pose is None:
            self.get_logger().error("❌ Cannot transform goal: No leader goal pose")
            return False
        
        # Ambil goal pose leader
        x_goal = self.leader_goal_pose["x"]
        y_goal = self.leader_goal_pose["y"]
        yaw_goal = self.leader_goal_pose["yaw"]
        
        # Hitung offset di depan goal (sejauh goal_distance)
        # Arah berdasarkan orientasi goal leader
        offset_x_front = self.goal_distance
        
        # Transformasi offset ke koordinat global
        xt = x_goal - 0.2 * math.cos(yaw_goal)
        yt = y_goal + offset_x_front * math.sin(yaw_goal)
        
        # Rotasi orientasi 180 derajat dari orientasi goal leader
        thetat = self.normalize_angle(yaw_goal + math.pi)
        
        # Simpan transformed goal pose
        self.transformed_goal_pose = {
            "x": xt,
            "y": yt,
            "yaw": thetat
        }
        
        # Log transformasi
        self.get_logger().info(
            f"🔄 Goal Transformation:\n"
            f"  Leader goal: ({x_goal:.2f}, {y_goal:.2f}, {math.degrees(yaw_goal):.1f}°)\n"
            f"  Transformed: ({xt:.2f}, {yt:.2f}, {math.degrees(thetat):.1f}°)\n"
            f"  Offset: {self.goal_distance:.2f}m di depan, rotasi 180°"
        )
        
        return True

    def publish_transformed_goal_once(self):
        """Publish transformed goal pose ke topic /robot2/goal_pose (hanya sekali)"""
        if self.transformed_goal_pose is None:
            self.get_logger().error("❌ Cannot publish goal: No transformed goal pose")
            return False
        
        if self.goal_published:
            self.get_logger().debug("Goal already published")
            return True
        
        goal_msg = PoseStamped()
        goal_msg.header.stamp = self.get_clock().now().to_msg()
        goal_msg.header.frame_id = 'map'
        
        goal_msg.pose.position.x = self.transformed_goal_pose["x"]
        goal_msg.pose.position.y = self.transformed_goal_pose["y"]
        goal_msg.pose.position.z = 0.0
        
        # Convert yaw to quaternion
        yaw = self.transformed_goal_pose["yaw"]
        qz = math.sin(yaw / 2.0)
        qw = math.cos(yaw / 2.0)
        
        goal_msg.pose.orientation.x = 0.0
        goal_msg.pose.orientation.y = 0.0
        goal_msg.pose.orientation.z = qz
        goal_msg.pose.orientation.w = qw
        
        self.goal_pub.publish(goal_msg)
        self.goal_published = True
        
        self.get_logger().info(f"📤 Published to /robot2/goal_pose: "
                              f"({self.transformed_goal_pose['x']:.2f}, "
                              f"{self.transformed_goal_pose['y']:.2f}, "
                              f"{math.degrees(self.transformed_goal_pose['yaw']):.1f}°)")
        
        return True

    def quaternion_to_yaw(self, q):
        """Convert quaternion dictionary to yaw angle"""
        x = q["x"]
        y = q["y"]
        z = q["z"]
        w = q["w"]
        
        siny_cosp = 2.0 * (w * z + x * y)
        cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
        yaw = math.atan2(siny_cosp, cosy_cosp)
        
        return yaw

    def update_global_leader_pose(self, dt):
        """Update global leader pose dengan dead reckoning"""
        if self.global_leader_pose is None:
            return None
            
        if self.leader_cmd is not None:
            vx = self.leader_cmd["vx"]
            vy = self.leader_cmd["vy"]
            w = self.leader_cmd["w"]
            
            theta = self.global_leader_pose["yaw"]
            
            self.global_leader_pose["x"] += (vx * math.cos(theta) - vy * math.sin(theta)) * dt
            self.global_leader_pose["y"] += (vx * math.sin(theta) + vy * math.cos(theta)) * dt
            self.global_leader_pose["yaw"] += w * dt
            
            self.global_leader_pose["yaw"] = self.normalize_angle(self.global_leader_pose["yaw"])
        
        return self.global_leader_pose

    def check_transition_condition(self):
        """Cek apakah kondisi untuk transisi sudah terpenuhi - BENAR: time_elapsed ≥ threshold"""
        # Hanya cek jika masih dalam follower mode dan belum triggered
        if self.current_mode != self.MODE_FOLLOWER or self.transition_triggered:
            return False
        
        # Cek apakah ada data ETA dan goal sudah diproses
        if self.leader_eta is None or not self.goal_received:
            return False
        
        # Ambil waktu yang sudah berjalan dari leader
        time_elapsed = self.leader_eta.get('time_elapsed', 0)
        
        # BENAR: Transisi saat time_elapsed ≥ threshold (4 detik)
        if time_elapsed >= self.transition_threshold:
            time_remaining = self.leader_eta.get('time_remaining', 0)
            time_total = time_elapsed + time_remaining
            
            self.get_logger().warn(
                f"🚨 TRANSITION TRIGGERED: "
                f"Leader elapsed {time_elapsed:.1f}s ≥ {self.transition_threshold}s "
                f"(Total: {time_total:.1f}s, Remaining: {time_remaining:.1f}s)"
            )
            return True
        
        return False

    def control_loop(self):
        """Main control loop - memilih mode berdasarkan state"""
        
        # CEK TRANSISI DI CONTROL LOOP UTAMA
        if (not self.transition_triggered and 
            self.current_mode == self.MODE_FOLLOWER):
            
            if self.check_transition_condition():
                self.transition_triggered = True
        
        # Cek dan lakukan transisi jika diperlukan
        if self.transition_triggered and not self.transition_complete:
            self.perform_transition()
            return
        
        # Pilih mode kontrol berdasarkan current_mode
        if self.current_mode == self.MODE_FOLLOWER:
            self.follower_control_loop()
        elif self.current_mode == self.MODE_DWA:
            self.dwa_control_loop()

    def perform_transition(self):
        """Perform transisi dari FOLLOWER mode ke DWA mode"""
        self.get_logger().warn("🔄 TRANSISI: Switching to DWA mode...")
        
        # 1. Hentikan robot
        self.publish_zero_vel()
        
        # 2. Tunggu sejenak untuk stabilisasi
        time.sleep(0.5)
        
        # 3. Update mode operasi
        self.current_mode = self.MODE_DWA
        
        # 4. Pastikan path sudah ada sebelum transisi
        if not self.global_path or len(self.global_path) < 2:
            self.get_logger().error("❌ No global path available for DWA!")
            self.get_logger().info("⚠️ Fallback to straight-line path")
            
            # Fallback: buat path sederhana
            if self.follower_pose is not None and self.final_goal_pose is not None:
                xf = self.follower_pose.position.x
                yf = self.follower_pose.position.y
                thetaf = self.yaw_from_q(self.follower_pose.orientation)
                
                # Buat path dengan beberapa intermediate points untuk smoothness
                self.global_path = [
                    [xf, yf, thetaf],
                    [(xf + self.final_goal_pose[0])/2, (yf + self.final_goal_pose[1])/2, 
                     (thetaf + self.final_goal_pose[2])/2],
                    self.final_goal_pose
                ]
            else:
                self.get_logger().error("❌ Cannot create fallback path!")
                return
        
        # 5. Reset state machine DWA dengan path yang ada
        self.dwa_state = self.DWA_STATE_ALIGNING
        self.current_path_index = 0
        self.progress_along_segment = 0.0
        self.position_reached = False
        
        # 6. Hitung ulang path length
        self.calculate_path_length()
        
        self.get_logger().info(
            f"🛣️ DWA Path ready:\n"
            f"  Start: ({self.global_path[0][0]:.2f}, {self.global_path[0][1]:.2f})\n"
            f"  Goal: ({self.global_path[-1][0]:.2f}, {self.global_path[-1][1]:.2f})\n"
            f"  Points: {len(self.global_path)}, Length: {self.total_path_length:.2f}m"
        )
        
        # 7. Tandai transisi selesai
        self.transition_complete = True
        
        self.get_logger().info("✅ TRANSISI SELESAI: Now in DWA mode")

    # ========== ETA FUNCTIONS (SINKRON DENGAN LEADER) ==========
    def eta_loop(self):
        """Loop ETA untuk mengatur kecepatan DWA (sinkron dengan leader)"""
        if not self.mission_active or self.mission_stopped:
            return
        
        # Waktu sudah disinkronkan via UDP, langsung gunakan
        
        # Cek jika waktu habis
        if self.time_remaining <= 0:
            self.handle_time_up()
            return
        
        # Cek jika waktu hampir habis
        if 0 < self.time_remaining <= self.stop_time_threshold:
            self.handle_approaching_stop()
            return
        
        # Hitung path length jika dalam DWA mode dan ada path
        if self.current_mode == self.MODE_DWA and self.global_path and len(self.global_path) >= 2:
            self.calculate_path_length()
            
            # Hitung kecepatan yang diperlukan berdasarkan waktu tersisa dari leader
            if self.time_remaining > 0 and self.remaining_path_length > 0:
                self.needed_speed = self.calculate_needed_speed()
                
                # Update DWA velocity parameters
                self.update_dwa_speed_parameters()
                
                # Log kondisi khusus
                self.log_speed_conditions()

    def calculate_path_length(self):
        """Hitung total path length dan remaining path length untuk DWA"""
        if not self.global_path or len(self.global_path) < 2:
            self.total_path_length = 0.0
            self.remaining_path_length = 0.0
            return
        
        # Hitung total path length
        total_length = 0.0
        for i in range(len(self.global_path) - 1):
            x1, y1, _ = self.global_path[i]
            x2, y2, _ = self.global_path[i + 1]
            total_length += math.hypot(x2 - x1, y2 - y1)
        
        self.total_path_length = total_length
        
        # Hitung remaining path length dari posisi saat ini
        if not self.follower_pose:
            self.remaining_path_length = total_length
            return
        
        robot_x = self.follower_pose.position.x
        robot_y = self.follower_pose.position.y
        remaining_length = 0.0
        
        # Cari segmen saat ini
        if self.current_path_index < len(self.global_path) - 1:
            # Hitung dari posisi saat ini ke akhir segmen
            x1, y1, _ = self.global_path[self.current_path_index]
            x2, y2, _ = self.global_path[self.current_path_index + 1]
            
            # Posisi saat ini pada segmen
            seg_x = x1 + self.progress_along_segment * (x2 - x1)
            seg_y = y1 + self.progress_along_segment * (y2 - y1)
            
            # Sisa segmen saat ini
            remaining_length += math.hypot(x2 - seg_x, y2 - seg_y)
            
            # Tambahkan semua segmen berikutnya
            for i in range(self.current_path_index + 1, len(self.global_path) - 1):
                x1, y1, _ = self.global_path[i]
                x2, y2, _ = self.global_path[i + 1]
                remaining_length += math.hypot(x2 - x1, y2 - y1)
        
        self.remaining_path_length = max(0.0, remaining_length)

    def calculate_needed_speed(self):
        """Hitung kecepatan yang diperlukan berdasarkan waktu tersisa dari leader"""
        if self.remaining_path_length <= 0 or self.time_remaining <= 0:
            return self.min_speed
        
        needed_speed = self.remaining_path_length / self.time_remaining
        needed_speed *= self.safety_margin  # Safety margin
        
        return needed_speed

    def update_dwa_speed_parameters(self):
        """Update parameter kecepatan DWA berdasarkan kecepatan yang diperlukan"""
        # Terapkan limit kecepatan
        new_speed = max(self.min_speed, min(self.needed_speed, self.max_speed))
        
        # Update DWA velocity limits
        self.dwa_max_vel_x = new_speed
        
        # Update sideways velocity berdasarkan ratio
        self.dwa_max_vel_y = min(self.dwa_max_vel_x * self.dwa_max_sideways_ratio, 0.2)
        
        # Update current speed
        self.current_speed = math.hypot(self.robot_vel[0], self.robot_vel[1])

    def log_speed_conditions(self):
        """Log kondisi kecepatan khusus"""
        # Check for MAX speed
        if self.dwa_max_vel_x >= self.max_speed * 0.95:
            self.max_speed_commands += 1
            if self.max_speed_commands % 10 == 0:  # Log setiap 10 kali
                self.get_logger().warn(f"🚀 MAX SPEED: {self.dwa_max_vel_x:.2f}m/s")

    def handle_approaching_stop(self):
        """Handle saat waktu hampir habis"""
        if not hasattr(self, 'stop_warning_sent'):
            self.get_logger().warn(f"⚠️ APPROACHING STOP: {self.time_remaining:.1f}s remaining")
            self.stop_warning_sent = True

    def handle_time_up(self):
        """Handle ketika waktu habis - STOP robot"""
        if not hasattr(self, 'time_up_handled'):
            self.get_logger().warn("⏰ TIME'S UP! STOPPING robot")
            self.stop_robot_eta()
            self.time_up_handled = True
            self.mission_stopped = True

    def stop_robot_eta(self):
        """Stop robot (kecepatan 0)"""
        self.stop_commands_sent += 1
        self.get_logger().info(f"🛑 STOP Command #{self.stop_commands_sent}: Setting speed to 0")
        
        # Set kecepatan ke 0 untuk semua mode
        self.dwa_max_vel_x = 0.0
        self.dwa_max_vel_y = 0.0
        self.dwa_max_rot_vel = 0.0
        
        # Juga set follower speed ke 0 jika masih dalam follower mode
        self.vx_max = 0.0
        self.vy_max = 0.0
        
        # Hentikan robot
        self.stop_robot()
        self.get_logger().info("✅ Robot STOPPED")

    def follower_control_loop(self):
        """Control loop untuk follower mode"""
        # Validasi data dasar
        if self.global_leader_pose is None or self.follower_pose is None:
            if not self.initial_waiting_logged:
                self.get_logger().warn("⏳ Waiting for initial data...")
                self.initial_waiting_logged = True
            return
        
        # Cek timeout data leader
        now = self.get_clock().now()
        time_since_update = (now - self.last_leader_update).nanoseconds * 1e-9
        if time_since_update > self.amcl_timeout:
            if not self.stale_data_warning_logged:
                self.get_logger().warn(f"⏰ Leader data stale ({time_since_update:.1f}s)! Stopping...")
                self.stale_data_warning_logged = True
            self.publish_zero_vel()
            return
        else:
            self.stale_data_warning_logged = False
        
        # Reset initial waiting flag jika data sudah tersedia
        self.initial_waiting_logged = False
        
        # CEK JIKA WAKTU ETA SUDAH HABIS - STOP!
        if self.time_remaining <= 0 and self.mission_stopped:
            self.publish_zero_vel()
            return
        
        # Hitung dt
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
        
        xt = xl + self.offset_x * math.cos(thetal) - self.offset_y * math.sin(thetal)
        yt = yl + self.offset_x * math.sin(thetal) + self.offset_y * math.cos(thetal)
        thetat = thetal
        
        # Dapatkan pose follower
        xf = self.follower_pose.position.x
        yf = self.follower_pose.position.y
        thetaf = self.yaw_from_q(self.follower_pose.orientation)
        
        # Hitung error
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
        
        # Cek jika sudah mencapai target
        distance_error = math.sqrt(ex**2 + ey**2)
        
        if distance_error < self.pos_deadband and abs(eyaw) < self.yaw_deadband:
            self.publish_zero_vel()
            return
        
        # PID Control
        e = np.array([[ex], [ey], [eyaw]])
        
        self.errSum += e * dt
        self.errSum = np.clip(self.errSum, -1.0, 1.0)
        
        dErr = (e - self.lastErr) / dt
        self.lastErr = e
        
        u_pid = self.Kp @ e + self.Ki @ self.errSum + self.Kd @ dErr
        
        # Feedforward term
        ff_term = np.zeros((3, 1))
        if self.leader_cmd is not None:
            vx_leader = self.leader_cmd["vx"]
            vy_leader = self.leader_cmd["vy"]
            w_leader = self.leader_cmd["w"]

            vx_offset = vx_leader - w_leader * self.offset_y
            vy_offset = vy_leader + w_leader * self.offset_x

            theta_rel = thetaf - thetal
            vx_ff = vx_offset * math.cos(theta_rel) + vy_offset * math.sin(theta_rel)
            vy_ff = -vx_offset * math.sin(theta_rel) + vy_offset * math.cos(theta_rel)
            w_ff = w_leader
            
            ff_term = np.array([[vx_ff], [vy_ff], [w_ff]])
        
        u = u_pid + self.alpha_ff @ ff_term

        # Clamp velocity dengan limit yang sudah diset (5/3 dari leader)
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
        
        # Publish command dengan scaling factor seperti leader
        vx *= self.scaling_factor
        vy *= self.scaling_factor
        
        # Clamp lagi setelah scaling
        vx = max(-self.vx_max, min(self.vx_max, vx))
        vy = max(-self.vy_max, min(self.vy_max, vy))
        
        cmd = Twist()
        cmd.linear.x = vx
        cmd.linear.y = vy
        cmd.angular.z = w
        self.cmd_pub.publish(cmd)
        
        # Update robot velocity untuk tracking
        self.robot_vel = [vx, vy, w]

    # ========== DWA CONTROL FUNCTIONS (IMPROVED) ==========
    def dwa_control_loop(self):
        """Control loop untuk DWA mode dengan kontrol kecepatan ETA"""
        if not self.laser_data:
            self.get_logger().warn("⚠️ No laser data for DWA")
            return
        
        if not self.final_goal_pose:
            self.get_logger().warn("⚠️ No goal pose for DWA")
            self.stop_robot()
            return
        
        # CEK JIKA WAKTU ETA SUDAH HABIS - STOP!
        if self.time_remaining <= 0 and self.mission_stopped:
            self.stop_robot()
            return
        
        # Get current robot pose
        if not self.follower_pose:
            return
        
        robot_x = self.follower_pose.position.x
        robot_y = self.follower_pose.position.y
        robot_theta = self.yaw_from_q(self.follower_pose.orientation)
        
        # Cek stuck condition
        self.check_stuck_condition(robot_x, robot_y)
        
        # Update path tracking
        self.update_tracking_position()
        
        # Get target point
        self.get_next_target()
        
        # Check if position goal is reached
        goal_x, goal_y, goal_theta = self.final_goal_pose
        dist_to_goal = math.hypot(goal_x - robot_x, goal_y - robot_y)
        
        if dist_to_goal < self.dwa_goal_tolerance and not self.position_reached:
            self.position_reached = True
            self.get_logger().info("📍 DWA: POSITION GOAL REACHED!")
            self.dwa_state = self.DWA_STATE_FINAL_ALIGNING
        
        # Determine DWA state dengan perbaikan
        self.determine_dwa_state_improved(dist_to_goal)
        
        # Log state transitions
        self.log_dwa_state_transition()
        
        # Execute control berdasarkan state
        if self.dwa_state == self.DWA_STATE_ALIGNING:
            vx, vy, w = self.dwa_heading_alignment_control()
        elif self.dwa_state == self.DWA_STATE_TRACKING:
            vx, vy, w = self.dwa_tracking_control_improved()
        elif self.dwa_state == self.DWA_STATE_APPROACHING:
            vx, vy, w = self.dwa_approach_control()
        elif self.dwa_state == self.DWA_STATE_FINAL_ALIGNING:
            vx, vy, w = self.dwa_final_alignment_control()
        else:  # STATE_IDLE
            self.stop_robot()
            return
        
        # Apply command dengan kontrol kecepatan ETA
        self.publish_command_with_eta(vx, vy, w)
        
        # Publish local plan untuk visualization
        self.publish_local_plan(self.dwa_predict_trajectory(vx, vy, w))

    def publish_command_with_eta(self, vx, vy, w):
        """Publish velocity command dengan kontrol kecepatan ETA (sama dengan leader)"""
        # ✨ TAMBAHKAN FAKTOR PENGALI SAMA DENGAN LEADER ✨
        vx *= self.scaling_factor  # Sama dengan leader (1.2)
        vy *= self.scaling_factor  # Sama dengan leader (1.2)
        
        # Pastikan tidak melebihi batas maksimum setelah scaling
        vx = max(-self.dwa_max_vel_x, min(self.dwa_max_vel_x, vx))
        vy = max(-self.dwa_max_vel_y, min(self.dwa_max_vel_y, vy))
        w = max(-self.dwa_max_rot_vel, min(self.dwa_max_rot_vel, w))
        
        twist = Twist()
        twist.linear.x = float(vx)
        twist.linear.y = float(vy)
        twist.angular.z = float(w)
        self.cmd_pub.publish(twist)
        
        self.robot_vel = [vx, vy, w]
        self.commands_sent += 1

    def determine_dwa_state_improved(self, dist_to_goal):
        """Determine current DWA behavior state dengan perbaikan"""
        if not self.follower_pose:
            return
        
        robot_theta = self.yaw_from_q(self.follower_pose.orientation)
        
        if not self.target_point:
            return
        
        # Calculate heading error
        heading_error = 0.0
        if self.target_heading is not None:
            heading_error = abs(self.normalize_angle(self.target_heading - robot_theta))
        
        # State machine logic dengan hysteresis
        if self.position_reached:
            self.dwa_state = self.DWA_STATE_FINAL_ALIGNING
        elif dist_to_goal < self.dwa_goal_tolerance * 1.2:  # Threshold lebih rendah untuk approach
            self.dwa_state = self.DWA_STATE_APPROACHING
        elif heading_error > self.dwa_heading_alignment_tolerance * 1.5:  # Threshold lebih tinggi untuk alignment
            self.dwa_state = self.DWA_STATE_ALIGNING
        else:
            self.dwa_state = self.DWA_STATE_TRACKING

    def log_dwa_state_transition(self):
        """Log state transitions untuk debugging"""
        state_names = {
            self.DWA_STATE_ALIGNING: "ALIGNING",
            self.DWA_STATE_TRACKING: "TRACKING",
            self.DWA_STATE_APPROACHING: "APPROACHING",
            self.DWA_STATE_FINAL_ALIGNING: "FINAL_ALIGNING",
            self.DWA_STATE_IDLE: "IDLE"
        }
        
        current_state_name = state_names.get(self.dwa_state, "UNKNOWN")
        
        if current_state_name not in self.dwa_state_logged:
            self.get_logger().info(f"🔀 DWA State: {current_state_name}")
            self.dwa_state_logged[current_state_name] = True
        elif time.time() - self.last_log_time > 2.0:  # Log setiap 2 detik
            self.get_logger().debug(f"🔀 DWA State: {current_state_name}")
            self.last_log_time = time.time()

    def check_stuck_condition(self, robot_x, robot_y):
        """Cek apakah robot stuck dalam waktu tertentu"""
        current_time = time.time()
        
        if self.last_robot_position is None:
            self.last_robot_position = (robot_x, robot_y, current_time)
            return
        
        last_x, last_y, last_time = self.last_robot_position
        dist_moved = math.hypot(robot_x - last_x, robot_y - last_y)
        
        if current_time - last_time > self.stuck_check_interval:
            if dist_moved < 0.05:  # Hampir tidak bergerak
                self.stuck_counter += 1
                self.get_logger().warn(f"⚠️ DWA: Possible stuck #{self.stuck_counter} (moved {dist_moved:.3f}m in {self.stuck_check_interval:.1f}s)")
                
                if self.stuck_counter >= self.max_stuck_count:
                    self.handle_stuck_recovery()
            else:
                self.stuck_counter = max(0, self.stuck_counter - 1)
            
            self.last_robot_position = (robot_x, robot_y, current_time)

    def handle_stuck_recovery(self):
        """Handle recovery dari stuck condition"""
        self.get_logger().warn("🔄 DWA: Executing stuck recovery maneuver")
        
        # Reset stuck counter
        self.stuck_counter = 0
        
        # Execute recovery maneuver
        recovery_cmd = Twist()
        recovery_cmd.linear.x = -0.1  # Mundur sedikit
        recovery_cmd.angular.z = 0.3  # Putar sedikit
        
        for _ in range(5):  # Eksekusi selama 0.25 detik
            self.cmd_pub.publish(recovery_cmd)
            time.sleep(0.05)
        
        self.get_logger().info("✅ DWA: Recovery maneuver completed")

    def dwa_heading_alignment_control(self):
        """Pure rotation to align with target heading"""
        if not self.follower_pose:
            return 0.0, 0.0, 0.0
        
        robot_theta = self.yaw_from_q(self.follower_pose.orientation)
        
        if self.target_heading is None:
            return 0.0, 0.0, 0.0
        
        heading_error = self.normalize_angle(self.target_heading - robot_theta)
        
        # P-control for rotation dengan deadzone
        kp = 1.5
        w = kp * heading_error
        
        # Deadzone untuk heading error kecil
        if abs(heading_error) < 0.05:
            w = 0.0
        
        w = max(-self.dwa_max_rot_vel, min(self.dwa_max_rot_vel, w))
        
        # Damping near alignment
        if abs(heading_error) < 0.3:
            w *= 0.7
        
        # Tambahkan sedikit gerakan forward untuk membantu alignment
        vx = 0.0
        if abs(heading_error) < 0.5:  # Jika sudah cukup align
            vx = 0.05  # Gerakan forward kecil
        
        return vx, 0.0, w

    def dwa_tracking_control_improved(self):
        """Improved DWA for path tracking dengan obstacle avoidance yang lebih baik"""
        if not self.follower_pose or not self.target_point:
            return 0.0, 0.0, 0.0
        
        # CEK: Hindari pembagian dengan 0
        if self.dwa_max_vel_x < 0.001:
            return 0.0, 0.0, 0.0
        
        robot_x = self.follower_pose.position.x
        robot_y = self.follower_pose.position.y
        robot_theta = self.yaw_from_q(self.follower_pose.orientation)
        target_x, target_y = self.target_point
        
        # Current velocities
        current_vx, current_vy, current_w = self.robot_vel
        
        # Dynamic window - menggunakan limit dari ETA
        vx_min = max(-self.dwa_max_vel_x, current_vx - 0.15)
        vx_max = min(self.dwa_max_vel_x, current_vx + 0.15)
        vy_min = max(-self.dwa_max_vel_y, current_vy - 0.08)
        vy_max = min(self.dwa_max_vel_y, current_vy + 0.08)
        w_min = max(-self.dwa_max_rot_vel, current_w - 0.3)
        w_max = min(self.dwa_max_rot_vel, current_w + 0.3)
        
        # Generate samples
        vx_samples = np.linspace(vx_min, vx_max, self.dwa_vx_samples)
        vy_samples = np.linspace(vy_min, vy_max, self.dwa_vy_samples)
        w_samples = np.linspace(w_min, w_max, self.dwa_w_samples)
        
        best_score = -float('inf')
        best_vx, best_vy, best_w = 0.0, 0.0, 0.0
        
        # Calculate desired direction
        desired_dir = math.atan2(target_y - robot_y, target_x - robot_x)
        heading_error = self.normalize_angle(desired_dir - robot_theta)
        
        for vx in vx_samples:
            for vy in vy_samples:
                # Prefer forward motion
                if vx < 0 and abs(heading_error) < 1.0:
                    continue
                
                # Limit sideways ketika heading sudah cukup baik
                if abs(heading_error) < self.dwa_min_heading_error_for_sideways:
                    if abs(vy) > 0.05:
                        continue
                
                for w in w_samples:
                    # Skip jika rotating terlalu cepat sementara gerakan linear kecil
                    if (abs(vx) + abs(vy)) < 0.05 and abs(w) > 0.3:
                        continue
                    
                    # Predict trajectory
                    pred_x, pred_y, pred_theta = self.dwa_predict_pose(vx, vy, w)
                    
                    # Check collision dengan threshold yang lebih ketat
                    if self.check_collision_improved(pred_x, pred_y, vx, vy):
                        continue
                    
                    # Score this motion
                    score = self.calculate_dwa_score_improved(
                        pred_x, pred_y, pred_theta,
                        target_x, target_y,
                        vx, vy, w,
                        robot_x, robot_y, robot_theta,
                        heading_error
                    )
                    
                    if score > best_score:
                        best_score = score
                        best_vx = vx
                        best_vy = vy
                        best_w = w
        
        # Jika tidak ada trajectory yang valid, berhenti
        if best_score == -float('inf'):
            self.get_logger().warn("⚠️ DWA: No valid trajectory found!")
            return 0.0, 0.0, 0.0
        
        return best_vx, best_vy, best_w

    def check_collision_improved(self, x, y, vx, vy):
        """Improved collision check untuk DWA"""
        if not self.laser_data or not self.follower_pose:
            return False
        
        robot_x = self.follower_pose.position.x
        robot_y = self.follower_pose.position.y
        robot_theta = self.yaw_from_q(self.follower_pose.orientation)
        
        dx = x - robot_x
        dy = y - robot_y
        
        cos_t = math.cos(-robot_theta)
        sin_t = math.sin(-robot_theta)
        
        point_x = dx * cos_t - dy * sin_t
        point_y = dx * sin_t + dy * cos_t
        
        point_dist = math.hypot(point_x, point_y)
        point_angle = math.atan2(point_y, point_x)
        
        # Periksa collision di area yang lebih luas
        angle_range = 2.0  # radian (±114°)
        if abs(point_angle) > angle_range:
            return False
        
        # Safety distance berdasarkan kecepatan
        safety_distance = 0.2 + 0.5 * (abs(vx) + abs(vy))
        
        beam_idx = int((point_angle - self.laser_data.angle_min) /
                      self.laser_data.angle_increment)
        
        if 0 <= beam_idx < len(self.laser_data.ranges):
            laser_dist = self.laser_data.ranges[beam_idx]
            
            if (self.laser_data.range_min < laser_dist < self.laser_data.range_max and
                laser_dist < point_dist + safety_distance):
                return True
        
        return False

    def calculate_dwa_score_improved(self, pred_x, pred_y, pred_theta,
                                   target_x, target_y,
                                   vx, vy, w,
                                   robot_x, robot_y, robot_theta,
                                   heading_error):
        """Improved scoring untuk DWA path following"""
        
        # 1. Distance to target point (prioritas tinggi)
        dist_to_target = math.hypot(target_x - pred_x, target_y - pred_y)
        goal_score = -3.5 * dist_to_target
        
        # 2. Forward motion bias
        if self.dwa_max_vel_x > 0.001:
            forward_score = 2.5 * max(0, vx) / self.dwa_max_vel_x
        else:
            forward_score = 0.0
        
        # 3. Heading alignment
        if self.target_heading is not None:
            target_heading_error = abs(self.normalize_angle(self.target_heading - pred_theta))
            heading_score = -2.0 * target_heading_error
        else:
            heading_score = 0.0
        
        # 4. Progress along path
        if self.final_goal_pose:
            final_x, final_y, _ = self.final_goal_pose
            current_dist = math.hypot(final_x - robot_x, final_y - robot_y)
            pred_dist = math.hypot(final_x - pred_x, final_y - pred_y)
            progress_score = 2.0 * max(0, current_dist - pred_dist)
        else:
            progress_score = 0.0
        
        # 5. Sideways penalty (diperkecil jika perlu untuk maneuver)
        if self.dwa_max_vel_y > 0.001:
            sideways_penalty = 0.5 * abs(vy) / self.dwa_max_vel_y
        else:
            sideways_penalty = 0.0
        
        # 6. Velocity direction vs target direction
        vel_angle = math.atan2(vy, vx) if abs(vx) > 0.01 or abs(vy) > 0.01 else 0.0
        target_angle_rel = self.normalize_angle(heading_error - vel_angle)
        direction_score = -0.8 * abs(target_angle_rel)
        
        # 7. Smoothness (prioritas lebih rendah)
        smooth_penalty = 0.05 * abs(vx - self.robot_vel[0]) + \
                        0.1 * abs(vy - self.robot_vel[1]) + \
                        0.05 * abs(w - self.robot_vel[2])
        
        # 8. Obstacle clearance bonus
        obstacle_clearance = self.get_obstacle_clearance(pred_x, pred_y)
        clearance_bonus = 0.3 * obstacle_clearance
        
        total_score = (goal_score + forward_score + heading_score + 
                      progress_score + direction_score + clearance_bonus - 
                      sideways_penalty - smooth_penalty)
        
        return total_score

    def get_obstacle_clearance(self, x, y):
        """Get clearance distance dari obstacle terdekat"""
        if not self.laser_data or not self.follower_pose:
            return 0.0
        
        robot_x = self.follower_pose.position.x
        robot_y = self.follower_pose.position.y
        robot_theta = self.yaw_from_q(self.follower_pose.orientation)
        
        dx = x - robot_x
        dy = y - robot_y
        
        cos_t = math.cos(-robot_theta)
        sin_t = math.sin(-robot_theta)
        
        point_x = dx * cos_t - dy * sin_t
        point_y = dx * sin_t + dy * cos_t
        
        point_angle = math.atan2(point_y, point_x)
        
        min_clearance = float('inf')
        
        # Check beberapa beam di sekitar predicted point
        angle_range = 0.5  # ±28.6°
        start_angle = point_angle - angle_range
        end_angle = point_angle + angle_range
        
        start_idx = int((start_angle - self.laser_data.angle_min) /
                       self.laser_data.angle_increment)
        end_idx = int((end_angle - self.laser_data.angle_min) /
                     self.laser_data.angle_increment)
        
        start_idx = max(0, start_idx)
        end_idx = min(len(self.laser_data.ranges) - 1, end_idx)
        
        for idx in range(start_idx, end_idx + 1):
            if 0 <= idx < len(self.laser_data.ranges):
                dist = self.laser_data.ranges[idx]
                if self.laser_data.range_min < dist < self.laser_data.range_max:
                    min_clearance = min(min_clearance, dist)
        
        if min_clearance == float('inf'):
            return 1.0  # Tidak ada obstacle terdeteksi
        
        return min(min_clearance, 1.0)

    def dwa_approach_control(self):
        """Final approach to position goal dengan kontrol yang lebih smooth"""
        if not self.follower_pose or not self.target_point:
            return 0.0, 0.0, 0.0
        
        robot_x = self.follower_pose.position.x
        robot_y = self.follower_pose.position.y
        robot_theta = self.yaw_from_q(self.follower_pose.orientation)
        target_x, target_y = self.target_point
        
        # Calculate direction to target
        dx = target_x - robot_x
        dy = target_y - robot_y
        dist = math.hypot(dx, dy)
        
        if dist < 0.05:
            return 0.0, 0.0, 0.0
        
        # Convert to robot frame
        cos_t = math.cos(robot_theta)
        sin_t = math.sin(robot_theta)
        
        vx_local = dx * cos_t + dy * sin_t
        vy_local = -dx * sin_t + dy * cos_t
        
        # P-control dengan gain yang menurun saat mendekati target
        kp = 0.8 * min(1.0, dist / 0.5)  # Gain berkurang saat mendekati
        vx = kp * vx_local
        vy = kp * vy_local
        
        # Limit speed dengan kurva smooth
        max_approach_speed = 0.15 * min(1.0, dist / 0.3)
        speed = math.hypot(vx, vy)
        if speed > max_approach_speed:
            vx *= max_approach_speed / speed
            vy *= max_approach_speed / speed
        
        # Minimal rotation untuk menjaga heading
        w = 0.0
        
        return vx, vy, w

    def dwa_final_alignment_control(self):
        """Align to final goal orientation dengan kontrol smooth"""
        if not self.follower_pose or not self.final_goal_pose:
            return 0.0, 0.0, 0.0
        
        robot_x = self.follower_pose.position.x
        robot_y = self.follower_pose.position.y
        robot_theta = self.yaw_from_q(self.follower_pose.orientation)
        goal_x, goal_y, goal_theta = self.final_goal_pose
        
        # Check if we're still at the goal position
        pos_error = math.hypot(goal_x - robot_x, goal_y - robot_y)
        
        if pos_error > self.dwa_goal_tolerance * 2.0:
            # Moved away from goal
            self.position_reached = False
            self.dwa_state = self.DWA_STATE_APPROACHING
            return self.dwa_approach_control()
        
        # Calculate heading error to final orientation
        heading_error = self.normalize_angle(goal_theta - robot_theta)
        
        # Check if already aligned
        if abs(heading_error) < self.dwa_heading_alignment_tolerance:
            self.get_logger().info("🎉 DWA: GOAL COMPLETELY REACHED!")
            self.dwa_state = self.DWA_STATE_IDLE
            return 0.0, 0.0, 0.0
        
        # P-control for rotation dengan deadzone
        kp = 2.5
        w = kp * heading_error
        
        # Deadzone untuk heading error sangat kecil
        if abs(heading_error) < 0.02:
            w = 0.0
        
        w = max(-self.dwa_max_rot_vel * 0.6, min(self.dwa_max_rot_vel * 0.6, w))
        
        # Smooth braking near alignment
        if abs(heading_error) < 0.3:
            w *= abs(heading_error) / 0.3
        
        # Small position correction if needed
        vx, vy = 0.0, 0.0
        if pos_error > self.dwa_goal_tolerance:
            dx = goal_x - robot_x
            dy = goal_y - robot_y
            
            cos_t = math.cos(robot_theta)
            sin_t = math.sin(robot_theta)
            
            vx = 0.02 * (dx * cos_t + dy * sin_t)
            vy = 0.02 * (-dx * sin_t + dy * cos_t)
            
            corr_speed = math.hypot(vx, vy)
            if corr_speed > 0.02:
                vx *= 0.02 / corr_speed
                vy *= 0.02 / corr_speed
        
        return vx, vy, w

    def dwa_predict_pose(self, vx, vy, w):
        """Predict pose after prediction_time"""
        if not self.follower_pose:
            return 0.0, 0.0, 0.0
        
        x = self.follower_pose.position.x
        y = self.follower_pose.position.y
        theta = self.yaw_from_q(self.follower_pose.orientation)
        
        steps = int(self.dwa_prediction_time / self.dwa_dt)
        for _ in range(steps):
            theta += w * self.dwa_dt
            dx_world = vx * math.cos(theta) - vy * math.sin(theta)
            dy_world = vx * math.sin(theta) + vy * math.cos(theta)
            x += dx_world * self.dwa_dt
            y += dy_world * self.dwa_dt
        
        return x, y, theta

    def dwa_predict_trajectory(self, vx, vy, w):
        """Predict trajectory for visualization"""
        trajectory = []
        
        if not self.follower_pose:
            return trajectory
        
        x = self.follower_pose.position.x
        y = self.follower_pose.position.y
        theta = self.yaw_from_q(self.follower_pose.orientation)
        
        steps = int(self.dwa_prediction_time / self.dwa_dt)
        viz_step = max(1, steps // 20)
        
        for step in range(steps):
            theta += w * self.dwa_dt
            dx_world = vx * math.cos(theta) - vy * math.sin(theta)
            dy_world = vx * math.sin(theta) + vy * math.cos(theta)
            x += dx_world * self.dwa_dt
            y += dy_world * self.dwa_dt
            
            if step % viz_step == 0:
                trajectory.append([x, y, theta])
        
        if steps > 0 and (steps - 1) % viz_step != 0:
            trajectory.append([x, y, theta])
        
        return trajectory

    def get_next_target(self):
        """Get next target point for DWA dengan lookahead yang adaptif"""
        if not self.global_path or len(self.global_path) < 2:
            # Jika tidak ada path, target adalah final goal
            if self.final_goal_pose:
                self.target_point = [self.final_goal_pose[0], self.final_goal_pose[1]]
                self.target_heading = self.final_goal_pose[2]
            return
        
        # Dapatkan posisi robot
        robot_x, robot_y = 0.0, 0.0
        if self.follower_pose:
            robot_x = self.follower_pose.position.x
            robot_y = self.follower_pose.position.y
        
        # Lookahead distance adaptif berdasarkan kecepatan
        adaptive_lookahead = self.dwa_lookahead_distance * (1.0 + 0.5 * self.current_speed)
        lookahead_remaining = min(adaptive_lookahead, 1.0)  # Maksimum 1.0m
        
        # Start from current segment
        start_idx = self.current_path_index
        start_progress = self.progress_along_segment
        
        # Jika sudah di akhir path
        if start_idx >= len(self.global_path) - 1:
            self.target_point = [self.global_path[-1][0], self.global_path[-1][1]]
            self.target_heading = self.global_path[-1][2]
            return
        
        # Hitung target point dengan lookahead
        x1, y1, _ = self.global_path[start_idx]
        x2, y2, _ = self.global_path[start_idx + 1]
        
        segment_len = math.hypot(x2 - x1, y2 - y1)
        
        if segment_len == 0:
            # Skip zero-length segment
            self.current_path_index += 1
            self.progress_along_segment = 0.0
            self.get_next_target()
            return
        
        # Posisi saat ini pada segmen
        current_x = x1 + start_progress * (x2 - x1)
        current_y = y1 + start_progress * (y2 - y1)
        
        # Sisa jarak pada segmen saat ini
        remaining_on_segment = (1 - start_progress) * segment_len
        
        if remaining_on_segment >= lookahead_remaining:
            # Target masih dalam segmen saat ini
            ratio = lookahead_remaining / segment_len
            target_x = current_x + ratio * (x2 - x1)
            target_y = current_y + ratio * (y2 - y1)
            self.target_point = [target_x, target_y]
            self.target_heading = math.atan2(y2 - y1, x2 - x1)
        else:
            # Perlu melihat segmen berikutnya
            lookahead_remaining -= remaining_on_segment
            current_idx = start_idx + 1
            
            # Cari segmen yang cukup panjang untuk lookahead
            while current_idx < len(self.global_path) - 1 and lookahead_remaining > 0:
                x1, y1, _ = self.global_path[current_idx]
                x2, y2, _ = self.global_path[current_idx + 1]
                
                segment_len = math.hypot(x2 - x1, y2 - y1)
                
                if segment_len == 0:
                    current_idx += 1
                    continue
                
                if segment_len >= lookahead_remaining:
                    # Target ditemukan pada segmen ini
                    ratio = lookahead_remaining / segment_len
                    target_x = x1 + ratio * (x2 - x1)
                    target_y = y1 + ratio * (y2 - y1)
                    self.target_point = [target_x, target_y]
                    self.target_heading = math.atan2(y2 - y1, x2 - x1)
                    return
                else:
                    lookahead_remaining -= segment_len
                    current_idx += 1
            
            # Jika mencapai akhir path, target adalah goal
            self.target_point = [self.global_path[-1][0], self.global_path[-1][1]]
            self.target_heading = self.global_path[-1][2]

    def update_tracking_position(self):
        """Update path tracking position untuk DWA dengan path yang valid"""
        if not self.follower_pose or not self.global_path or len(self.global_path) < 2:
            return
        
        robot_x = self.follower_pose.position.x
        robot_y = self.follower_pose.position.y
        
        # Cari segmen terdekat di sekitar current index
        search_start = max(0, self.current_path_index - 2)
        search_end = min(len(self.global_path) - 1, self.current_path_index + 4)
        
        min_dist = float('inf')
        best_idx = self.current_path_index
        best_t = self.progress_along_segment
        
        for i in range(search_start, search_end):
            if i >= len(self.global_path) - 1:
                break
            
            x1, y1, _ = self.global_path[i]
            x2, y2, _ = self.global_path[i + 1]
            
            closest_point, dist, t = self.closest_point_on_segment(robot_x, robot_y, x1, y1, x2, y2)
            
            if dist < min_dist:
                min_dist = dist
                best_idx = i
                best_t = t
        
        # Update jika dalam tolerance yang lebih longgar
        if min_dist < self.dwa_path_tracking_tolerance * 1.5:
            self.current_path_index = best_idx
            self.progress_along_segment = best_t
            
            # Maju ke segmen berikutnya jika sudah hampir selesai
            if best_t > 0.95 and self.current_path_index < len(self.global_path) - 2:
                self.current_path_index += 1
                self.progress_along_segment = 0.0
        
        # Debug log untuk tracking
        if min_dist > 1.5:  # Jika terlalu jauh dari path
            self.get_logger().warn(f"⚠️ DWA: Robot far from path! Distance: {min_dist:.2f}m")

    def check_collision(self, x, y):
        """Collision check untuk DWA"""
        if not self.laser_data or not self.follower_pose:
            return False
        
        robot_x = self.follower_pose.position.x
        robot_y = self.follower_pose.position.y
        robot_theta = self.yaw_from_q(self.follower_pose.orientation)
        
        dx = x - robot_x
        dy = y - robot_y
        
        cos_t = math.cos(-robot_theta)
        sin_t = math.sin(-robot_theta)
        
        point_x = dx * cos_t - dy * sin_t
        point_y = dx * sin_t + dy * cos_t
        
        point_dist = math.hypot(point_x, point_y)
        point_angle = math.atan2(point_y, point_x)
        
        if abs(point_angle) > 1.57:
            return False
        
        beam_idx = int((point_angle - self.laser_data.angle_min) /
                      self.laser_data.angle_increment)
        
        if 0 <= beam_idx < len(self.laser_data.ranges):
            laser_dist = self.laser_data.ranges[beam_idx]
            
            if (self.laser_data.range_min < laser_dist < self.laser_data.range_max and
                laser_dist < point_dist - 0.1):
                return True
        
        return False

    def closest_point_on_segment(self, px, py, x1, y1, x2, y2):
        """Find closest point on line segment"""
        dx = x2 - x1
        dy = y2 - y1
        
        if dx == 0 and dy == 0:
            return [x1, y1], math.hypot(px - x1, py - y1), 0.0
        
        t = ((px - x1) * dx + (py - y1) * dy) / (dx*dx + dy*dy)
        t = max(0, min(1, t))
        
        closest_x = x1 + t * dx
        closest_y = y1 + t * dy
        
        dist = math.hypot(px - closest_x, py - closest_y)
        
        return [closest_x, closest_y], dist, t

    def publish_local_plan(self, trajectory):
        """Publish local plan untuk visualization"""
        if not trajectory:
            path_msg = Path()
            path_msg.header.stamp = self.get_clock().now().to_msg()
            path_msg.header.frame_id = 'map'
            self.local_plan_pub.publish(path_msg)
            return
        
        path_msg = Path()
        path_msg.header.stamp = self.get_clock().now().to_msg()
        path_msg.header.frame_id = 'map'
        
        for point in trajectory:
            x, y, theta = point
            
            pose_stamped = PoseStamped()
            pose_stamped.header.stamp = path_msg.header.stamp
            pose_stamped.header.frame_id = path_msg.header.frame_id
            
            pose_stamped.pose.position.x = float(x)
            pose_stamped.pose.position.y = float(y)
            pose_stamped.pose.position.z = 0.0
            
            qz = math.sin(theta / 2.0)
            qw = math.cos(theta / 2.0)
            
            pose_stamped.pose.orientation.x = 0.0
            pose_stamped.pose.orientation.y = 0.0
            pose_stamped.pose.orientation.z = qz
            pose_stamped.pose.orientation.w = qw
            
            path_msg.poses.append(pose_stamped)
        
        self.local_plan_pub.publish(path_msg)

    def log_status(self):
        """Log status system"""
        # Mode information
        mode_str = "FOLLOWER" if self.current_mode == self.MODE_FOLLOWER else "DWA"
        
        # ETA information (sinkron dengan leader)
        eta_str = f"R:{self.time_remaining:.1f}s/E:{self.time_elapsed:.1f}s/T:{self.target_time:.1f}s"
        
        if self.time_remaining <= 0:
            eta_str = f"⏰ TIME'S UP! (E:{self.time_elapsed:.1f}s/T:{self.target_time:.1f}s)"
        elif self.time_remaining <= self.stop_time_threshold:
            eta_str = f"🛑 R:{self.time_remaining:.1f}s/E:{self.time_elapsed:.1f}s/T:{self.target_time:.1f}s"
        
        # Transition status
        trans_str = ""
        if self.transition_triggered:
            if self.transition_complete:
                trans_str = "✅ COMPLETE"
            else:
                trans_str = "🔄 IN PROGRESS"
        
        # Goal status
        goal_status = []
        if self.goal_received:
            goal_status.append("📋 Received")
        if self.goal_published:
            goal_status.append("📤 Published")
        
        goal_str = " | ".join(goal_status) if goal_status else "❌ No goal"
        
        # Robot pose
        pose_str = "N/A"
        if self.follower_pose:
            x = self.follower_pose.position.x
            y = self.follower_pose.position.y
            theta = self.yaw_from_q(self.follower_pose.orientation)
            pose_str = f"({x:.2f}, {y:.2f}, {math.degrees(theta):.1f}°)"
        
        # Goal information
        goal_pose_str = "N/A"
        if self.transformed_goal_pose:
            x = self.transformed_goal_pose["x"]
            y = self.transformed_goal_pose["y"]
            yaw = self.transformed_goal_pose["yaw"]
            goal_pose_str = f"({x:.2f}, {y:.2f}, {math.degrees(yaw):.1f}°)"
        
        # Compose log message
        log_lines = [
            f"🤖 MODE: {mode_str}",
            f"⏰ ETA: {eta_str}",
            f"🔄 TRANS: {trans_str}" if trans_str else "",
            f"🎯 GOAL: {goal_str}",
            f"📍 POSE: {pose_str}",
            f"🎯 TARGET: {goal_pose_str}"
        ]
        
        # Filter out empty lines
        log_lines = [line for line in log_lines if line]
        
        self.get_logger().info(" | ".join(log_lines))
        
        # Log tambahan untuk DWA mode dengan info ETA
        if self.current_mode == self.MODE_DWA:
            if self.follower_pose and self.final_goal_pose:
                robot_x = self.follower_pose.position.x
                robot_y = self.follower_pose.position.y
                goal_x, goal_y, _ = self.final_goal_pose
                dist = math.hypot(goal_x - robot_x, goal_y - robot_y)
                
                # DWA state information
                state_names = {
                    self.DWA_STATE_ALIGNING: "ALIGNING",
                    self.DWA_STATE_TRACKING: "TRACKING",
                    self.DWA_STATE_APPROACHING: "APPROACHING",
                    self.DWA_STATE_FINAL_ALIGNING: "FINAL_ALIGNING",
                    self.DWA_STATE_IDLE: "IDLE"
                }
                
                self.get_logger().info(
                    f"📊 DWA: {state_names.get(self.dwa_state, 'UNKNOWN')} | "
                    f"Dist: {dist:.2f}m | "
                    f"Path: {self.remaining_path_length:.2f}m / {self.total_path_length:.2f}m | "
                    f"Speed: {self.current_speed:.2f}m/s (target: {self.needed_speed:.2f}m/s) | "
                    f"Vel: [{self.robot_vel[0]:.2f}, {self.robot_vel[1]:.2f}, {self.robot_vel[2]:.2f}] | "
                    f"Target idx: {self.current_path_index}/{len(self.global_path)-1 if self.global_path else 0}"
                )

    def publish_zero_vel(self):
        """Mengirim kecepatan nol"""
        cmd = Twist()
        cmd.linear.x = 0.0
        cmd.linear.y = 0.0
        cmd.angular.z = 0.0
        self.cmd_pub.publish(cmd)
        
        # Reset integral term saat berhenti
        self.errSum = np.zeros((3, 1))
        self.robot_vel = [0.0, 0.0, 0.0]

    def stop_robot(self):
        """Stop robot dan publish empty path"""
        self.publish_zero_vel()
        
        empty_path = Path()
        empty_path.header.stamp = self.get_clock().now().to_msg()
        empty_path.header.frame_id = 'map'
        self.local_plan_pub.publish(empty_path)

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
    node = Robot2DualModeController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()