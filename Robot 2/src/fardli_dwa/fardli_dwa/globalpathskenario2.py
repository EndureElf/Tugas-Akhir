#!/usr/bin/env python3
"""
Global Path Planner - Dengan Support Waypoint dari Master Coordinator
A* VERSION: Dijkstra upgraded to A* for faster computation
MODIFIED: Menerima goal_pose dan waypoint dari masterskenario2.py
MODIFIED: Planning melalui waypoint sebelum mencapai goal akhir
DITAMBAH: cmd_vel tracking untuk estimasi jarak real-time
DITAMBAH: A* Node Visualization untuk plotting
DITAMBAH: Waypoint tracking dan perhitungan panjang per waypoint

PERBAIKAN: Menerima BOTH waypoint dan goal dari topics terpisah
"""

import rclpy
from rclpy.node import Node
import numpy as np
import math
import heapq
import time
import csv
import os
from datetime import datetime
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped, Twist
from nav_msgs.msg import OccupancyGrid, Path
from std_msgs.msg import Header, Float32, Bool, Int32, String
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSReliabilityPolicy, QoSHistoryPolicy

class GlobalPathPlanner(Node):
    def __init__(self):
        super().__init__('global_path_planner')
        
        # ========== PARAMETERS ==========
        self.occupied_threshold = 60      
        self.free_threshold = 15          
        self.inflation_radius = 10.5        # cells untuk robot radius
        self.map_resolution = 0.05
        
        # Goal checking parameters
        self.position_threshold = 0.2      # 10 cm threshold untuk posisi
        self.orientation_threshold = 2  # ~5 derajat dalam radian
        
        # Waypoint parameters
        self.waypoint_reached_threshold = 1.0  # 20 cm threshold untuk mencapai waypoint
        
        # Costmap parameters
        self.costmap_scaling_factor = 3.0  # Gradient scaling
        self.lethal_cost = 254
        self.inscribed_cost = 253
        self.possibly_inscribed_cost = 128
        self.free_cost = 0
        
        # A* parameters
        self.max_astar_nodes = 50000    # Max nodes untuk explore
        self.astar_heuristic_weight = 1.0  # Weight untuk heuristic A*
        
        # Path smoothing parameters
        self.smoothing_max_angle = math.radians(0)  # Sudut maksimal sebelum smoothing
        
        # ========== A* VISUALIZATION PARAMETERS ==========
        self.astar_visualization_enabled = True  # Enable/disable visualization data
        self.astar_nodes_data = []  # Untuk menyimpan data semua node A*
        self.astar_open_set_data = []  # Untuk menyimpan open set nodes
        self.astar_closed_set_data = []  # Untuk menyimpan closed set nodes
        self.astar_final_path = []  # Untuk menyimpan final path nodes
        self.astar_export_counter = 0  # Counter untuk multiple exports
        
        # File paths
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        self.csv_base_path = f"/home/sunrise/astar_data_{timestamp}"
        os.makedirs(self.csv_base_path, exist_ok=True)
        
        # ========== ENHANCED: REAL-TIME LENGTH UPDATE PARAMETERS ==========
        self.fast_update_hz = 10.0          # 10 Hz updates
        self.length_update_interval = 1.0 / self.fast_update_hz
        
        # ========== CMD_VEL BASED TRACKING ==========
        self.cmd_vel_linear = 0.0
        self.last_cmd_vel_update_time = None
        self.traveled_from_cmd_vel = 0.0
        
        # ========== STATE ==========
        self.robot_pose = None  # x, y, theta
        self.goal_pose = None   # [x, y, theta] - FINAL GOAL dari masterskenario2 (robot4)
        self.waypoint_pose = None  # [x, y, theta] - WAYPOINT dari masterskenario2 (robot3)
        self.map_data = None
        self.map_width = 0
        self.map_height = 0
        self.map_origin = [0.0, 0.0]
        
        # Grids
        self.original_grid = None      # Grid asli dari map
        self.binary_grid = None        # Grid binary (0=free, 1=occupied)
        self.costmap = None           # Global costmap dengan gradient
        self.planning_grid = None     # Grid untuk planning (dengan inflation)
        
        # Status flags
        self.map_received = False
        self.amcl_received = False
        self.goal_received = False
        self.waypoint_received = False  # Flag untuk waypoint dari masterskenario2
        self.path_valid = False
        self.costmap_created = False
        self.goal_reached = False      # Flag apakah goal sudah tercapai
        self.waypoint_reached = False  # Flag apakah waypoint sudah tercapai
        self.cmd_vel_received = False  # Flag cmd_vel received
        
        # MODIFIED: Path data dengan multiple waypoints dari masterskenario2
        self.global_path = []          # Current path [x, y] (termasuk waypoint + goal)
        self.cached_path = []          # Cached path untuk fast length updates
        self.path_grid = []           # Path dalam grid coordinates
        self.path_length = 0.0        # Total panjang lintasan (start ke goal melalui waypoint)
        self.remaining_length = 0.0    # Remaining length (robot ke goal) - REAL-TIME
        self.traveled_length = 0.0     # Panjang yang sudah ditempuh
        
        # NEW: Multiple waypoint tracking dari masterskenario2
        self.current_waypoint_index = 0  # Indeks waypoint saat ini (0 = robot3 waypoint, 1 = robot4 goal)
        self.waypoint_list = []          # List semua waypoint [waypoint, goal]
        self.waypoint_distances = []     # Jarak kumulatif ke setiap waypoint dari start
        self.waypoint_remaining = []     # Sisa jarak dari robot ke setiap waypoint
        self.waypoint_status = []        # Status setiap waypoint (False=belum, True=sudah)
        self.active_waypoint = None      # Waypoint aktif saat ini [x, y]
        
        # Untuk length tracking
        self.last_robot_position = None  # Untuk hitung traveled distance
        self.last_path_publication_time = 0.0
        
        # Performance tracking
        self.costmap_creation_time = 0.0
        self.astar_explored_nodes = 0
        self.total_plans = 0
        self.astar_time = 0.0
        self.start_time = time.time()
        
        # ========== QoS PROFILES ==========
        map_qos = QoSProfile(
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL
        )
        
        amcl_qos = QoSProfile(
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10,
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL
        )
        
        # ========== SUBSCRIBERS ==========
        # 1. Map subscriber
        self.create_subscription(
            OccupancyGrid,
            '/map',
            self.map_callback,
            map_qos
        )
        
        # 2. AMCL pose subscriber
        self.create_subscription(
            PoseWithCovarianceStamped,
            '/robot2/amcl_pose',
            self.amcl_callback,
            amcl_qos
        )
        
        # **PERBAIKAN: Subscribe ke BOTH topics TERPISAH untuk waypoint dan goal**
        
        # 3. WAYPOINT dari master (robot3) - TOPIC BARU
        self.create_subscription(
            PoseStamped,
            '/robot2/goal_pose',
            self.master_waypoint_callback,
            10
        )
        
        # 6. Master command subscriber (tetap ada)
        self.create_subscription(
            String,
            '/robot2/master_command',
            self.master_command_callback,
            10
        )
        
        # 7. CMD_VEL subscriber (tetap ada)
        self.create_subscription(
            Twist,
            '/robot2/cmd_vel',
            self.cmd_vel_callback,
            10
        )
        
        # ========== PUBLISHERS ==========
        self.path_pub = self.create_publisher(Path, '/robot2/plan', 10)
        self.costmap_pub = self.create_publisher(OccupancyGrid, '/robot2/global_costmap/costmap', 10)
        self.path_length_pub = self.create_publisher(Float32, '/robot2/path_length', 10)
        self.remaining_length_pub = self.create_publisher(Float32, '/robot2/remaining_length', 10)
        self.traveled_length_pub = self.create_publisher(Float32, '/robot2/traveled_length', 10)
        self.goal_status_pub = self.create_publisher(Bool, '/robot2/goal_reached', 10)
        self.performance_pub = self.create_publisher(Float32, '/robot2/planning_time', 10)
        
        # ========== MODIFIED: WAYPOINT PUBLISHERS ==========
        self.waypoint_index_pub = self.create_publisher(Int32, '/robot2/current_waypoint', 10)
        self.waypoint_total_pub = self.create_publisher(Int32, '/robot2/total_waypoints', 10)
        self.active_waypoint_pub = self.create_publisher(PoseStamped, '/robot2/active_waypoint', 10)
        self.waypoint_remaining_pub = self.create_publisher(Float32, '/robot2/waypoint_remaining', 10)
        self.next_waypoint_remaining_pub = self.create_publisher(Float32, '/robot2/next_waypoint_remaining', 10)
        
        # NEW: Publisher untuk status waypoint
        self.waypoint_status_pub = self.create_publisher(String, '/robot2/waypoint_status', 10)
        
        # ========== TIMERS ==========
        # ENHANCED: Real-time length updates (10 Hz)
        self.create_timer(self.length_update_interval, self.fast_length_update_callback)
        self.last_pose_time = None
        self.last_pose_position = None
        self.pose_received_count = 0 
        # ORIGINAL TIMERS (ALL PRESERVED)
        self.create_timer(0.5, self.publish_path_callback)  # Publish path
        self.create_timer(0.5, self.publish_costmap_callback)  # Publish costmap
        self.create_timer(0.1, self.check_waypoint_reached_callback)  # High-frequency waypoint checking
        self.create_timer(3.0, self.attempt_path_computation)
        self.create_timer(2.0, self.status_report)
        
        # ========== NEW: WAYPOINT TRACKING TIMER ==========
        self.create_timer(0.1, self.update_waypoint_tracking_callback)  # Update waypoint tracking
        
        self.get_logger().info("=" * 70)
        self.get_logger().info("🚀 Global Path Planner - WITH MASTER COORDINATOR SUPPORT")
        self.get_logger().info("   Menerima BOTH waypoint dan goal dari masterskenario2.py")
        self.get_logger().info("   Waypoint topic: /robot3/goal_pose")
        self.get_logger().info("   Goal topic: /robot4/goal_pose")
        self.get_logger().info("   Planning melalui waypoint ke goal")
        self.get_logger().info("=" * 70)
        self.get_logger().info(f"A* Data akan disimpan di: {self.csv_base_path}")
        self.get_logger().info("Waiting for waypoint and goal from master coordinator...")
    
    # ========== NEW: MASTER COMMAND CALLBACK ==========
    def master_command_callback(self, msg):
        """Terima command dari master coordinator"""
        command = msg.data
        self.get_logger().info(f"Master command received: {command}")
        
        if command == "reset_mission":
            self.reset_mission_state()
        elif command == "compute_path":
            self.attempt_path_computation()
    
    # ========== NEW: SEPARATE CALLBACKS FOR WAYPOINT AND GOAL ==========
    def master_waypoint_callback(self, msg):
        """Process ALL pose dari follower - perlu bedakan waypoint vs goal"""
        # Extract position
        x = msg.pose.position.x
        y = msg.pose.position.y
        
        # Extract orientation
        q = msg.pose.orientation
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        yaw = math.atan2(siny_cosp, cosy_cosp)
        
        current_time = time.time()
        current_position = (x, y)
        
        # PERBAIKAN: Logika untuk membedakan waypoint dan goal
        is_goal = False
        
        # Logika 1: Jika ini adalah pose pertama yang diterima, anggap sebagai waypoint
        if self.last_pose_time is None:
            self.get_logger().info(f"📍 POSE PERTAMA diterima (diasumsikan WAYPOINT): ({x:.2f}, {y:.2f})")
            self.waypoint_pose = [x, y, yaw]
            self.waypoint_received = True
            is_goal = False
        
        # Logika 2: Jika posisi berbeda dengan yang terakhir, dan jarak > 1m, anggap sebagai goal
        elif self.last_pose_position:
            dx = x - self.last_pose_position[0]
            dy = y - self.last_pose_position[1]
            distance = math.sqrt(dx*dx + dy*dy)
            
            if distance > 1.0:  # Jika posisi berbeda lebih dari 1m
                self.get_logger().info(f"🎯 POSE KEDUA dengan posisi berbeda >1m (diasumsikan GOAL): ({x:.2f}, {y:.2f})")
                self.goal_pose = [x, y, yaw]
                self.goal_received = True
                is_goal = True
            else:
                # Jika posisi sama atau sangat dekat, abaikan (mungkin duplikat)
                self.get_logger().debug(f"⚠️ Pose duplikat atau sangat dekat, diabaikan")
                return
        
        # Logika 3: Tracking urutan (pose pertama = waypoint, pose kedua = goal)
        self.pose_received_count += 1
        
        if self.pose_received_count == 1:
            self.get_logger().info(f"📌 POSE #{self.pose_received_count}: WAYPOINT robot3")
            self.waypoint_pose = [x, y, yaw]
            self.waypoint_received = True
            is_goal = False
        elif self.pose_received_count == 2:
            self.get_logger().info(f"🎯 POSE #{self.pose_received_count}: GOAL robot4")
            self.goal_pose = [x, y, yaw]
            self.goal_received = True
            is_goal = True
        
        # Update tracking
        self.last_pose_time = current_time
        self.last_pose_position = current_position
        
        # Log dengan informasi jelas
        if is_goal:
            self.get_logger().info(f"🎯 GOAL dari follower (Robot4): ({x:.2f}, {y:.2f}), theta: {math.degrees(yaw):.1f}°")
        else:
            self.get_logger().info(f"📍 WAYPOINT dari follower (Robot3): ({x:.2f}, {y:.2f}), theta: {math.degrees(yaw):.1f}°")
        
        # Update waypoint list jika keduanya sudah diterima
        if self.waypoint_received and self.goal_received:
            self.update_waypoint_list()
        else:
            self.get_logger().info(f"⏳ Menunggu {'goal' if not self.goal_received else 'waypoint'}...")
    
    def master_goal_callback(self, msg):
        """Process GOAL pose dari masterskenario2.py (robot4 goal)"""
        # Extract position
        x = msg.pose.position.x
        y = msg.pose.position.y
        
        # Extract orientation
        q = msg.pose.orientation
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        yaw = math.atan2(siny_cosp, cosy_cosp)
        
        self.goal_pose = [x, y, yaw]
        self.goal_received = True
        
        self.get_logger().info(f"🎯 GOAL dari master (Robot4): ({x:.2f}, {y:.2f}), "
                              f"theta: {math.degrees(yaw):.1f}°")
        
        # Update waypoint list jika sudah ada waypoint
        if self.waypoint_received:
            self.update_waypoint_list()
    
    def robot2_goal_callback(self, msg):
        """Process active goal untuk robot2 (bisa waypoint atau goal)"""
        x = msg.pose.position.x
        y = msg.pose.position.y
        
        # Extract orientation
        q = msg.pose.orientation
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        yaw = math.atan2(siny_cosp, cosy_cosp)
        
        self.get_logger().info(f"🎯 Active goal untuk Robot2: ({x:.2f}, {y:.2f})")
        
        # Toleransi yang lebih besar untuk floating-point comparison
        tolerance = 0.15  # 15 cm
        
        # Cek apakah ini waypoint dari master
        if self.waypoint_pose:
            wp_x, wp_y, _ = self.waypoint_pose
            dist_to_wp = math.sqrt((x - wp_x)**2 + (y - wp_y)**2)
            if dist_to_wp <= tolerance:
                self.get_logger().info(f"   ✅ Ini adalah WAYPOINT dari Robot3")
                self.current_segment = "waypoint"
                return
        
        # Cek apakah ini goal dari master
        if self.goal_pose:
            goal_x, goal_y, _ = self.goal_pose
            dist_to_goal = math.sqrt((x - goal_x)**2 + (y - goal_y)**2)
            if dist_to_goal <= tolerance:
                self.get_logger().info(f"   ✅ Ini adalah FINAL GOAL dari Robot4")
                self.current_segment = "goal"
                return
        
        # Jika bukan keduanya
        self.get_logger().warn(f"   ⚠️ Goal tidak dikenal! Mungkin dari source lain")
        self.get_logger().warn(f"   Received: ({x:.3f}, {y:.3f})")
        if self.waypoint_pose:
            self.get_logger().warn(f"   Waypoint: ({self.waypoint_pose[0]:.3f}, {self.waypoint_pose[1]:.3f})")
        if self.goal_pose:
            self.get_logger().warn(f"   Goal: ({self.goal_pose[0]:.3f}, {self.goal_pose[1]:.3f})")
    
    def update_waypoint_list(self):
        """Update list waypoint dari masterskenario2"""
        self.waypoint_list = []
        
        # Tambahkan waypoint jika ada
        if self.waypoint_pose:
            self.waypoint_list.append(self.waypoint_pose)
            self.get_logger().info(f"Added waypoint to list: Robot3 waypoint")
        
        # Tambahkan goal akhir
        if self.goal_pose:
            self.waypoint_list.append(self.goal_pose)
            self.get_logger().info(f"Added goal to list: Robot4 goal")
        
        # Reset semua status waypoint saat menerima waypoint/goal baru
        self.reset_waypoint_tracking()
        
        # Publish status
        status_msg = String()
        status_msg.data = f"Waypoints updated: {len(self.waypoint_list)} waypoints"
        self.waypoint_status_pub.publish(status_msg)
        
        self.get_logger().info(f"✅ Updated waypoint list: {len(self.waypoint_list)} points")
        
        # Coba compute path jika semua data tersedia
        if self.map_received and self.amcl_received and len(self.waypoint_list) > 0:
            self.get_logger().info("All data available, computing path through waypoints...")
            self.attempt_path_computation()
    
    def reset_waypoint_tracking(self):
        """Reset semua tracking waypoint"""
        self.goal_reached = False
        self.waypoint_reached = False
        self.current_waypoint_index = 0
        self.waypoint_status = [False] * len(self.waypoint_list) if self.waypoint_list else []
        self.waypoint_distances = []
        self.waypoint_remaining = []
        self.active_waypoint = self.waypoint_list[0] if self.waypoint_list else None
        
        # Reset traveled length
        self.traveled_length = 0.0
        self.traveled_from_cmd_vel = 0.0
        self.last_robot_position = None
        self.last_cmd_vel_update_time = None
        
        self.publish_goal_status()
    
    def reset_mission_state(self):
        """Reset semua state mission"""
        self.get_logger().info("Resetting mission state...")
        self.goal_pose = None
        self.waypoint_pose = None
        self.goal_received = False
        self.waypoint_received = False
        self.goal_reached = False
        self.waypoint_reached = False
        self.global_path = []
        self.cached_path = []
        self.path_valid = False
        self.current_waypoint_index = 0
        self.waypoint_list = []
        self.waypoint_status = []
        self.waypoint_distances = []
        self.waypoint_remaining = []
        self.active_waypoint = None
        self.traveled_length = 0.0
        self.traveled_from_cmd_vel = 0.0
        
        # Publish reset status
        self.publish_goal_status()
        
        status_msg = String()
        status_msg.data = "Mission reset"
        self.waypoint_status_pub.publish(status_msg)
    
    # ========== NEW: WAYPOINT TRACKING FUNCTIONS ==========
    def calculate_waypoint_distances(self):
        """Hitung jarak kumulatif dari start ke setiap waypoint"""
        if not self.global_path or len(self.global_path) < 2:
            self.waypoint_distances = []
            self.waypoint_remaining = []
            self.waypoint_status = []
            return
        
        # Hitung jarak kumulatif
        cumulative_distance = 0.0
        self.waypoint_distances = [0.0]  # Start point memiliki jarak 0
        
        for i in range(1, len(self.global_path)):
            x1, y1 = self.global_path[i-1]
            x2, y2 = self.global_path[i]
            segment_distance = math.sqrt((x2 - x1)**2 + (y2 - y1)**2)
            cumulative_distance += segment_distance
            self.waypoint_distances.append(cumulative_distance)
        
        # Inisialisasi status waypoint
        self.waypoint_status = [False] * len(self.global_path)
        self.waypoint_remaining = self.waypoint_distances.copy()
        
        self.get_logger().info(f"Calculated distances for {len(self.waypoint_distances)} waypoints")
        for i, dist in enumerate(self.waypoint_distances):
            self.get_logger().debug(f"Waypoint {i}: cumulative distance = {dist:.2f}m")
    
    def update_waypoint_tracking_callback(self):
        """Update waypoint tracking berdasarkan posisi robot saat ini"""
        self.update_waypoint_tracking()
    
    def update_waypoint_tracking(self):
        """Update status waypoint berdasarkan posisi robot"""
        if not self.robot_pose or not self.global_path:
            return
        
        if self.goal_reached:
            # Semua waypoint sudah tercapai
            self.current_waypoint_index = len(self.global_path) - 1
            self.active_waypoint = self.global_path[-1]
            self.publish_waypoint_info()
            return
        
        robot_x, robot_y, _ = self.robot_pose
        
        """
        # Cari waypoint terdekat yang belum tercapai
        for i in range(self.current_waypoint_index, len(self.global_path)):
            wx, wy = self.global_path[i]
            distance_to_waypoint = math.sqrt((wx - robot_x)**2 + (wy - robot_y)**2)
            
            # Update sisa jarak untuk waypoint ini
            if i < len(self.waypoint_remaining):
                self.waypoint_remaining[i] = max(0.0, self.waypoint_distances[i] - self.traveled_length)
            
            # Jika robot dekat dengan waypoint, tandai sebagai tercapai
            if distance_to_waypoint <= self.waypoint_reached_threshold:
                if not self.waypoint_status[i]:
                    self.waypoint_status[i] = True
                    self.current_waypoint_index = i
                    
                    # Log khusus untuk waypoint dari master
                    if i == 0 and len(self.waypoint_list) > 1:
                        self.get_logger().info(f"✅ Waypoint dari Robot3 reached! ({wx:.2f}, {wy:.2f})")
                        self.waypoint_reached = True
                    elif i == len(self.global_path) - 1:
                        self.get_logger().info("🎉 FINAL GOAL (Robot4) REACHED!")
                        self.goal_reached = True
                        self.waypoint_reached = True
                    
                    self.publish_goal_status()
        """
        # Update active waypoint (waypoint berikutnya yang belum tercapai)
        if self.waypoint_list and len(self.waypoint_list) > 0:
            current_target = self.waypoint_list[0]
            target_x, target_y, _ = current_target
            distance = math.sqrt((target_x - robot_x)**2 + (target_y - robot_y)**2)
            
            # Update active waypoint
            self.active_waypoint = [target_x, target_y]
            
            # Publish sisa jarak
            remaining_msg = Float32()
            remaining_msg.data = float(distance)
            self.waypoint_remaining_pub.publish(remaining_msg)
    
    def check_waypoint_reached_callback(self):
        """High-frequency callback untuk check apakah waypoint/goal sudah tercapai"""
        self.check_waypoint_reached()
    
    def check_waypoint_reached(self):
        """Check apakah robot sudah mencapai waypoint atau goal"""
        if not self.robot_pose or not self.waypoint_list:
            return
        
        if self.goal_reached:
            return
        
        current_target = self.waypoint_list[0]  # [x, y, theta]
        target_x, target_y, target_theta = current_target
        robot_x, robot_y, robot_theta = self.robot_pose
        
        dx = target_x - robot_x
        dy = target_y - robot_y
        distance = math.sqrt(dx*dx + dy*dy)

        is_final_goal = (len(self.waypoint_list) == 1)
    
        if is_final_goal:
            # ===== GOAL FINAL =====
            # Butuh: posisi AKURAT (10cm) + orientasi AKURAT (5°)
            orientation_diff = target_theta - robot_theta
            orientation_diff = math.atan2(math.sin(orientation_diff), math.cos(orientation_diff))
            orientation_diff_abs = abs(orientation_diff)
            
            position_ok = distance <= self.position_threshold  # 0.1m
            orientation_ok = orientation_diff_abs <= self.orientation_threshold  # 5°
            
            condition_met = position_ok and orientation_ok
            
            self.get_logger().debug(
                f"Checking GOAL: dist={distance:.3f}m (need <{self.position_threshold}m), "
                f"orient={math.degrees(orientation_diff_abs):.1f}° (need <{math.degrees(self.orientation_threshold):.1f}°)"
            )
        else:
            # ===== WAYPOINT INTERMEDIATE =====
            # Cukup: posisi DEKAT (30cm), orientasi TIDAK PENTING
            position_ok = distance <= self.waypoint_reached_threshold  # 0.3m
            condition_met = position_ok  # Abaikan orientasi!
            
            self.get_logger().debug(
                f"Checking WAYPOINT: dist={distance:.3f}m (need <{self.waypoint_reached_threshold}m)"
            )
        
        if condition_met:
            # HAPUS point yang sudah dicapai
            reached_point = self.waypoint_list.pop(0)
            
            if is_final_goal:
                self.get_logger().info(f"🎉 FINAL GOAL REACHED!")
                self.goal_reached = True
            else:
                self.get_logger().info(f"✅ WAYPOINT REACHED! Moving to next...")
                # Replan ke target berikutnya
                self.compute_global_path_through_waypoints()
            
            self.publish_goal_status()
            self.publish_waypoint_info()
    
    def publish_waypoint_info(self, active_remaining=None):
        """Publish informasi waypoint ke berbagai topic"""
        if not self.global_path:
            return
        
        # Publish indeks waypoint saat ini
        idx_msg = Int32()
        idx_msg.data = self.current_waypoint_index
        self.waypoint_index_pub.publish(idx_msg)
        
        # Publish total waypoints
        total_msg = Int32()
        total_msg.data = len(self.global_path)
        self.waypoint_total_pub.publish(total_msg)
        
        # Publish active waypoint
        if self.active_waypoint and isinstance(self.active_waypoint, list):
            active_msg = PoseStamped()
            active_msg.header = Header()
            active_msg.header.stamp = self.get_clock().now().to_msg()
            active_msg.header.frame_id = "map"
            active_msg.pose.position.x = self.active_waypoint[0]
            active_msg.pose.position.y = self.active_waypoint[1]
            active_msg.pose.position.z = 0.0
            
            # Hitung orientasi menuju waypoint berikutnya
            if self.current_waypoint_index < len(self.global_path) - 1:
                next_wp = self.global_path[self.current_waypoint_index + 1]
                yaw = math.atan2(next_wp[1] - self.active_waypoint[1], 
                                 next_wp[0] - self.active_waypoint[0])
            else:
                yaw = self.goal_pose[2] if self.goal_pose else 0.0
            
            # Convert yaw to quaternion
            cy = math.cos(yaw * 0.5)
            sy = math.sin(yaw * 0.5)
            active_msg.pose.orientation.w = cy
            active_msg.pose.orientation.x = 0.0
            active_msg.pose.orientation.y = 0.0
            active_msg.pose.orientation.z = sy
            
            self.active_waypoint_pub.publish(active_msg)
        
        # Publish sisa jarak ke waypoint aktif
        if active_remaining is not None:
            remaining_msg = Float32()
            remaining_msg.data = float(active_remaining)
            self.waypoint_remaining_pub.publish(remaining_msg)
        
        # Publish sisa jarak ke waypoint berikutnya (untuk control)
        if self.current_waypoint_index < len(self.global_path) - 1:
            next_idx = self.current_waypoint_index + 1
            if next_idx < len(self.waypoint_remaining):
                next_remaining = self.waypoint_remaining[next_idx]
                next_msg = Float32()
                next_msg.data = float(next_remaining)
                self.next_waypoint_remaining_pub.publish(next_msg)
    
    # ========== MODIFIED: CMD_VEL CALLBACK ==========
    def cmd_vel_callback(self, msg):
        """Track traveled distance from cmd_vel commands"""
        current_time = time.time()
        
        if not self.cmd_vel_received:
            self.cmd_vel_received = True
            self.get_logger().info("cmd_vel received for distance tracking")
        
        # Hitung jarak tempuh sejak update terakhir
        if self.last_cmd_vel_update_time is not None:
            dt = current_time - self.last_cmd_vel_update_time
            
            # Hitung jarak = kecepatan × waktu
            distance = abs(msg.linear.x) * dt
            
            # Akumulasi jika signifikan
            if distance > 0.0005:  # 0.5 mm threshold
                self.traveled_from_cmd_vel += distance
                
                # Update traveled length utama
                self.traveled_length = self.traveled_from_cmd_vel
                
                # Update remaining length
                if self.path_length > 0:
                    self.remaining_length = max(0.0, self.path_length - self.traveled_length)
                    
                    # Publish immediately untuk respons real-time
                    self.publish_traveled_length(self.traveled_length)
                    self.publish_remaining_length(self.remaining_length)
                    
                    # Debug log dengan info waypoint
                    waypoint_info = ""
                    if self.current_waypoint_index < len(self.waypoint_list):
                        target = self.waypoint_list[self.current_waypoint_index]
                        if self.current_waypoint_index == 0 and len(self.waypoint_list) > 1:
                            waypoint_info = f" to Robot3 waypoint"
                        elif self.current_waypoint_index == len(self.waypoint_list) - 1:
                            waypoint_info = f" to Robot4 goal"
                    
                    self.get_logger().debug(
                        f"Cmd_vel:{waypoint_info}: +{distance:.3f}m, "
                        f"Total: {self.traveled_length:.2f}m, "
                        f"Remaining: {self.remaining_length:.2f}m",
                        throttle_duration_sec=2.0
                    )
        
        # Update state untuk next iteration
        self.last_cmd_vel_update_time = current_time
        self.cmd_vel_linear = msg.linear.x
    
    # ========== ORIGINAL FUNCTIONS (MODIFIED) ==========
    def map_callback(self, msg):
        """Process incoming map data"""
        if not self.map_received:
            self.map_received = True
            elapsed = time.time() - self.start_time
            self.get_logger().info(f"Map received after {elapsed:.2f} seconds")
        
        self.map_data = msg
        self.map_resolution = msg.info.resolution
        self.map_width = msg.info.width
        self.map_height = msg.info.height
        self.map_origin = [msg.info.origin.position.x, msg.info.origin.position.y]
        
        # Convert to numpy array and reshape
        data_array = np.array(msg.data, dtype=np.int8)
        self.original_grid = data_array.reshape((self.map_height, self.map_width))
        
        # Create binary grid
        self.create_binary_grid()
        
        # Create global costmap dengan gradient
        costmap_start = time.time()
        self.create_fast_global_costmap()
        self.costmap_creation_time = time.time() - costmap_start
        
        # Create planning grid (dengan inflation untuk A*)
        self.create_planning_grid()
        
        self.get_logger().info(f"Map: {self.map_width}x{self.map_height} cells, "
                              f"resolution: {self.map_resolution:.3f} m/cell")
        self.get_logger().info(f"Global costmap created with gradient inflation")
    
    def create_binary_grid(self):
        """Convert occupancy grid to binary grid"""
        self.binary_grid = np.zeros((self.map_height, self.map_width), dtype=np.int8)
        
        # Unknown cells (-1) dianggap sebagai obstacle untuk safety
        unknown_mask = (self.original_grid == -1)
        
        # Occupied cells
        occupied_mask = (self.original_grid > self.occupied_threshold)
        
        # Mark obstacles
        self.binary_grid[occupied_mask | unknown_mask] = 100
    
    def create_fast_global_costmap(self):
        """FAST VERSION: Create global costmap dengan vectorized operations"""
        if self.binary_grid is None:
            return
        
        start_time = time.time()
        
        # Initialize costmap dengan nilai free
        self.costmap = np.full((self.map_height, self.map_width), 
                               self.free_cost, dtype=np.uint8)
        
        # Find obstacle cells
        obstacle_cells = np.where(self.binary_grid == 100)
        obstacle_y, obstacle_x = obstacle_cells[0], obstacle_cells[1]
        
        if len(obstacle_y) == 0:
            self.costmap_created = True
            return
        
        self.get_logger().info(f"Creating fast costmap gradient for {len(obstacle_y)} obstacle cells...")
        
        # ========== FAST DISTANCE FIELD ==========
        # Initialize distance field
        distance_field = np.full((self.map_height, self.map_width), 
                                float('inf'), dtype=np.float32)
        
        # Inflation radius dalam cells
        inflation_cells = int(self.inflation_radius)
        max_inflation = inflation_cells * 2
        
        # Process obstacles dengan windowing untuk efisiensi
        for idx in range(len(obstacle_y)):
            y, x = obstacle_y[idx], obstacle_x[idx]
            
            # Calculate window bounds
            y_min = max(0, y - max_inflation)
            y_max = min(self.map_height, y + max_inflation + 1)
            x_min = max(0, x - max_inflation)
            x_max = min(self.map_width, x + max_inflation + 1)
            
            # Create coordinate grids dengan numpy (VECTORIZED)
            yy, xx = np.mgrid[y_min:y_max, x_min:x_max]
            
            # Calculate distances (VECTORIZED)
            distances = np.sqrt((xx - x)**2 + (yy - y)**2)
            
            # Update distance field (VECTORIZED)
            current_slice = distance_field[y_min:y_max, x_min:x_max]
            update_mask = distances < current_slice
            current_slice[update_mask] = distances[update_mask]
            distance_field[y_min:y_max, x_min:x_max] = current_slice
        
        # Set obstacle cells to 0 distance
        distance_field[obstacle_y, obstacle_x] = 0
        
        # ========== APPLY GRADIENT COSTS ==========
        # Lethal area (inner radius)
        lethal_mask = distance_field <= self.inflation_radius * 0.7
        self.costmap[lethal_mask] = self.lethal_cost
        
        # Inscribed radius area
        inscribed_mask = (distance_field > self.inflation_radius * 0.7) & \
                        (distance_field <= self.inflation_radius)
        self.costmap[inscribed_mask] = self.inscribed_cost
        
        # Gradient area
        gradient_mask = (distance_field > self.inflation_radius) & \
                       (distance_field <= self.inflation_radius * 1.5)
        
        # Exponential decay gradient (VECTORIZED)
        if np.any(gradient_mask):
            gradient_indices = np.where(gradient_mask)
            distances_grad = distance_field[gradient_indices]
            ratios = (distances_grad - self.inflation_radius) / (self.inflation_radius * 0.5)
            costs = self.inscribed_cost * np.exp(-ratios * 2.0)
            costs = np.clip(costs, self.free_cost, self.inscribed_cost).astype(np.uint8)
            self.costmap[gradient_indices] = costs
        
        self.costmap_created = True
        total_time = time.time() - start_time
        self.get_logger().info(f"✅ Fast costmap created in {total_time:.2f}s")
    
    def create_planning_grid(self):
        """Create planning grid from costmap (untuk A*)"""
        if self.costmap is None:
            return
        
        # Planning grid menggunakan costmap
        # Untuk A*, kita konversi costmap ke grid dengan threshold
        self.planning_grid = self.costmap.copy()
        
        # Mark cells dengan cost terlalu tinggi sebagai obstacle untuk planning
        obstacle_threshold = 200  # Cells dengan cost > 200 tidak bisa dilalui
        self.planning_grid[self.costmap > obstacle_threshold] = 100
        
        self.get_logger().info(f"Planning grid created from costmap")
    
    def amcl_callback(self, msg):
        """Process AMCL pose data"""
        if not self.amcl_received:
            self.amcl_received = True
            elapsed = time.time() - self.start_time
            self.get_logger().info(f"AMCL pose received after {elapsed:.2f} seconds")
        
        # Extract position
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        
        # Extract orientation from quaternion
        q = msg.pose.pose.orientation
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        yaw = math.atan2(siny_cosp, cosy_cosp)
        
        self.robot_pose = [x, y, yaw]
        
        self.get_logger().info(f"Robot pose: ({x:.2f}, {y:.2f}), "
                              f"theta: {math.degrees(yaw):.1f}°", 
                              throttle_duration_sec=2.0)
    
    # ========== MODIFIED: ENHANCED LENGTH UPDATES ==========
    def fast_length_update_callback(self):
        """10 Hz fast length updates dengan waypoint tracking"""
        if self.goal_reached:
            self.publish_remaining_length(0.0)
            self.publish_traveled_length(self.traveled_length)
            # Update waypoint tracking meski goal sudah tercapai
            self.update_waypoint_tracking()
            return
        
        if not self.robot_pose or not self.waypoint_list:
            return
        
        # Update traveled distance dari cmd_vel
        # (cmd_vel_callback sudah menangani ini)
        
        # Update waypoint tracking
        self.update_waypoint_tracking()
        self.check_waypoint_reached()
        
        # Jika menggunakan cmd_vel tracking, jarak sudah diupdate real-time
        # Tapi tetap hitung remaining untuk cross-check dan fallback
        if self.cached_path and len(self.cached_path) > 1:
            # Cross-check dengan geometric calculation
            geometric_remaining = self.calculate_remaining_from_cached_path()
            
            # Gunakan yang lebih kecil antara cmd_vel based dan geometric
            # untuk konservatif estimation
            if self.remaining_length > geometric_remaining:
                self.remaining_length = geometric_remaining
        else:
            # Fallback: Euclidean distance ke target saat ini
            if self.current_waypoint_index < len(self.waypoint_list):
                target = self.waypoint_list[self.current_waypoint_index]
                self.remaining_length = math.sqrt(
                    (target[0] - self.robot_pose[0])**2 + 
                    (target[1] - self.robot_pose[1])**2
                )
        
        # Publish lengths
        self.publish_remaining_length(self.remaining_length)
        self.publish_traveled_length(self.traveled_length)
        
        # Debug log dengan waypoint info
        if self.waypoint_list and self.current_waypoint_index < len(self.waypoint_list):
            target = self.waypoint_list[self.current_waypoint_index]
            dist_to_target = math.sqrt((target[0] - self.robot_pose[0])**2 + 
                                      (target[1] - self.robot_pose[1])**2)
            
            target_name = "Robot3 waypoint" if self.current_waypoint_index == 0 and len(self.waypoint_list) > 1 else "Robot4 goal"
            
            self.get_logger().debug(
                f"Target: {target_name}, "
                f"dist: {dist_to_target:.2f}m, "
                f"traveled: {self.traveled_length:.2f}m, "
                f"remaining: {self.remaining_length:.2f}m",
                throttle_duration_sec=1.0
            )
    
    def calculate_remaining_from_cached_path(self):
        """Hitung remaining length dari cached path dengan projection"""
        if not self.cached_path or len(self.cached_path) < 2:
            return 0.0
        
        robot_x, robot_y, _ = self.robot_pose
        
        # Cari titik terdekat di cached path
        closest_idx = 0
        min_dist = float('inf')
        
        for i in range(len(self.cached_path)):
            px, py = self.cached_path[i]
            dist = math.sqrt((px - robot_x)**2 + (py - robot_y)**2)
            if dist < min_dist:
                min_dist = dist
                closest_idx = i
        
        # Hitung remaining length dari titik terdekat ke goal
        remaining = min_dist  # Robot ke path
        
        # Path length dari closest point ke goal
        for i in range(closest_idx, len(self.cached_path) - 1):
            x1, y1 = self.cached_path[i]
            x2, y2 = self.cached_path[i + 1]
            remaining += math.sqrt((x2 - x1)**2 + (y2 - y1)**2)
        
        return remaining
    
    def update_traveled_distance(self):
        """Update traveled distance berdasarkan pergerakan robot (backup method)"""
        if not self.robot_pose:
            return
        
        current_pos = (self.robot_pose[0], self.robot_pose[1])
        
        if self.last_robot_position:
            # Hitung jarak dari posisi terakhir (geometric fallback)
            dx = current_pos[0] - self.last_robot_position[0]
            dy = current_pos[1] - self.last_robot_position[1]
            distance = math.sqrt(dx*dx + dy*dy)
            
            # Hanya tambah jika pergerakan signifikan
            if distance > 0.001:  # 1 mm threshold
                # Update sebagai fallback jika cmd_vel tidak tersedia
                if not self.cmd_vel_received:
                    self.traveled_length += distance
                
                # Update last position
                self.last_robot_position = current_pos
        else:
            # Inisialisasi pertama kali
            self.last_robot_position = current_pos
    
    def publish_remaining_length(self, length):
        """Publish remaining length"""
        length_msg = Float32()
        length_msg.data = float(length)
        self.remaining_length_pub.publish(length_msg)
    
    def publish_traveled_length(self, length):
        """Publish traveled length"""
        length_msg = Float32()
        length_msg.data = float(length)
        self.traveled_length_pub.publish(length_msg)
    
    def publish_goal_status(self):
        """Publish status apakah goal sudah tercapai"""
        status_msg = Bool()
        status_msg.data = self.goal_reached
        self.goal_status_pub.publish(status_msg)
        
        # Juga publish waypoint status
        waypoint_status_msg = String()
        if self.goal_reached:
            waypoint_status_msg.data = "GOAL_REACHED"
        elif self.waypoint_reached and len(self.waypoint_list) > 1:
            waypoint_status_msg.data = "WAYPOINT_REACHED"
        else:
            waypoint_status_msg.data = "IN_PROGRESS"
        self.waypoint_status_pub.publish(waypoint_status_msg)
        
        self.get_logger().debug(f"Published goal status: {'REACHED' if self.goal_reached else 'NOT REACHED'}",
                               throttle_duration_sec=1.0)
    
    # ========== MODIFIED: PATH COMPUTATION ==========
    def attempt_path_computation(self):
        """Attempt to compute path if all data is available"""
        # Skip path computation jika goal sudah tercapai
        if self.goal_reached:
            self.get_logger().debug("Goal already reached, skipping path computation")
            return
        
        if not self.map_received:
            self.get_logger().debug("Waiting for map data...")
            return
        
        if not self.costmap_created:
            self.get_logger().debug("Waiting for costmap creation...")
            return
        
        if not self.amcl_received:
            self.get_logger().debug("Waiting for AMCL pose...")
            return
        
        if not self.waypoint_list:
            self.get_logger().debug("Waiting for waypoints from master...")
            return
        
        # All data available, compute path melalui waypoints
        self.get_logger().info("All data available, computing global path through waypoints...")
        self.compute_global_path_through_waypoints()
    
    def compute_global_path_through_waypoints(self):
        """Compute path dari robot ke goal melalui waypoints menggunakan costmap-aware A*"""
        # Skip jika goal sudah tercapai
        if self.goal_reached:
            self.get_logger().debug("Goal already reached, skipping path computation")
            return
        
        if self.planning_grid is None:
            self.get_logger().warn("Planning grid not available")
            return
        
        if not self.waypoint_list:
            self.get_logger().warn("Waypoint list empty")
            return
        
        # ========== CEK JARAK KE WAYPOINT AKTIF ==========
        if self.robot_pose and self.waypoint_list:
            robot_x, robot_y, _ = self.robot_pose
            target_x, target_y, _ = self.waypoint_list[0]
            distance_to_target = math.sqrt((target_x - robot_x)**2 + (target_y - robot_y)**2)
            
            # Skip planning jika sudah sangat dekat dengan target (kecuali untuk path pertama)
            if self.global_path and distance_to_target < 0.5:  # 0.5 meter threshold
                self.get_logger().debug(f"Already close to target ({distance_to_target:.2f}m), skipping replan")
                return
        
        # ========== RESET TRAVELED LENGTH ==========
        # Reset traveled_length HANYA jika:
        # 1. Ini adalah path pertama kali (global_path kosong)
        # 2. Atau jika sedang berpindah ke waypoint berikutnya
        should_reset_traveled = False
        
        if not self.global_path or len(self.global_path) == 0:
            # Ini adalah path pertama
            should_reset_traveled = True
            self.get_logger().info("First path computation, resetting traveled distance")
        elif self.waypoint_list and len(self.waypoint_list) > 0:
            # Cek apakah target berubah (berpindah ke waypoint berikutnya)
            current_target = self.waypoint_list[0]
            if self.active_waypoint and self.active_waypoint != [current_target[0], current_target[1]]:
                # Target berubah, reset traveled distance
                should_reset_traveled = True
                self.get_logger().info(f"Target changed to new waypoint, resetting traveled distance")
        
        if should_reset_traveled:
            self.traveled_length = 0.0
            self.traveled_from_cmd_vel = 0.0
            self.last_cmd_vel_update_time = None
            self.last_robot_position = None
            self.get_logger().info(f"✅ Reset traveled_length to 0.0m")
        
        self.get_logger().info(f"Computing path through {len(self.waypoint_list)} waypoints")
        
        # Bangun path melalui semua waypoints
        all_path_points = []
        all_grid_paths = []
        
        # Start dari robot position
        current_start = (self.robot_pose[0], self.robot_pose[1])
        
        for i, waypoint in enumerate(self.waypoint_list):
            target_x, target_y, _ = waypoint
            target_world = (target_x, target_y)
            
            self.get_logger().info(f"Planning segment {i}: {current_start} -> {target_world}")
            
            # Convert to grid coordinates
            start_grid = self.world_to_grid(current_start[0], current_start[1])
            goal_grid = self.world_to_grid(target_world[0], target_world[1])
            
            if not start_grid or not goal_grid:
                self.get_logger().error(f"Invalid coordinates for segment {i}")
                return
            
            # Check if start or goal is in obstacle menggunakan costmap
            start_cost = self.costmap[start_grid[1], start_grid[0]]
            goal_cost = self.costmap[goal_grid[1], goal_grid[0]]
            
            if start_cost >= self.inscribed_cost or goal_cost >= self.inscribed_cost:
                self.get_logger().error(f"Segment {i}: Start or goal position is in/near obstacle")
                self.path_valid = False
                return
            
            # Run costmap-aware A* algorithm untuk segment ini
            astar_start = time.time()
            segment_grid_path = self.costmap_aware_astar(start_grid, goal_grid)
            self.astar_time = time.time() - astar_start
            
            if not segment_grid_path:
                self.get_logger().error(f"No path found for segment {i}")
                self.path_valid = False
                return
            
            # Convert grid path to world coordinates
            segment_world_path = self.smooth_path(segment_grid_path)
            
            # Tambahkan ke total path (hindari duplikasi titik pertama)
            if all_path_points:
                all_path_points.extend(segment_world_path[1:])  # Skip first point karena sama dengan last point sebelumnya
            else:
                all_path_points.extend(segment_world_path)
            
            all_grid_paths.extend(segment_grid_path)
            
            # Update start untuk segment berikutnya
            current_start = target_world
        
        # Store semua path
        self.path_grid = all_grid_paths
        self.global_path = all_path_points
        
        # Apply angle-based smoothing untuk sharp turns
        self.reduce_sharp_turns()
        
        # Cache path untuk fast length updates
        self.cached_path = self.global_path.copy()
        
        # Hitung panjang lintasan total
        self.path_length = self.calculate_path_length(self.global_path)
        
        # Reset remaining length ke path length
        self.remaining_length = self.path_length
        
        # Hitung waypoint distances
        self.calculate_waypoint_distances()
        
        # Reset last robot position untuk traveled distance calculation
        if self.robot_pose:
            self.last_robot_position = (self.robot_pose[0], self.robot_pose[1])
        
        # Reset waypoint tracking
        self.current_waypoint_index = 0
        if self.global_path:
            self.active_waypoint = self.global_path[0]
        
        # Update active target untuk reset logic
        if self.waypoint_list:
            current_target = self.waypoint_list[0]
            self.active_target_world = (current_target[0], current_target[1])
        
        self.path_valid = True
        self.total_plans += 1
        
        # Logging informatif
        self.get_logger().info(f"✅ Path through {len(self.waypoint_list)} waypoints computed:")
        self.get_logger().info(f"   Total waypoints in path: {len(self.global_path)}")
        self.get_logger().info(f"   Total path length: {self.path_length:.2f} meters")
        self.get_logger().info(f"   Traveled length reset to: {self.traveled_length:.2f} meters")
        self.get_logger().info(f"   Remaining length: {self.remaining_length:.2f} meters")
        self.get_logger().info(f"   A* time: {self.astar_time*1000:.1f}ms")
        
        # Log segment info
        if len(self.waypoint_list) > 1:
            self.get_logger().info(f"   Route: Robot -> Robot3 waypoint -> Robot4 goal")
        else:
            # Cek apakah ini goal final atau waypoint
            if self.goal_pose and self.waypoint_list[0] == self.goal_pose:
                self.get_logger().info(f"   Route: Robot -> Robot4 FINAL GOAL")
            else:
                self.get_logger().info(f"   Route: Robot -> Robot3 WAYPOINT")
        
        # Publish the path
        self.publish_path()
        
        # Publish waypoint info
        self.publish_waypoint_info()
    
    # ========== A* ALGORITHM (SAME AS BEFORE) ==========
    def costmap_aware_astar(self, start_grid, goal_grid):
        """
        A* algorithm yang aware terhadap costmap gradient
        Dengan pencatatan lengkap untuk visualisasi
        """
        if self.astar_visualization_enabled:
            # Reset semua data
            self.astar_nodes_data = []
            self.astar_open_set_data = []
            self.astar_closed_set_data = []
            self.astar_final_path = []
        
        # 8-direction movement untuk path yang lebih smooth
        directions = [(0, 1), (1, 0), (0, -1), (-1, 0),
                     (1, 1), (1, -1), (-1, 1), (-1, -1)]
        diagonal_cost = math.sqrt(2)
        
        # Priority queue untuk A*
        open_set = []
        heapq.heappush(open_set, (0, start_grid))
        
        # Dictionaries untuk tracking
        g_score = {start_grid: 0}  # Cost dari start ke node saat ini
        f_score = {start_grid: self.euclidean_heuristic(start_grid, goal_grid)}  # Total estimated cost
        came_from = {start_grid: None}
        
        # Untuk visualisasi
        closed_set = set()  # Closed set nodes
        iteration_counter = 0
        
        nodes_explored = 0
        
        while open_set and nodes_explored < self.max_astar_nodes:
            # Pop node dengan f_score terendah
            current_f, current = heapq.heappop(open_set)
            nodes_explored += 1
            iteration_counter += 1
            
            # ========== CAPTURE EXPANDED NODE ==========
            if self.astar_visualization_enabled:
                self._capture_astar_node_data(
                    node_id=nodes_explored,
                    node=current,
                    g_score=g_score[current],
                    f_score=current_f,
                    h_score=self.euclidean_heuristic(current, goal_grid),
                    status='expanded',
                    iteration=iteration_counter,
                    parent=came_from[current]
                )
            # ========== END CAPTURE ==========
            
            # Add to closed set
            closed_set.add(current)
            
            # EARLY EXIT: Jika sudah mencapai goal
            if current == goal_grid:
                # Capture open set snapshot sebelum exit
                if self.astar_visualization_enabled:
                    self._capture_open_set_snapshot(open_set, g_score, f_score, goal_grid, iteration_counter)
                break
            
            # Explore neighbors
            for idx, (dx, dy) in enumerate(directions):
                neighbor = (current[0] + dx, current[1] + dy)
                
                # Check bounds
                if not (0 <= neighbor[0] < self.map_width and 
                        0 <= neighbor[1] < self.map_height):
                    continue
                
                # Check if cell is traversable menggunakan planning grid
                if self.planning_grid[neighbor[1], neighbor[0]] > self.inscribed_cost:
                    continue
                
                # Skip jika neighbor sudah di closed set
                if neighbor in closed_set:
                    continue
                
                # Base movement cost (1 untuk orthogonal, √2 untuk diagonal)
                movement_cost = 1.0 if idx < 4 else diagonal_cost
                
                # Costmap cost (lebih tinggi = lebih berbahaya)
                cell_cost = self.costmap[neighbor[1], neighbor[0]]

                if cell_cost >= self.lethal_cost:  # 254
                    # Langsung skip lethal cells
                    continue
                elif cell_cost >= self.inscribed_cost:  # 253
                    # Very high cost untuk inscribed radius
                    costmap_weight = 50.0
                elif cell_cost >= self.possibly_inscribed_cost:  # 128
                    # High cost untuk possibly inscribed
                    costmap_weight = 10.0
                else:
                    # Gradien cost berdasarkan nilai costmap
                    costmap_weight = 1.0 + (cell_cost / 50.0) ** 2  # Quadratic
                
                # Tentative g_score
                tentative_g = g_score[current] + (movement_cost * costmap_weight)
                
                # ========== CAPTURE DISCOVERED NODE ==========
                is_new_node = neighbor not in g_score
                # ========== END CAPTURE ==========
                
                # Jika neighbor belum ada di g_score atau ditemukan path yang lebih baik
                if is_new_node or tentative_g < g_score[neighbor]:
                    # Update path
                    came_from[neighbor] = current
                    g_score[neighbor] = tentative_g
                    
                    # Hitung f_score = g_score + heuristic
                    heuristic = self.euclidean_heuristic(neighbor, goal_grid)
                    f_score[neighbor] = tentative_g + (heuristic * self.astar_heuristic_weight)
                    
                    # Add to open set jika belum ada
                    if is_new_node:
                        heapq.heappush(open_set, (f_score[neighbor], neighbor))
                    
                    # ========== CAPTURE DISCOVERED/UPDATE NODE ==========
                    if self.astar_visualization_enabled:
                        status = 'discovered' if is_new_node else 'updated'
                        self._capture_astar_node_data(
                            node_id=f"{nodes_explored}_{status}_{len(g_score)}",
                            node=neighbor,
                            g_score=tentative_g,
                            f_score=f_score[neighbor],
                            h_score=heuristic,
                            status=status,
                            iteration=iteration_counter,
                            parent=current
                        )
                    # ========== END CAPTURE ==========
            
            # Capture open set snapshot setiap 100 iterations atau di akhir
            if self.astar_visualization_enabled and (iteration_counter % 100 == 0 or len(open_set) == 0):
                self._capture_open_set_snapshot(open_set, g_score, f_score, goal_grid, iteration_counter)
        
        self.astar_explored_nodes = nodes_explored
        
        # Reconstruct path jika goal tercapai
        if goal_grid not in came_from:
            self.get_logger().warn(f"No path found after exploring {nodes_explored} nodes")
            
            # Capture final state meskipun tidak ada path
            if self.astar_visualization_enabled:
                self._export_astar_data_to_csv()
            return []
        
        # Reconstruct the path dari goal ke start
        path = []
        node = goal_grid
        while node is not None:
            path.append(node)
            
            # ========== CAPTURE FINAL PATH NODE ==========
            if self.astar_visualization_enabled:
                # Find and mark as final path
                for node_data in self.astar_nodes_data:
                    if (node_data['grid_x'] == node[0] and 
                        node_data['grid_y'] == node[1]):
                        node_data['status'] = 'final_path'
                        break
            # ========== END CAPTURE ==========
            
            node = came_from[node]
        path.reverse()
        
        # Capture all final path nodes
        if self.astar_visualization_enabled:
            for node in path:
                self.astar_final_path.append(node)
            
            # Capture closed set
            self.astar_closed_set_data = list(closed_set)
            
            # Capture final open set
            self._capture_open_set_snapshot(open_set, g_score, f_score, goal_grid, iteration_counter, is_final=True)
            
            # Ekspor data ke CSV
            self._export_astar_data_to_csv()
        
        return path
    
    # ========== HELPER FUNCTIONS (SAME AS BEFORE) ==========
    def _capture_astar_node_data(self, node_id, node, g_score, f_score, h_score, status, iteration, parent=None):
        """Capture data node A* untuk visualisasi"""
        grid_x, grid_y = node
        world_x, world_y = self.grid_to_world(grid_x, grid_y)
        
        # Ambil cost dari costmap
        costmap_value = self.costmap[grid_y, grid_x] if self.costmap is not None else 0
        
        # Hitung parent info jika ada
        parent_x, parent_y = None, None
        if parent:
            parent_x, parent_y = parent
            parent_wx, parent_wy = self.grid_to_world(parent_x, parent_y)
            parent_x, parent_y = parent_wx, parent_wy
        
        node_data = {
            'node_id': str(node_id),
            'grid_x': grid_x,
            'grid_y': grid_y,
            'world_x': world_x,
            'world_y': world_y,
            'g_score': float(g_score),
            'f_score': float(f_score),
            'h_score': float(h_score),
            'costmap_value': int(costmap_value),
            'status': status,
            'iteration': iteration,
            'parent_x': parent_x,
            'parent_y': parent_y,
            'timestamp': time.time()
        }
        
        self.astar_nodes_data.append(node_data)
    
    def _capture_open_set_snapshot(self, open_set, g_score, f_score, goal_grid, iteration, is_final=False):
        """Capture snapshot open set nodes"""
        snapshot_data = []
        
        # Copy open set untuk snapshot
        open_set_copy = list(open_set)
        
        for priority, node in open_set_copy:
            grid_x, grid_y = node
            world_x, world_y = self.grid_to_world(grid_x, grid_y)
            
            node_data = {
                'node_id': f"open_{iteration}_{grid_x}_{grid_y}",
                'grid_x': grid_x,
                'grid_y': grid_y,
                'world_x': world_x,
                'world_y': world_y,
                'g_score': float(g_score.get(node, 0)),
                'f_score': float(f_score.get(node, 0)),
                'h_score': float(self.euclidean_heuristic(node, goal_grid)),
                'costmap_value': int(self.costmap[grid_y, grid_x] if self.costmap is not None else 0),
                'status': 'open_set',
                'iteration': iteration,
                'priority': float(priority),
                'is_final_snapshot': is_final,
                'timestamp': time.time()
            }
            
            snapshot_data.append(node_data)
        
        # Tambahkan ke open set data
        self.astar_open_set_data.extend(snapshot_data)
    
    def _export_astar_data_to_csv(self):
        """Ekspor semua data A* ke file CSV"""
        try:
            # Update counter
            self.astar_export_counter += 1
            
            # 1. Export semua nodes
            all_nodes_file = f"{self.csv_base_path}/astar_all_nodes_{self.astar_export_counter}.csv"
            self._export_nodes_to_csv(self.astar_nodes_data, all_nodes_file, "all_nodes")
            
            # 2. Export open set data
            if self.astar_open_set_data:
                open_set_file = f"{self.csv_base_path}/astar_open_set_{self.astar_export_counter}.csv"
                self._export_nodes_to_csv(self.astar_open_set_data, open_set_file, "open_set")
            
            # 3. Export final path
            final_path_data = []
            for node in self.astar_final_path:
                grid_x, grid_y = node
                world_x, world_y = self.grid_to_world(grid_x, grid_y)
                node_data = {
                    'node_id': f"final_{grid_x}_{grid_y}",
                    'grid_x': grid_x,
                    'grid_y': grid_y,
                    'world_x': world_x,
                    'world_y': world_y,
                    'costmap_value': int(self.costmap[grid_y, grid_x] if self.costmap is not None else 0),
                    'status': 'final_path',
                    'timestamp': time.time()
                }
                final_path_data.append(node_data)
            
            if final_path_data:
                final_path_file = f"{self.csv_base_path}/astar_final_path_{self.astar_export_counter}.csv"
                self._export_nodes_to_csv(final_path_data, final_path_file, "final_path")
            
            # 4. Export summary statistics
            self._export_summary_statistics()
            
            self.get_logger().info(f"✅ A* data exported successfully (run #{self.astar_export_counter})")
            
        except Exception as e:
            self.get_logger().error(f"Failed to export A* data to CSV: {str(e)}")
    
    def _export_nodes_to_csv(self, nodes_data, file_path, data_type):
        """Ekspor nodes data ke CSV file"""
        if not nodes_data:
            return
        
        try:
            with open(file_path, 'w', newline='') as csvfile:
                # Determine fieldnames based on data
                if data_type == "all_nodes":
                    fieldnames = [
                        'node_id', 'grid_x', 'grid_y', 'world_x', 'world_y',
                        'g_score', 'f_score', 'h_score', 'costmap_value',
                        'status', 'iteration', 'parent_x', 'parent_y', 'timestamp'
                    ]
                elif data_type == "open_set":
                    fieldnames = [
                        'node_id', 'grid_x', 'grid_y', 'world_x', 'world_y',
                        'g_score', 'f_score', 'h_score', 'costmap_value',
                        'status', 'iteration', 'priority', 'is_final_snapshot', 'timestamp'
                    ]
                else:  # final_path
                    fieldnames = [
                        'node_id', 'grid_x', 'grid_y', 'world_x', 'world_y',
                        'costmap_value', 'status', 'timestamp'
                    ]
                
                writer = csv.DictWriter(csvfile, fieldnames=fieldnames)
                writer.writeheader()
                
                for node_data in nodes_data:
                    # Format float values
                    formatted_data = {}
                    for key, value in node_data.items():
                        if key in fieldnames:
                            if isinstance(value, float):
                                formatted_data[key] = f"{value:.6f}"
                            elif isinstance(value, int):
                                formatted_data[key] = str(value)
                            elif value is None:
                                formatted_data[key] = ""
                            else:
                                formatted_data[key] = str(value)
                    
                    writer.writerow(formatted_data)
            
            self.get_logger().debug(f"Exported {len(nodes_data)} {data_type} nodes to {file_path}")
            
        except Exception as e:
            self.get_logger().error(f"Failed to write {data_type} CSV: {str(e)}")
    
    def _export_summary_statistics(self):
        """Ekspor summary statistics dari A* run"""
        summary_file = f"{self.csv_base_path}/astar_summary_{self.astar_export_counter}.csv"
        
        try:
            # Hitung statistik
            expanded_nodes = sum(1 for n in self.astar_nodes_data if n['status'] == 'expanded')
            discovered_nodes = sum(1 for n in self.astar_nodes_data if n['status'] == 'discovered')
            updated_nodes = sum(1 for n in self.astar_nodes_data if n['status'] == 'updated')
            final_path_nodes = len(self.astar_final_path)
            open_set_nodes = len(self.astar_open_set_data)
            
            with open(summary_file, 'w', newline='') as csvfile:
                fieldnames = ['metric', 'value']
                writer = csv.DictWriter(csvfile, fieldnames=fieldnames)
                writer.writeheader()
                
                writer.writerow({'metric': 'total_astar_runs', 'value': self.astar_export_counter})
                writer.writerow({'metric': 'total_nodes_captured', 'value': len(self.astar_nodes_data)})
                writer.writerow({'metric': 'expanded_nodes', 'value': expanded_nodes})
                writer.writerow({'metric': 'discovered_nodes', 'value': discovered_nodes})
                writer.writerow({'metric': 'updated_nodes', 'value': updated_nodes})
                writer.writerow({'metric': 'final_path_nodes', 'value': final_path_nodes})
                writer.writerow({'metric': 'open_set_nodes', 'value': open_set_nodes})
                writer.writerow({'metric': 'astar_explored_nodes', 'value': self.astar_explored_nodes})
                writer.writerow({'metric': 'astar_time_ms', 'value': f"{self.astar_time*1000:.2f}"})
                writer.writerow({'metric': 'path_length_m', 'value': f"{self.path_length:.2f}"})
                writer.writerow({'metric': 'waypoints_total', 'value': len(self.global_path) if self.global_path else 0})
                writer.writerow({'metric': 'waypoints_reached', 'value': sum(self.waypoint_status) if self.waypoint_status else 0})
                writer.writerow({'metric': 'map_width', 'value': self.map_width})
                writer.writerow({'metric': 'map_height', 'value': self.map_height})
                writer.writerow({'metric': 'map_resolution', 'value': f"{self.map_resolution:.3f}"})
                writer.writerow({'metric': 'start_x', 'value': f"{self.robot_pose[0]:.2f}" if self.robot_pose else "N/A"})
                writer.writerow({'metric': 'start_y', 'value': f"{self.robot_pose[1]:.2f}" if self.robot_pose else "N/A"})
                writer.writerow({'metric': 'export_timestamp', 'value': datetime.now().isoformat()})
            
        except Exception as e:
            self.get_logger().error(f"Failed to export summary statistics: {str(e)}")
    
    def euclidean_heuristic(self, node, goal):
        """Euclidean distance heuristic untuk A* (admissible)"""
        dx = node[0] - goal[0]
        dy = node[1] - goal[1]
        return math.sqrt(dx*dx + dy*dy)
    
    def smooth_path(self, path_grid):
        """Smooth path dan downsample"""
        if len(path_grid) < 3:
            # Convert langsung jika path pendek
            return [self.grid_to_world(gx, gy) for (gx, gy) in path_grid]
        
        # Douglas-Peucker simplification sederhana
        simplified = []
        
        # Selalu tambah titik awal
        simplified.append(path_grid[0])
        
        i = 0
        while i < len(path_grid) - 1:
            # Cari titik terjauh yang masih "lurus"
            j = i + 2
            while j < len(path_grid):
                # Check jika titik i ke j membentuk garis lurus (dalam toleransi)
                if not self.is_straight_line(path_grid[i], path_grid[j], path_grid[i+1:j]):
                    break
                j += 1
            
            # Tambah titik j-1 ke simplified path
            simplified.append(path_grid[j-1])
            i = j - 1
        
        # Convert ke world coordinates
        world_path = []
        for i, (gx, gy) in enumerate(simplified):
            if i % 1 == 0 or i == len(simplified) - 1:  # Downsample
                wx, wy = self.grid_to_world(gx, gy)
                world_path.append([wx, wy])
        
        # Pastikan start dan goal ada
        start_world = self.grid_to_world(path_grid[0][0], path_grid[0][1])
        goal_world = self.grid_to_world(path_grid[-1][0], path_grid[-1][1])
        
        if len(world_path) == 0 or world_path[0] != start_world:
            world_path.insert(0, start_world)
        if world_path[-1] != goal_world:
            world_path.append(goal_world)
        
        return world_path
    
    def is_straight_line(self, start, end, intermediate_points):
        """Check if points form a straight line (dalam toleransi)"""
        if not intermediate_points:
            return True
        
        # Vector dari start ke end
        dx = end[0] - start[0]
        dy = end[1] - start[1]
        
        # Normalize
        length = math.sqrt(dx*dx + dy*dy)
        if length < 0.001:
            return True
        
        dx /= length
        dy /= length
        
        # Check setiap intermediate point
        for point in intermediate_points:
            # Vector dari start ke point
            px = point[0] - start[0]
            py = point[1] - start[1]
            
            # Projection ke line
            projection = px * dx + py * dy
            
            # Perpendicular distance
            perp_x = px - projection * dx
            perp_y = py - projection * dy
            distance = math.sqrt(perp_x*perp_x + perp_y*perp_y)
            
            # Jika distance terlalu besar, bukan garis lurus
            if distance > 1:  # Tolerance in cells
                return False
        
        return True
    
    def reduce_sharp_turns(self):
        """Reduksi sharp turns dengan menambahkan intermediate points"""
        if len(self.global_path) < 4:
            return
        
        i = 1
        while i < len(self.global_path) - 2:
            p0 = self.global_path[i-1]
            p1 = self.global_path[i]
            p2 = self.global_path[i+1]
            
            # Hitung vektor
            v1 = (p1[0] - p0[0], p1[1] - p0[1])
            v2 = (p2[0] - p1[0], p2[1] - p1[1])
            
            len_v1 = math.sqrt(v1[0]**2 + v1[1]**2)
            len_v2 = math.sqrt(v2[0]**2 + v2[1]**2)
            
            if len_v1 > 0 and len_v2 > 0:
                # Hitung sudut
                dot = (v1[0]*v2[0] + v1[1]*v2[1]) / (len_v1 * len_v2)
                angle = math.acos(max(-1.0, min(1.0, dot)))
                
                # Jika sudut terlalu tajam (> 90 derajat), tambahkan intermediate point
                if angle > self.smoothing_max_angle:
                    # Interpolasi antara p0 dan p2
                    interp_x = p0[0] + 0.7 * (p2[0] - p0[0])
                    interp_y = p0[1] + 0.7 * (p2[1] - p0[1])
                    
                    # Cek keamanan
                    if self.is_point_safe(interp_x, interp_y):
                        # Ganti p1 dengan intermediate point
                        self.global_path[i] = [interp_x, interp_y]
            
            i += 1
    
    def calculate_path_length(self, path_points):
        """Hitung total panjang lintasan dari serangkaian titik"""
        if len(path_points) < 2:
            return 0.0
        
        total_length = 0.0
        for i in range(len(path_points) - 1):
            x1, y1 = path_points[i]
            x2, y2 = path_points[i + 1]
            distance = math.sqrt((x2 - x1)**2 + (y2 - y1)**2)
            total_length += distance
        
        return total_length
    
    def is_point_safe(self, x, y):
        """Cek apakah titik aman (tidak di obstacle)"""
        grid_point = self.world_to_grid(x, y)
        if not grid_point:
            return False
        
        grid_x, grid_y = grid_point
        if not (0 <= grid_x < self.map_width and 0 <= grid_y < self.map_height):
            return False
        
        cell_cost = self.costmap[grid_y, grid_x]
        return cell_cost < self.inscribed_cost
    
    # ========== HELPER FUNCTIONS ==========
    def world_to_grid(self, world_x, world_y):
        """Convert world coordinates to grid indices"""
        grid_x = int((world_x - self.map_origin[0]) / self.map_resolution)
        grid_y = int((world_y - self.map_origin[1]) / self.map_resolution)
        
        if (0 <= grid_x < self.map_width and 0 <= grid_y < self.map_height):
            return (grid_x, grid_y)
        
        self.get_logger().warn(f"Coordinates out of map bounds: "
                              f"({world_x:.2f}, {world_y:.2f})")
        return None
    
    def grid_to_world(self, grid_x, grid_y):
        """Convert grid indices to world coordinates"""
        world_x = grid_x * self.map_resolution + self.map_origin[0]
        world_y = grid_y * self.map_resolution + self.map_origin[1]
        return (world_x, world_y)
    
    # ========== PUBLISHING ==========
    def publish_path_callback(self):
        """Timer callback to publish path dan length"""
        # Skip publish jika goal sudah tercapai
        if self.goal_reached:
            return
        
        if self.path_valid and self.global_path:
            self.publish_path()
    
    def publish_path(self):
        """Publish global path ke /robot2/plan dan length ke /robot2/path_length"""
        if not self.global_path or len(self.global_path) < 2:
            return
        
        # Create Path message
        path_msg = Path()
        path_msg.header = Header()
        path_msg.header.stamp = self.get_clock().now().to_msg()
        path_msg.header.frame_id = "map"
        
        # Add poses to the path
        for i, (x, y) in enumerate(self.global_path):
            pose = PoseStamped()
            pose.header = path_msg.header
            pose.pose.position.x = x
            pose.pose.position.y = y
            pose.pose.position.z = 0.0
            
            # Calculate orientation towards next point
            if i < len(self.global_path) - 1:
                next_x, next_y = self.global_path[i + 1]
                yaw = math.atan2(next_y - y, next_x - x)
            else:
                yaw = self.goal_pose[2] if self.goal_pose else 0.0
            
            # Convert yaw to quaternion
            cy = math.cos(yaw * 0.5)
            sy = math.sin(yaw * 0.5)
            
            pose.pose.orientation.w = cy
            pose.pose.orientation.x = 0.0
            pose.pose.orientation.y = 0.0
            pose.pose.orientation.z = sy
            
            path_msg.poses.append(pose)
        
        # Publish the path
        self.path_pub.publish(path_msg)
        
        # Publish TOTAL path length
        self.publish_path_length()
        
        # Publish waypoint info
        self.publish_waypoint_info()
        
        # Log dengan informasi waypoint
        reached_count = sum(self.waypoint_status) if self.waypoint_status else 0
        waypoint_info = f"Waypoints from master: {len(self.waypoint_list)}" if self.waypoint_list else "No waypoints"
        
        self.get_logger().info(f"Published path: {len(path_msg.poses)} poses, "
                              f"reached: {reached_count}, "
                              f"active: {self.current_waypoint_index}, "
                              f"{waypoint_info}, "
                              f"length: {self.path_length:.2f}m, "
                              f"traveled: {self.traveled_length:.2f}m, "
                              f"remaining: {self.remaining_length:.2f}m",
                              throttle_duration_sec=2.0)
    
    def publish_path_length(self):
        """Publish path length ke /robot2/path_length"""
        length_msg = Float32()
        length_msg.data = float(self.path_length)
        self.path_length_pub.publish(length_msg)
        
        # Debug log
        self.get_logger().debug(f"Published path length: {self.path_length:.2f}m",
                               throttle_duration_sec=1.0)
    
    def publish_costmap_callback(self):
        """Publish global costmap regularly"""
        if self.costmap is not None:
            self.publish_costmap()
    
    def publish_costmap(self):
        """Publish global costmap ke /robot2/global_costmap/costmap"""
        costmap_msg = OccupancyGrid()
        costmap_msg.header = Header()
        costmap_msg.header.stamp = self.get_clock().now().to_msg()
        costmap_msg.header.frame_id = "map"
        
        # Copy map info
        if self.map_data:
            costmap_msg.info = self.map_data.info
        
        # Convert costmap to occupancy grid format (0-100)
        # Costmap values: 0-254 -> OccupancyGrid: 0-100
        occupancy_data = (self.costmap / 254.0 * 100).astype(np.int8)
        
        # Set unknown cells to -1
        unknown_mask = (self.original_grid == -1)
        occupancy_data[unknown_mask] = -1
        
        costmap_msg.data = occupancy_data.flatten().tolist()
        
        self.costmap_pub.publish(costmap_msg)
        self.get_logger().debug(f"Published global costmap to /robot2/global_costmap/costmap",
                               throttle_duration_sec=5.0)
    
    def publish_performance_data(self):
        """Publish planning performance data"""
        perf_msg = Float32()
        perf_msg.data = float(self.astar_time * 1000)  # Convert to ms
        self.performance_pub.publish(perf_msg)
    
    # ========== MODIFIED: STATUS REPORTING ==========
    def status_report(self):
        """Periodic status report dengan waypoint info"""
        status = []
        
        if self.map_received:
            status.append(f"Map: {self.map_width}x{self.map_height}")
        else:
            status.append("Map: Waiting")
        
        if self.costmap_created:
            status.append(f"Costmap: ✓")
        else:
            status.append("Costmap: Creating...")
        
        if self.amcl_received:
            status.append(f"Robot: ({self.robot_pose[0]:.2f}, {self.robot_pose[1]:.2f})")
        else:
            status.append("Robot: Waiting")
        
        # Waypoint info dari master
        if self.waypoint_list:
            wp_count = len(self.waypoint_list)
            status.append(f"Master Points: {wp_count}")
            
            if wp_count > 1:
                status.append(f"Route: Robot3 Waypoint → Robot4 Goal")
                if self.current_waypoint_index == 0:
                    status.append(f"Current: Robot3 Waypoint")
                else:
                    status.append(f"Current: Robot4 Goal")
        else:
            status.append("Master Points: Waiting")
        
        if self.cmd_vel_received:
            status.append(f"Vel: {self.cmd_vel_linear:.2f}m/s")
        else:
            status.append("Vel: No cmd_vel")
        
        if self.goal_reached:
            status.append("GOAL REACHED! ✓")
        elif self.waypoint_reached and len(self.waypoint_list) > 1 and self.current_waypoint_index == 0:
            status.append("WAYPOINT REACHED! ✓")
        elif self.path_valid and self.global_path:
            # Hitung progress
            progress = (self.traveled_length / self.path_length * 100) if self.path_length > 0 else 0
            
            status.append(f"Path: {len(self.global_path)} wp")
            status.append(f"Total: {self.path_length:.2f}m")
            status.append(f"Traveled: {self.traveled_length:.2f}m")
            status.append(f"Remaining: {self.remaining_length:.2f}m")
            status.append(f"Progress: {progress:.1f}%")
            
            # Tampilkan info waypoint aktif jika ada
            if self.active_waypoint:
                status.append(f"Active WP: ({self.active_waypoint[0]:.2f}, {self.active_waypoint[1]:.2f})")
            
            status.append(f"Plans: {self.total_plans}")
            status.append(f"A* Exports: {self.astar_export_counter}")
        else:
            status.append("Path: Not computed")
        
        self.get_logger().info("Status: " + " | ".join(status))

def main(args=None):
    rclpy.init(args=args)
    node = GlobalPathPlanner()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        # Final performance report
        node.get_logger().info("=" * 70)
        node.get_logger().info("🛑 GLOBAL PATH PLANNER - FINAL STATS")
        node.get_logger().info(f"   Total Runtime: {time.time() - node.start_time:.1f}s")
        node.get_logger().info(f"   Costmap Creation Time: {node.costmap_creation_time:.2f}s")
        node.get_logger().info(f"   Total Path Computations: {node.total_plans}")
        node.get_logger().info(f"   A* Data Exports: {node.astar_export_counter}")
        node.get_logger().info(f"   Max A* Nodes Explored: {node.astar_explored_nodes}")
        node.get_logger().info(f"   Avg A* Time: {node.astar_time*1000:.1f}ms")
        node.get_logger().info(f"   Total Traveled Distance: {node.traveled_length:.2f}m")
        
        # Waypoint info
        if node.waypoint_list:
            node.get_logger().info(f"   Waypoints from Master: {len(node.waypoint_list)}")
            for i, wp in enumerate(node.waypoint_list):
                wp_name = "Robot3 waypoint" if i == 0 and len(node.waypoint_list) > 1 else "Robot4 goal"
                node.get_logger().info(f"     {wp_name}: ({wp[0]:.2f}, {wp[1]:.2f})")
        
        if node.global_path:
            node.get_logger().info(f"   Total Waypoints in Path: {len(node.global_path)}")
            reached_count = sum(node.waypoint_status) if node.waypoint_status else 0
            node.get_logger().info(f"   Waypoints Reached: {reached_count}")
        
        node.get_logger().info(f"   Final Waypoint Index: {node.current_waypoint_index}")
        node.get_logger().info(f"   Waypoint Status: {'REACHED' if node.waypoint_reached else 'NOT REACHED'}")
        node.get_logger().info(f"   Goal Status: {'REACHED' if node.goal_reached else 'NOT REACHED'}")
        node.get_logger().info(f"   A* Data saved to: {node.csv_base_path}")
        node.get_logger().info("=" * 70)
        
        node.get_logger().info("Node stopped by user")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()