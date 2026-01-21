#!/usr/bin/env python3

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
from std_msgs.msg import Header, Float32, Bool, String
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSReliabilityPolicy, QoSHistoryPolicy

class GlobalPathPlanner(Node):
    def __init__(self):
        super().__init__('global_path_planner')
        
        # Parameter
        self.occupied_threshold = 60      
        self.free_threshold = 15          
        self.inflation_radius = 15        # cells untuk robot radius
        self.map_resolution = 0.05
        
        # Goal checking parameters
        self.position_threshold = 0.1      # 10 cm threshold untuk posisi
        self.orientation_threshold = 0.175  # ~5 derajat dalam radian
        
        # Waypoint parameters
        self.waypoint_proximity_threshold = 0.2  # 20 cm threshold untuk mencapai waypoint
        
        # Costmap parameters
        self.costmap_scaling_factor = 3.0  # Gradient scaling
        self.lethal_cost = 254
        self.inscribed_cost = 253
        self.possibly_inscribed_cost = 128
        self.free_cost = 0
        
        # Dijkstra parameters
        self.max_dijkstra_nodes = 50000    # Max nodes untuk explore
        
        # Path smoothing parameters
        self.smoothing_max_angle = math.radians(60)  # Sudut maksimal sebelum smoothing

        self.dijkstra_data_saving_enabled = True  # Enable/disable data saving
        self.dijkstra_data_saved = False  # Flag untuk menandai data sudah disimpan
        self.dijkstra_save_path = "/home/sunrise/dijkstra_data"  # Path untuk penyimpanan
        
        # Data storage untuk Dijkstra
        self.dijkstra_nodes_data = []      # Untuk menyimpan data semua node Dijkstra
        self.dijkstra_open_set_data = []   # Untuk menyimpan open set nodes
        self.dijkstra_final_path = []      # Untuk menyimpan final path nodes
        
        # Membuat direktori jika belum ada
        os.makedirs(self.dijkstra_save_path, exist_ok=True)
        
        self.waypoints = []  # List of waypoints [[x, y, theta], ...]
        self.current_waypoint_index = 0  # Index waypoint yang sedang aktif
        self.final_goal_pose = None  # Goal akhir setelah semua waypoints
        self.waypoint_mode = False  # Mode waypoint aktif/tidak
        
        # Untuk path chaining
        self.full_path = []  # Path lengkap dari start ke goal melalui semua waypoints
        self.path_segments = []  # List of path segments untuk setiap waypoint
        self.accumulated_lengths = []  # Panjang akumulasi untuk setiap segment

        self.fast_update_hz = 10.0          # 10 Hz updates
        self.length_update_interval = 1.0 / self.fast_update_hz
        
        self.cmd_vel_linear = 0.0
        self.last_cmd_vel_update_time = None
        self.traveled_from_cmd_vel = 0.0
        
        self.robot_pose = None  # x, y, theta
        self.goal_pose = None   # [x, y, theta] untuk waypoint atau goal pose
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
        self.path_valid = False
        self.costmap_created = False
        self.goal_reached = False      # Flag apakah goal sudah tercapai
        self.cmd_vel_received = False  # Flag cmd_vel received
        self.waypoint_active = False   # Flag waypoint sedang aktif
        
        # Path data
        self.global_path = []          # Current path [x, y]
        self.cached_path = []          # Cached path untuk fast length updates
        self.path_grid = []           # Path dalam grid coordinates
        self.path_length = 0.0        # Total panjang lintasan (start ke goal)
        self.remaining_length = 0.0    # Remaining length (robot ke goal) - REAL-TIME
        self.traveled_length = 0.0     # Panjang yang sudah ditempuh
        
        # Untuk length tracking
        self.last_robot_position = None  # Untuk hitung traveled distance
        self.last_path_publication_time = 0.0
        
        # Performance tracking
        self.costmap_creation_time = 0.0
        self.dijkstra_explored_nodes = 0
        self.total_plans = 0
        self.dijkstra_time = 0.0
        self.start_time = time.time()

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
        
        self.create_subscription(
            OccupancyGrid,
            '/map',
            self.map_callback,
            map_qos
        )
        
        self.create_subscription(
            PoseWithCovarianceStamped,
            '/robot1/amcl_pose',
            self.amcl_callback,
            amcl_qos
        )
        
        self.create_subscription(
            PoseStamped,
            '/robot1/goal_pose',
            self.goal_callback,
            10
        )
        
        # Untuk Goal 1
        self.create_subscription(
            PoseStamped,
            '/robot3/goal_pose',
            self.waypoint_callback,
            10
        )
    
        self.create_subscription(
            Twist,
            '/robot1/cmd_vel',
            self.cmd_vel_callback,
            10
        )
        
        self.path_pub = self.create_publisher(Path, '/robot1/plan', 10)
        self.costmap_pub = self.create_publisher(OccupancyGrid, '/robot1/global_costmap/costmap', 10)
        self.path_length_pub = self.create_publisher(Float32, '/robot1/path_length', 10)
        self.remaining_length_pub = self.create_publisher(Float32, '/robot1/remaining_length', 10)
        self.traveled_length_pub = self.create_publisher(Float32, '/robot1/traveled_length', 10)
        self.goal_status_pub = self.create_publisher(Bool, '/robot1/goal_reached', 10)
        self.performance_pub = self.create_publisher(Float32, '/robot1/planning_time', 10)
        
        self.waypoint_status_pub = self.create_publisher(String, '/robot1/waypoint_status', 10)
        
        # Timers
        # ENHANCED: Real-time length updates (10 Hz)
        self.create_timer(self.length_update_interval, self.fast_length_update_callback)
        
        # ORIGINAL Timers
        self.create_timer(0.5, self.publish_path_callback)  # Publish path
        self.create_timer(0.5, self.publish_costmap_callback)  # Publish costmap
        self.create_timer(0.1, self.check_waypoint_reached_callback)  # High-frequency waypoint checking
        self.create_timer(0.1, self.check_goal_reached_callback)  # High-frequency goal checking
        self.create_timer(3.0, self.attempt_path_computation)
        self.create_timer(2.0, self.status_report)
        
        self.get_logger().info("=" * 70)
        self.get_logger().info("Global Path Planner - DIJKSTRA VERSION")
        self.get_logger().info("Dengan cmd_vel tracking untuk estimasi jarak real-time")
        self.get_logger().info("Dengan FITUR WAYPOINT dengan PATH CHAINING")
        self.get_logger().info("Dengan PENYIMPANAN DATA DIJKSTRA (sekali saja)")
        self.get_logger().info("=" * 70)
        self.get_logger().info(f"Global Path Planner with Goal Checking and Waypoints started")
    
    # Djikstra
    def _reset_dijkstra_data(self):
        """Reset data Dijkstra untuk penyimpanan baru"""
        self.dijkstra_nodes_data = []
        self.dijkstra_open_set_data = []
        self.dijkstra_final_path = []
        
    def _capture_dijkstra_node_data(self, node_id, node, g_cost, status, iteration, parent=None):
        """Capture data node Dijkstra untuk penyimpanan"""
        grid_x, grid_y = node
        world_x, world_y = self.grid_to_world(grid_x, grid_y)
        
        # Ambil cost dari costmap
        costmap_value = self.costmap[grid_y, grid_x] if self.costmap is not None else 0
        
        node_data = {
            'node_id': str(node_id),
            'grid_x': grid_x,
            'grid_y': grid_y,
            'world_x': world_x,
            'world_y': world_y,
            'g_cost': float(g_cost),
            'costmap_value': int(costmap_value),
            'status': status,
            'iteration': iteration,
            'parent_x': parent[0] if parent else None,
            'parent_y': parent[1] if parent else None,
            'timestamp': time.time()
        }
        
        self.dijkstra_nodes_data.append(node_data)
    
    def _capture_open_set_snapshot_dijkstra(self, open_set, g_cost, iteration):
        """Capture snapshot open set nodes untuk Dijkstra"""
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
                'g_cost': float(g_cost.get(node, 0)),
                'costmap_value': int(self.costmap[grid_y, grid_x] if self.costmap is not None else 0),
                'status': 'open_set',
                'iteration': iteration,
                'priority': float(priority),
                'timestamp': time.time()
            }
            
            snapshot_data.append(node_data)
        
        # Tambahkan ke open set data
        self.dijkstra_open_set_data.extend(snapshot_data)
    
    def _save_dijkstra_data(self):
        """Simpan semua data Dijkstra ke file CSV"""
        if not self.dijkstra_data_saving_enabled or self.dijkstra_data_saved:
            return
        
        try:
            timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
            save_dir = f"{self.dijkstra_save_path}/{timestamp}"
            os.makedirs(save_dir, exist_ok=True)
            
            # 1. Save all_nodes
            if self.dijkstra_nodes_data:
                all_nodes_file = f"{save_dir}/dijkstra_all_nodes.csv"
                self._save_nodes_to_csv(self.dijkstra_nodes_data, all_nodes_file, "all_nodes")
                self.get_logger().info(f"Saved all_nodes: {len(self.dijkstra_nodes_data)} nodes")
            
            # 2. Save final_path
            if self.dijkstra_final_path:
                final_path_data = []
                for node in self.dijkstra_final_path:
                    grid_x, grid_y = node
                    world_x, world_y = self.grid_to_world(grid_x, grid_y)
                    node_data = {
                        'node_id': f"final_{grid_x}_{grid_y}",
                        'grid_x': grid_x,
                        'grid_y': grid_y,
                        'world_x': world_x,
                        'world_y': world_y,
                        'costmap_value': int(self.costmap[grid_y, grid_x] if self.costmap is not None else 0),
                        'timestamp': time.time()
                    }
                    final_path_data.append(node_data)
                
                final_path_file = f"{save_dir}/dijkstra_final_path.csv"
                self._save_nodes_to_csv(final_path_data, final_path_file, "final_path")
                self.get_logger().info(f"Saved final_path: {len(self.dijkstra_final_path)} nodes")
            
            # 3. Save open_set
            if self.dijkstra_open_set_data:
                open_set_file = f"{save_dir}/dijkstra_open_set.csv"
                self._save_nodes_to_csv(self.dijkstra_open_set_data, open_set_file, "open_set")
                self.get_logger().info(f"Saved open_set: {len(self.dijkstra_open_set_data)} nodes")
            
            # 4. Save summary
            summary_file = f"{save_dir}/dijkstra_summary.csv"
            self._save_summary_statistics(summary_file)
            
            self.dijkstra_data_saved = True
            self.get_logger().info(f"Dijkstra data saved successfully to: {save_dir}")
            
        except Exception as e:
            self.get_logger().error(f"Failed to save Dijkstra data: {str(e)}")
    
    def _save_nodes_to_csv(self, nodes_data, file_path, data_type):
        """Save nodes data ke CSV file"""
        if not nodes_data:
            return
        
        try:
            with open(file_path, 'w', newline='') as csvfile:
                # Tentukan fieldnames berdasarkan tipe data
                if data_type == "all_nodes":
                    fieldnames = [
                        'node_id', 'grid_x', 'grid_y', 'world_x', 'world_y',
                        'g_cost', 'costmap_value', 'status', 'iteration',
                        'parent_x', 'parent_y', 'timestamp'
                    ]
                elif data_type == "open_set":
                    fieldnames = [
                        'node_id', 'grid_x', 'grid_y', 'world_x', 'world_y',
                        'g_cost', 'costmap_value', 'status', 'iteration',
                        'priority', 'timestamp'
                    ]
                else:  # final_path
                    fieldnames = [
                        'node_id', 'grid_x', 'grid_y', 'world_x', 'world_y',
                        'costmap_value', 'timestamp'
                    ]
                
                writer = csv.DictWriter(csvfile, fieldnames=fieldnames)
                writer.writeheader()
                
                for node_data in nodes_data:
                    writer.writerow(node_data)
            
        except Exception as e:
            self.get_logger().error(f"Failed to write {data_type} CSV: {str(e)}")
    
    def _save_summary_statistics(self, file_path):
        """Save summary statistics dari Dijkstra run"""
        try:
            # Hitung statistik
            expanded_nodes = sum(1 for n in self.dijkstra_nodes_data if n.get('status') == 'expanded')
            discovered_nodes = sum(1 for n in self.dijkstra_nodes_data if n.get('status') == 'discovered')
            updated_nodes = sum(1 for n in self.dijkstra_nodes_data if n.get('status') == 'updated')
            final_path_nodes = len(self.dijkstra_final_path)
            open_set_nodes = len(self.dijkstra_open_set_data)
            
            with open(file_path, 'w', newline='') as csvfile:
                fieldnames = ['metric', 'value']
                writer = csv.DictWriter(csvfile, fieldnames=fieldnames)
                writer.writeheader()
                
                writer.writerow({'metric': 'total_nodes_captured', 'value': len(self.dijkstra_nodes_data)})
                writer.writerow({'metric': 'expanded_nodes', 'value': expanded_nodes})
                writer.writerow({'metric': 'discovered_nodes', 'value': discovered_nodes})
                writer.writerow({'metric': 'updated_nodes', 'value': updated_nodes})
                writer.writerow({'metric': 'final_path_nodes', 'value': final_path_nodes})
                writer.writerow({'metric': 'open_set_nodes', 'value': open_set_nodes})
                writer.writerow({'metric': 'dijkstra_explored_nodes', 'value': self.dijkstra_explored_nodes})
                writer.writerow({'metric': 'dijkstra_time_ms', 'value': f"{self.dijkstra_time*1000:.2f}"})
                writer.writerow({'metric': 'path_length_m', 'value': f"{self.path_length:.2f}"})
                writer.writerow({'metric': 'map_width', 'value': self.map_width})
                writer.writerow({'metric': 'map_height', 'value': self.map_height})
                writer.writerow({'metric': 'map_resolution', 'value': f"{self.map_resolution:.3f}"})
                writer.writerow({'metric': 'saved_timestamp', 'value': datetime.now().isoformat()})
            
        except Exception as e:
            self.get_logger().error(f"Failed to save summary statistics: {str(e)}")
    
    def costmap_aware_dijkstra(self, start_grid, goal_grid, save_data=False):
        """Dijkstra algorithm yang aware terhadap costmap gradient dengan data saving"""
        # Reset data jika akan menyimpan
        if save_data and self.dijkstra_data_saving_enabled and not self.dijkstra_data_saved:
            self._reset_dijkstra_data()
        
        # 8-direction movement untuk path yang lebih smooth
        directions = [(0, 1), (1, 0), (0, -1), (-1, 0),
                     (1, 1), (1, -1), (-1, 1), (-1, -1)]
        diagonal_cost = math.sqrt(2)
        
        # Priority queue untuk Dijkstra
        open_set = []
        heapq.heappush(open_set, (0, start_grid))
        
        # Dictionaries untuk tracking
        g_score = {start_grid: 0}  # Cost dari start ke node saat ini
        came_from = {start_grid: None}
        
        nodes_explored = 0
        iteration_counter = 0
        
        # Start time untuk dijkstra_time
        dijkstra_start_time = time.time()
        
        while open_set and nodes_explored < self.max_dijkstra_nodes:
            # Pop node dengan g_score terendah
            current_g, current = heapq.heappop(open_set)
            nodes_explored += 1
            iteration_counter += 1
            
            # Capture node data jika menyimpan data
            if save_data and self.dijkstra_data_saving_enabled and not self.dijkstra_data_saved:
                self._capture_dijkstra_node_data(
                    node_id=nodes_explored,
                    node=current,
                    g_cost=g_score[current],
                    status='expanded',
                    iteration=iteration_counter,
                    parent=came_from[current]
                )
            
            # EARLY EXIT: jika sudah mencapai goal
            if current == goal_grid:
                if save_data and self.dijkstra_data_saving_enabled and not self.dijkstra_data_saved:
                    self._capture_open_set_snapshot_dijkstra(open_set, g_score, iteration_counter)
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
                
                # Jika neighbor belum ada di g_score atau ditemukan path yang lebih baik
                if neighbor not in g_score or tentative_g < g_score[neighbor]:
                    # Update path
                    came_from[neighbor] = current
                    g_score[neighbor] = tentative_g
                    
                    # Add to open set jika belum ada
                    heapq.heappush(open_set, (tentative_g, neighbor))
                    
                    # Capture node data jika menyimpan data
                    if save_data and self.dijkstra_data_saving_enabled and not self.dijkstra_data_saved:
                        status = 'discovered' if neighbor not in g_score else 'updated'
                        self._capture_dijkstra_node_data(
                            node_id=f"{nodes_explored}_{status}_{len(g_score)}",
                            node=neighbor,
                            g_cost=tentative_g,
                            status=status,
                            iteration=iteration_counter,
                            parent=current
                        )
            
            # Capture open set snapshot setiap 100 iterations
            if save_data and self.dijkstra_data_saving_enabled and not self.dijkstra_data_saved and iteration_counter % 100 == 0:
                self._capture_open_set_snapshot_dijkstra(open_set, g_score, iteration_counter)
        
        self.dijkstra_explored_nodes = nodes_explored
        self.dijkstra_time = time.time() - dijkstra_start_time
        
        # Reconstruct path jika goal tercapai
        if goal_grid not in came_from:
            self.get_logger().warn(f"No path found after exploring {nodes_explored} nodes")
            return []
        
        # Reconstruct the path dari goal ke start
        path = []
        node = goal_grid
        while node is not None:
            path.append(node)
            node = came_from[node]
        path.reverse()
        
        # Capture final path jika menyimpan data
        if save_data and self.dijkstra_data_saving_enabled and not self.dijkstra_data_saved:
            self.dijkstra_final_path = path.copy()
            
            # Mark nodes as final path
            for node in path:
                for node_data in self.dijkstra_nodes_data:
                    if node_data['grid_x'] == node[0] and node_data['grid_y'] == node[1]:
                        node_data['status'] = 'final_path'
                        break
            
            # Capture final open set
            self._capture_open_set_snapshot_dijkstra(open_set, g_score, iteration_counter)
            
            # Simpan data
            self._save_dijkstra_data()
        
        return path
    
    # New waypoint
    def waypoint_callback(self, msg):
        """Process waypoint pose dari Rviz (robot3/goal_pose) dengan path chaining"""
        # Extract position
        x = msg.pose.position.x
        y = msg.pose.position.y
        
        # Extract orientation
        q = msg.pose.orientation
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        yaw = math.atan2(siny_cosp, cosy_cosp)
        
        waypoint = [x, y, yaw]
        
        # Tambahkan waypoint ke daftar
        self.waypoints.append(waypoint)
        
        # Aktifkan mode waypoint jika belum aktif
        if not self.waypoint_mode:
            self.waypoint_mode = True
            self.current_waypoint_index = 0
            self.waypoint_active = True
            self.get_logger().info(f"Waypoint mode activated! First waypoint added.")
            
            # Set waypoint pertama sebagai target saat ini
            self.final_goal_pose = waypoint
            self.goal_pose = waypoint
            self.goal_received = True
            self.goal_reached = False
            self.publish_goal_status()
            
            # Clear existing paths
            self.full_path = []
            self.path_segments = []
            self.accumulated_lengths = []
            
            self.get_logger().info(f"First waypoint set as immediate goal")
        else:
            # Jika sudah ada waypoints sebelumnya, kita perlu recalculate full path
            self.get_logger().info(f"Waypoint {len(self.waypoints)} added. Recalculating full path with chaining...")
            
            # Set goal terakhir sebagai final goal
            self.final_goal_pose = waypoint
            self.goal_pose = waypoint
            
            # Trigger recomputation of full path
            self.recompute_full_path_with_chaining()
        
        self.publish_waypoint_status()
    
    def recompute_full_path_with_chaining(self):
        """Hitung ulang full path dari robot ke goal melalui semua waypoints"""
        if not self.waypoints or len(self.waypoints) < 1:
            return
        
        # Clear existing data
        self.full_path = []
        self.path_segments = []
        self.accumulated_lengths = []
        
        total_path_length = 0.0
        accumulated_length = 0.0
        
        # Start dari posisi robot saat ini
        if self.robot_pose:
            start_point = (self.robot_pose[0], self.robot_pose[1])
        else:
            self.get_logger().warn("Cannot compute path: robot pose not available")
            return
        
        self.get_logger().info(f"Computing chained path through {len(self.waypoints)} waypoints")
        
        # Untuk setiap segment (start -> WP1, WP1 -> WP2, ...)
        for i in range(len(self.waypoints)):
            if i == 0:
                # Segment pertama: Robot -> WP1
                segment_start = start_point
            else:
                # Segment lainnya: WP(i-1) -> WP(i)
                segment_start = (self.waypoints[i-1][0], self.waypoints[i-1][1])
            
            segment_end = (self.waypoints[i][0], self.waypoints[i][1])
            
            # Compute path untuk segment ini (save data hanya untuk segment pertama)
            save_data = (i == 0 and not self.dijkstra_data_saved)
            segment_path = self.compute_path_between_points(segment_start, segment_end, save_data)
            
            if segment_path:
                # Simpan segment
                self.path_segments.append(segment_path)
                
                # Hitung panjang segment
                segment_length = self.calculate_path_length(segment_path)
                
                # Update akumulasi
                accumulated_length += segment_length
                self.accumulated_lengths.append(accumulated_length)
                
                # Tambahkan ke full path (hilangkan titik pertama jika bukan segment pertama)
                if i == 0:
                    self.full_path.extend(segment_path)
                else:
                    # Hindari duplikasi waypoint (titik terakhir segment sebelumnya = titik pertama segment ini)
                    self.full_path.extend(segment_path[1:])
                
                total_path_length += segment_length
                
                self.get_logger().info(f"Segment {i+1}: {len(segment_path)} points, length: {segment_length:.2f}m")
            else:
                self.get_logger().error(f"Cannot compute path for segment {i+1}")
                return
        
        # Update global path
        self.global_path = self.full_path.copy()
        self.cached_path = self.full_path.copy()
        self.path_length = total_path_length
        
        # Update goal untuk current waypoint
        self.goal_pose = self.waypoints[self.current_waypoint_index]
        
        self.path_valid = True
        
        self.get_logger().info(f"Path Computed: {len(self.global_path)} waypoints, "
                              f"total length: {self.path_length:.2f}m")
        
        # Publish the path
        self.publish_path()
    
    def compute_path_between_points(self, start_world, goal_world, save_data=False):
        """Compute path antara dua titik dunia menggunakan Dijkstra"""
        if not self.costmap_created or self.planning_grid is None:
            return None
        
        # Convert to grid coordinates
        start_grid = self.world_to_grid(start_world[0], start_world[1])
        goal_grid = self.world_to_grid(goal_world[0], goal_world[1])
        
        if not start_grid or not goal_grid:
            self.get_logger().warn(f"Invalid coordinates for path computation")
            return None
        
        # Check if points are traversable
        start_cost = self.costmap[start_grid[1], start_grid[0]]
        goal_cost = self.costmap[goal_grid[1], goal_grid[0]]
        
        if start_cost >= self.inscribed_cost or goal_cost >= self.inscribed_cost:
            self.get_logger().warn(f"Start or goal position is in/near obstacle")
            return None
        
        # Run Dijkstra algorithm
        path_grid = self.costmap_aware_dijkstra(start_grid, goal_grid, save_data)
        
        if not path_grid:
            return None
        
        # Convert grid path to world coordinates
        world_path = []
        for (gx, gy) in path_grid:
            wx, wy = self.grid_to_world(gx, gy)
            world_path.append([wx, wy])
        
        # Apply smoothing
        smoothed_path = self.smooth_path_direct(world_path)
        
        return smoothed_path
    
    def smooth_path_direct(self, path_points):
        """Smoothing sederhana untuk path points"""
        if len(path_points) < 3:
            return path_points
        
        smoothed = []
        
        # Selalu tambah titik pertama
        smoothed.append(path_points[0])
        
        # Downsample dengan skipping beberapa titik
        skip_interval = max(1, len(path_points) // 20)  # Maksimal 20 titik
        for i in range(0, len(path_points), skip_interval):
            if i != 0 and i != len(path_points) - 1:
                smoothed.append(path_points[i])
        
        # Selalu tambah titik terakhir
        if path_points[-1] not in smoothed:
            smoothed.append(path_points[-1])
        
        return smoothed
    
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
                    
                    # Debug log
                    self.get_logger().debug(
                        f"Cmd_vel distance: +{distance:.3f}m, "
                        f"Total: {self.traveled_length:.2f}m, "
                        f"Remaining: {self.remaining_length:.2f}m",
                        throttle_duration_sec=2.0
                    )
        
        # Update state untuk next iteration
        self.last_cmd_vel_update_time = current_time
        self.cmd_vel_linear = msg.linear.x
    
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
        
        # Create planning grid (dengan inflation untuk Dijkstra)
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
            
            # Create coordinate grids dengan numpy
            yy, xx = np.mgrid[y_min:y_max, x_min:x_max]
            
            # Calculate distance
            distances = np.sqrt((xx - x)**2 + (yy - y)**2)
            
            # Update distance field
            current_slice = distance_field[y_min:y_max, x_min:x_max]
            update_mask = distances < current_slice
            current_slice[update_mask] = distances[update_mask]
            distance_field[y_min:y_max, x_min:x_max] = current_slice
        
        # Set obstacle cells to 0 distance
        distance_field[obstacle_y, obstacle_x] = 0
        
        # Lethal area
        lethal_mask = distance_field <= self.inflation_radius * 0.7
        self.costmap[lethal_mask] = self.lethal_cost
        
        # Inscribed radius area
        inscribed_mask = (distance_field > self.inflation_radius * 0.7) & \
                        (distance_field <= self.inflation_radius)
        self.costmap[inscribed_mask] = self.inscribed_cost
        
        # Gradient area
        gradient_mask = (distance_field > self.inflation_radius) & \
                       (distance_field <= self.inflation_radius * 1.5)
        
        # Exponential decay gradient
        if np.any(gradient_mask):
            gradient_indices = np.where(gradient_mask)
            distances_grad = distance_field[gradient_indices]
            ratios = (distances_grad - self.inflation_radius) / (self.inflation_radius * 0.5)
            costs = self.inscribed_cost * np.exp(-ratios * 2.0)
            costs = np.clip(costs, self.free_cost, self.inscribed_cost).astype(np.uint8)
            self.costmap[gradient_indices] = costs
        
        self.costmap_created = True
        total_time = time.time() - start_time
        self.get_logger().info(f"Fast costmap created in {total_time:.2f}s")
    
    def create_planning_grid(self):
        """Create planning grid from costmap (untuk Dijkstra)"""
        if self.costmap is None:
            return
        
        # Planning grid menggunakan costmap
        # Untuk Dijkstra, kita konversi costmap ke grid dengan threshold
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
        
        # Jika mode waypoint aktif dan belum ada goal yang ditetapkan,
        # set waypoint pertama sebagai goal
        if self.waypoint_mode and self.waypoints and not self.goal_received:
            self.process_next_waypoint()
    
    def goal_callback(self, msg):
        """Process goal pose from Rviz (untuk goal akhir) dengan path chaining"""
        # Extract position
        x = msg.pose.position.x
        y = msg.pose.position.y
        
        # Extract orientation
        q = msg.pose.orientation
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        yaw = math.atan2(siny_cosp, cosy_cosp)
        
        final_goal = [x, y, yaw]
        
        # Jika mode waypoint aktif, tambahkan sebagai waypoint terakhir
        if self.waypoint_mode:
            self.get_logger().info(f"Final goal set as waypoint {len(self.waypoints) + 1}: ({x:.2f}, {y:.2f})")
            
            # Tambahkan ke daftar waypoints
            self.waypoints.append(final_goal)
            self.final_goal_pose = final_goal
            
            # Recompute full path dengan chaining
            self.recompute_full_path_with_chaining()
        else:
            # Mode normal: langsung set sebagai goal
            self.goal_pose = final_goal
            self.final_goal_pose = final_goal
            
            # Reset goal reached status saat goal baru diterima
            self.goal_reached = False
            self.publish_goal_status()
            
            # Reset traveled length saat goal baru
            self.traveled_length = 0.0
            self.traveled_from_cmd_vel = 0.0
            self.last_robot_position = None
            self.last_cmd_vel_update_time = None
            
            # Clear waypoint data
            self.waypoints = []
            self.full_path = []
            self.path_segments = []
            self.accumulated_lengths = []
            
            self.get_logger().info(f"Goal set (normal mode): ({x:.2f}, {y:.2f}), "
                                  f"theta: {math.degrees(yaw):.1f}°")
            
            # Trigger path computation
            if self.map_received and self.amcl_received:
                # Reset data saving flag untuk memungkinkan penyimpanan baru
                if self.dijkstra_data_saving_enabled:
                    self.dijkstra_data_saved = False
                self.attempt_path_computation()
    
    # real-time update
    def fast_length_update_callback(self):
        """10 Hz fast length updates tanpa replanning"""
        if self.goal_reached:
            self.publish_remaining_length(0.0)
            self.publish_traveled_length(self.traveled_length)
            return
        
        if not self.robot_pose or not self.goal_pose:
            return
        
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
            # Fallback: Euclidean distance
            self.remaining_length = self.calculate_euclidean_distance()
        
        # Publish lengths
        self.publish_remaining_length(self.remaining_length)
        self.publish_traveled_length(self.traveled_length)
        
        # Debug log
        self.get_logger().debug(
            f"Length update: remaining={self.remaining_length:.2f}m, "
            f"traveled={self.traveled_length:.2f}m, "
            f"cmd_vel_linear={self.cmd_vel_linear:.2f}m/s",
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
    
    def calculate_euclidean_distance(self):
        """Hitung Euclidean distance ke goal"""
        if not self.robot_pose or not self.goal_pose:
            return 0.0
        
        dx = self.goal_pose[0] - self.robot_pose[0]
        dy = self.goal_pose[1] - self.robot_pose[1]
        return math.sqrt(dx*dx + dy*dy)
    
    def update_traveled_distance(self):
        """Update traveled distance berdasarkan pergerakan robot (backup method)"""
        if not self.robot_pose:
            return
        
        current_pos = (self.robot_pose[0], self.robot_pose[1])
        
        if self.last_robot_position:
            # Hitung jarak dari posisi terakhir
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
    
    # goal terakhir
    def check_goal_reached_callback(self):
        """High-frequency callback untuk check apakah goal sudah tercapai"""
        self.check_goal_reached()
    
    def check_goal_reached(self):
        """Check apakah robot sudah mencapai goal dengan threshold posisi dan orientasi"""
        # Skip jika mode waypoint aktif (waypoint checking akan handle)
        if self.waypoint_active:
            return
        
        if not self.goal_received or not self.amcl_received:
            return
        
        if self.goal_reached:
            return  # Sudah mencapai goal, tidak perlu check lagi
        
        # Hitung jarak Euclidean antara robot dan goal
        dx = self.goal_pose[0] - self.robot_pose[0]
        dy = self.goal_pose[1] - self.robot_pose[1]
        distance = math.sqrt(dx*dx + dy*dy)
        
        # Hitung perbedaan orientasi (dalam range [-pi, pi])
        orientation_diff = self.goal_pose[2] - self.robot_pose[2]
        # Normalize ke range [-pi, pi]
        orientation_diff = math.atan2(math.sin(orientation_diff), math.cos(orientation_diff))
        orientation_diff_abs = abs(orientation_diff)
        
        # Check threshold
        position_ok = distance <= self.position_threshold
        orientation_ok = orientation_diff_abs <= self.orientation_threshold
        
        if position_ok and orientation_ok:
            if not self.goal_reached:
                self.goal_reached = True
                self.get_logger().info(f"GOAL REACHED! ✓")
                self.get_logger().info(f"  Distance: {distance:.3f}m (threshold: {self.position_threshold}m)")
                self.get_logger().info(f"  Orientation diff: {math.degrees(orientation_diff_abs):.1f}° (threshold: {math.degrees(self.orientation_threshold):.1f}°)")
                self.get_logger().info(f"  Total traveled: {self.traveled_length:.2f}m")
                
                # Publish status bahwa goal sudah tercapai
                self.publish_goal_status()
                
                # Clear current path karena sudah sampai
                self.global_path = []
                self.cached_path = []
                self.path_valid = False
                
                # Publish zero lengths
                self.publish_remaining_length(0.0)
                self.publish_path_length(0.0)
        else:
            # Debug log (hanya saat belum mencapai goal)
            if not self.goal_reached:
                self.get_logger().debug(
                    f"Goal check: dist={distance:.3f}m/{self.position_threshold}m, "
                    f"orient={math.degrees(orientation_diff_abs):.1f}°/{math.degrees(self.orientation_threshold):.1f}°",
                    throttle_duration_sec=2.0
                )
    
    def publish_goal_status(self):
        """Publish status apakah goal sudah tercapai"""
        status_msg = Bool()
        status_msg.data = self.goal_reached
        self.goal_status_pub.publish(status_msg)
        
        self.get_logger().debug(f"Published goal status: {'REACHED' if self.goal_reached else 'NOT REACHED'}",
                               throttle_duration_sec=1.0)
    
    # MODIFIED: ATTEMPT PATH COMPUTATION untuk PATH CHAINING 
    def attempt_path_computation(self):
        """Attempt to compute path if all data is available"""
        # Skip jika goal sudah tercapai
        if self.goal_reached:
            return
        
        if not self.map_received or not self.costmap_created or not self.amcl_received:
            return
        
        # Jika mode waypoint aktif dan sudah ada full path, tidak perlu compute ulang
        if self.waypoint_mode and self.full_path and len(self.full_path) > 0:
            # Cukup update path untuk posisi robot saat ini
            self.update_path_for_current_position()
            return
        
        # Jika tidak ada waypoint, compute path normal
        if not self.waypoint_mode and self.goal_received:
            # Reset data saving flag jika belum disimpan
            if self.dijkstra_data_saving_enabled and not self.dijkstra_data_saved:
                self.dijkstra_data_saved = False
            self.compute_global_path()
    
    def update_path_for_current_position(self):
        """Update path berdasarkan posisi robot saat ini di full path"""
        if not self.full_path or len(self.full_path) < 2:
            return
        
        if not self.robot_pose:
            return
        
        robot_x, robot_y, _ = self.robot_pose
        
        # Cari titik terdekat di full path
        closest_idx = 0
        min_dist = float('inf')
        
        for i in range(len(self.full_path)):
            px, py = self.full_path[i]
            dist = math.sqrt((px - robot_x)**2 + (py - robot_y)**2)
            if dist < min_dist:
                min_dist = dist
                closest_idx = i
        
        # Ambil subpath dari titik terdekat sampai akhir
        if closest_idx < len(self.full_path) - 1:
            self.global_path = self.full_path[closest_idx:]
            self.cached_path = self.global_path.copy()
            
            # Hitung remaining length
            remaining = 0.0
            for i in range(len(self.global_path) - 1):
                x1, y1 = self.global_path[i]
                x2, y2 = self.global_path[i + 1]
                remaining += math.sqrt((x2 - x1)**2 + (y2 - y1)**2)
            
            self.remaining_length = remaining
            
            # Update traveled length
            if self.path_length > 0:
                self.traveled_length = self.path_length - self.remaining_length
            
            self.get_logger().debug(f"Updated path from position: {len(self.global_path)} points remaining")
    
    # MODIFIED: COMPUTE GLOBAL PATH untuk PATH CHAINING
    def compute_global_path(self):
        """Compute path dari robot ke goal menggunakan costmap-aware Dijkstra"""
        # Skip jika goal sudah tercapai
        if self.goal_reached:
            return
        
        if self.planning_grid is None or self.goal_pose is None:
            return
        
        start_world = (self.robot_pose[0], self.robot_pose[1])
        goal_world = (self.goal_pose[0], self.goal_pose[1])
        
        self.get_logger().info(f"Computing direct path: {start_world} -> {goal_world}")
        
        # Compute path langsung
        save_data = self.dijkstra_data_saving_enabled and not self.dijkstra_data_saved
        direct_path = self.compute_path_between_points(start_world, goal_world, save_data)
        
        if not direct_path:
            self.get_logger().error("No path found")
            self.path_valid = False
            return
        
        # Update global path
        self.global_path = direct_path
        self.cached_path = direct_path.copy()
        self.full_path = direct_path.copy()
        
        # Hitung panjang
        self.path_length = self.calculate_path_length(direct_path)
        self.remaining_length = self.path_length
        
        self.path_valid = True
        self.total_plans += 1
        
        self.get_logger().info(f"Direct path computed: {len(self.global_path)} waypoints, "
                              f"length: {self.path_length:.2f} meters")
        
        # Publish the path
        self.publish_path()
    
    # waypoint function
    def process_next_waypoint(self):
        """Process waypoint berikutnya dalam daftar (saat mencapai waypoint)"""
        if not self.waypoint_mode or not self.waypoints:
            return
        
        # Increment waypoint index
        self.current_waypoint_index += 1
        
        if self.current_waypoint_index >= len(self.waypoints):
            # Semua waypoints sudah tercapai
            self.get_logger().info("All waypoints have been reached!")
            self.waypoint_mode = False
            self.waypoint_active = False
            self.publish_waypoint_status()
            
            # Set goal reached jika ini adalah goal akhir
            self.goal_reached = True
            self.publish_goal_status()
            return
        
        # Update current goal ke waypoint berikutnya
        next_waypoint = self.waypoints[self.current_waypoint_index]
        self.goal_pose = next_waypoint
        
        self.get_logger().info(f"➡️ Moving to waypoint {self.current_waypoint_index + 1}/{len(self.waypoints)}")
        self.get_logger().info(f"  Position: ({next_waypoint[0]:.2f}, {next_waypoint[1]:.2f})")
        
        # Tidak perlu recalculate path karena sudah ada full path
        # Cukup update status
        
        self.publish_waypoint_status()
    
    def check_waypoint_reached_callback(self):
        """Check apakah waypoint saat ini sudah tercapai"""
        self.check_waypoint_reached()
    
    def check_waypoint_reached(self):
        """Check apakah robot sudah mencapai waypoint saat ini"""
        if not self.waypoint_active or not self.waypoints:
            return
        
        if self.current_waypoint_index >= len(self.waypoints):
            return
        
        if not self.robot_pose or not self.goal_pose:
            return
        
        # Hitung jarak Euclidean antara robot dan waypoint
        current_waypoint = self.waypoints[self.current_waypoint_index]
        dx = current_waypoint[0] - self.robot_pose[0]
        dy = current_waypoint[1] - self.robot_pose[1]
        distance = math.sqrt(dx*dx + dy*dy)
        
        # Check threshold
        if distance <= self.waypoint_proximity_threshold:
            # Waypoint tercapai!
            self.get_logger().info(f"Waypoint {self.current_waypoint_index + 1} reached!")
            self.get_logger().info(f"Distance: {distance:.3f}m")
            
            # Process next waypoint
            self.process_next_waypoint()
    
    def publish_waypoint_status(self):
        """Publish status waypoint ke topic /robot1/waypoint_status"""
        status_msg = String()
        
        if not self.waypoint_mode:
            status_msg.data = "WAYPOINT_MODE: DISABLED"
        elif not self.waypoints:
            status_msg.data = "WAYPOINT_MODE: ENABLED - No waypoints set"
        elif self.current_waypoint_index >= len(self.waypoints):
            status_msg.data = f"WAYPOINT_MODE: COMPLETED - All {len(self.waypoints)} waypoints reached"
        else:
            status_msg.data = f"WAYPOINT_MODE: ACTIVE - Waypoint {self.current_waypoint_index + 1}/{len(self.waypoints)}"
        
        self.waypoint_status_pub.publish(status_msg)
        
        self.get_logger().debug(f"Published waypoint status: {status_msg.data}",
                               throttle_duration_sec=2.0)
    
    # Functions helper
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
    
    # Path Smoothing
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
            if i % 2 == 0 or i == len(simplified) - 1:  # Downsample
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
            if distance > 0.3:  # Tolerance in cells
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
    
    # publish
    def publish_path_callback(self):
        """Timer callback to publish path dan length"""
        # Skip publish jika goal sudah tercapai
        if self.goal_reached:
            return
        
        if self.path_valid and self.global_path:
            self.publish_path()
    
    def publish_path(self):
        """Publish global path ke /robot1/plan dan length ke /robot1/path_length"""
        if not self.global_path or len(self.global_path) < 2:
            return
        
        # Create Path message
        path_msg = Path()
        path_msg.header = Header()
        path_msg.header.stamp = self.get_clock().now().to_msg()
        path_msg.header.frame_id = "map"
        
        # Determine goal orientation (use final goal or current waypoint)
        target_pose = self.goal_pose if self.goal_pose else self.final_goal_pose
        
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
                yaw = target_pose[2] if target_pose else 0.0
            
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
        
        # Log dengan informasi
        waypoint_info = ""
        if self.waypoint_mode and self.waypoints:
            waypoint_info = f", Waypoints: {self.current_waypoint_index + 1}/{len(self.waypoints)}"
        
        # Log data saving status
        data_saving_info = ""
        if self.dijkstra_data_saving_enabled:
            data_saving_info = f", Data saved: {'YES' if self.dijkstra_data_saved else 'NO'}"
        
        self.get_logger().info(f"Published: {len(path_msg.poses)} poses, "
                              f"length: {self.path_length:.2f}m, "
                              f"traveled: {self.traveled_length:.2f}m, "
                              f"remaining: {self.remaining_length:.2f}m"
                              f"{waypoint_info}"
                              f"{data_saving_info}",
                              throttle_duration_sec=2.0)
    
    def publish_path_length(self):
        """Publish path length ke /robot1/path_length"""
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
        """Publish global costmap ke /robot1/global_costmap/costmap"""
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
        self.get_logger().debug(f"Published global costmap to /robot1/global_costmap/costmap",
                               throttle_duration_sec=5.0)
    
    def publish_performance_data(self):
        """Publish planning performance data"""
        perf_msg = Float32()
        perf_msg.data = float(self.dijkstra_time * 1000)  # Convert to ms
        self.performance_pub.publish(perf_msg)
    
    # Status Reporting
    def status_report(self):
        """Periodic status report"""
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
        
        # Waypoint status
        if self.waypoint_mode:
            if self.waypoints:
                if self.current_waypoint_index < len(self.waypoints):
                    status.append(f"WP: {self.current_waypoint_index + 1}/{len(self.waypoints)}")
                else:
                    status.append("WP: COMPLETED")
            else:
                status.append("WP: Mode ON (no waypoints)")
        else:
            status.append("WP: Mode OFF")
        
        if self.goal_received:
            if self.waypoint_active:
                status.append(f"Target: WP{self.current_waypoint_index + 1}")
            else:
                status.append(f"Goal: ({self.goal_pose[0]:.2f}, {self.goal_pose[1]:.2f})")
        else:
            status.append("Goal: Not set")
        
        if self.cmd_vel_received:
            status.append(f"Vel: {self.cmd_vel_linear:.2f}m/s")
        else:
            status.append("Vel: No cmd_vel")
        
        if self.dijkstra_data_saving_enabled:
            status.append(f"Data Save: {'YES' if self.dijkstra_data_saved else 'NO'}")
        
        if self.goal_reached:
            status.append("GOAL REACHED! ✓")
        elif self.path_valid and self.global_path:
            # Hitung progress
            progress = (self.traveled_length / self.path_length * 100) if self.path_length > 0 else 0
            
            status.append(f"Path: {len(self.global_path)} wp")
            status.append(f"Total: {self.path_length:.2f}m")
            status.append(f"Traveled: {self.traveled_length:.2f}m")
            status.append(f"Remaining: {self.remaining_length:.2f}m")
            status.append(f"Progress: {progress:.1f}%")
            status.append(f"Plans: {self.total_plans}")
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
        node.get_logger().info("GLOBAL PATH PLANNER - FINAL STATS")
        node.get_logger().info(f"Total Runtime: {time.time() - node.start_time:.1f}s")
        node.get_logger().info(f"Costmap Creation Time: {node.costmap_creation_time:.2f}s")
        node.get_logger().info(f"Total Path Computations: {node.total_plans}")
        node.get_logger().info(f"Max Dijkstra Nodes Explored: {node.dijkstra_explored_nodes}")
        
        # Data saving info
        if node.dijkstra_data_saving_enabled:
            node.get_logger().info(f"Dijkstra Data Saved: {'YES' if node.dijkstra_data_saved else 'NO'}")
            if node.dijkstra_data_saved:
                node.get_logger().info(f"Data Saved to: {node.dijkstra_save_path}")
        
        # Waypoint stats
        if node.waypoint_mode:
            node.get_logger().info(f"Waypoint Mode: ACTIVE")
            node.get_logger().info(f"Total Waypoints: {len(node.waypoints)}")
            node.get_logger().info(f"Completed Waypoints: {node.current_waypoint_index}")
            node.get_logger().info(f"Full Path Length: {node.path_length:.2f}m")
        else:
            node.get_logger().info(f"Waypoint Mode: INACTIVE")
        
        node.get_logger().info(f"Total Traveled Distance: {node.traveled_length:.2f}m")
        node.get_logger().info(f"Goal Status: {'REACHED' if node.goal_reached else 'NOT REACHED'}")
        node.get_logger().info("=" * 70)
        
        node.get_logger().info("Node stopped by user")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
