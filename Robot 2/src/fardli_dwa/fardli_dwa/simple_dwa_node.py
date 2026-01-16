#!/usr/bin/env python3
"""
HOLONOMIC DWA with HEADING ALIGNMENT and DYNAMIC PARAMETERS
WITH COSTMAP INTEGRATION for narrow corridor safety
Robot first aligns heading, then uses holonomic motion only when needed
FINAL ORIENTATION ONLY AFTER POSITION REACHED
"""

import rclpy
from rclpy.node import Node
import math
import numpy as np
from geometry_msgs.msg import Twist, PoseStamped, PoseWithCovarianceStamped
from nav_msgs.msg import Path, OccupancyGrid
from sensor_msgs.msg import LaserScan
import time

class HeadingAlignedDWATracker(Node):
    def __init__(self):
        super().__init__('heading_aligned_dwa_tracker')
        
        self.get_logger().info("🔄 Heading-Aligned Holonomic DWA - WITH COSTMAP INTEGRATION")
        
        # ========== DECLARE PARAMETERS ==========
        # Motion limits (now as ROS parameters)
        self.declare_parameter('max_vel_x', 0.2)  # Forward/backward
        self.declare_parameter('max_vel_y', 0.01)  # Sideways
        self.declare_parameter('max_rot_vel', 1.0)  # Rotation
        
        # Path tracking parameters
        self.declare_parameter('lookahead_distance', 0.6)
        self.declare_parameter('goal_tolerance', 0.1)
        self.declare_parameter('path_tracking_tolerance', 0.2)
        
        # Heading alignment parameters
        self.declare_parameter('heading_alignment_tolerance', 0.2)  # rad (~11.5°)
        self.declare_parameter('min_heading_error_for_sideways', 0.3)  # rad (~17°)
        self.declare_parameter('max_sideways_ratio', 0.5)  # Max sideways/forward ratio
        
        # Final orientation parameters
        self.declare_parameter('final_orientation_weight', 3.0)  # Weight for final orientation alignment
        self.declare_parameter('position_first', True)  # Reach position first, then orientation
        
        # DWA Parameters
        self.declare_parameter('prediction_time', 1.5)
        self.declare_parameter('dt', 0.1)
        self.declare_parameter('vx_samples', 9)     # Forward motion samples
        self.declare_parameter('vy_samples', 7)     # Sideways samples
        self.declare_parameter('w_samples', 11)     # Rotation samples
        
        # NEW: Costmap parameters
        self.declare_parameter('costmap_weight', 2.0)  # Weight for costmap penalty
        self.declare_parameter('costmap_threshold', 200)  # Threshold for considering as obstacle
        self.declare_parameter('safety_margin', 0.1)  # Safety margin from high-cost areas (meters)
        self.declare_parameter('use_costmap', False)  # Enable/disable costmap integration
        
        # Initial parameter values
        self.update_parameters()
        
        # Behavior states
        self.STATE_ALIGNING = 1
        self.STATE_TRACKING = 2
        self.STATE_APPROACHING = 3
        self.STATE_FINAL_ALIGNING = 4  # NEW: Final orientation alignment
        self.STATE_IDLE = 5
        
        # Local plan
        self.local_plan_size = 20
        
        # ========== STATE ==========
        self.robot_pose = [0.0, 0.0, 0.0]
        self.robot_vel = [0.0, 0.0, 0.0]
        self.laser_data = None
        self.current_state = self.STATE_ALIGNING
        
        # Path management
        self.global_path = []  # Now stores [x, y, theta]
        self.path_id = 0
        self.current_path_index = 0
        self.progress_along_segment = 0.0
        self.target_point = None
        self.target_heading = 0.0
        self.goal_orientation = 0.0  # Final goal orientation
        self.position_reached = False  # Flag untuk menandai posisi sudah tercapai
        
        # Final goal pose
        self.final_goal_pose = None  # [x, y, theta]
        
        # ========== COSTMAP STATE ==========
        self.costmap_data = None  # List of cost values
        self.costmap_resolution = 0.05  # meters per cell
        self.costmap_origin = [0.0, 0.0]  # [x, y] in meters
        self.costmap_width = 0  # cells
        self.costmap_height = 0  # cells
        self.costmap_received = False
        self.costmap_stamp = None
        
        # ========== SUBSCRIBERS ==========
        self.create_subscription(
            PoseWithCovarianceStamped,
            '/robot2/amcl_pose',
            self.pose_callback,
            10
        )
        
        self.create_subscription(
            Path,
            '/robot2/plan',
            self.path_callback,
            10
        )
        
        self.create_subscription(
            LaserScan,
            '/robot2/scan',
            self.scan_callback,
            10
        )
        
        # NEW: Costmap subscriber
        self.create_subscription(
            OccupancyGrid,
            '/robot2/global_costmap/costmap',
            self.costmap_callback,
            10
        )
        
        # ========== PUBLISHERS ==========
        self.cmd_pub = self.create_publisher(Twist, '/robot2/cmd_vel', 10)
        self.local_plan_pub = self.create_publisher(Path, '/robot2/local_plan', 10)
        
        # ========== TIMERS ==========
        self.create_timer(0.1, self.control_loop)
        self.create_timer(1.0, self.status_report)
        self.create_timer(2.0, self.update_parameters)  # Update parameters every 2 seconds
        
        self.get_logger().info("✅ Heading-Aligned DWA Ready - WITH COSTMAP INTEGRATION")
    
    def update_parameters(self):
        """Update all parameters from ROS parameter server"""
        # Motion limits
        self.max_vel_x = self.get_parameter('max_vel_x').value
        self.max_vel_y = self.get_parameter('max_vel_y').value
        self.max_rot_vel = self.get_parameter('max_rot_vel').value
        
        # Path tracking
        self.lookahead_distance = self.get_parameter('lookahead_distance').value
        self.goal_tolerance = self.get_parameter('goal_tolerance').value
        self.path_tracking_tolerance = self.get_parameter('path_tracking_tolerance').value
        
        # Heading alignment
        self.heading_alignment_tolerance = self.get_parameter('heading_alignment_tolerance').value
        self.min_heading_error_for_sideways = self.get_parameter('min_heading_error_for_sideways').value
        self.max_sideways_ratio = self.get_parameter('max_sideways_ratio').value
        
        # Final orientation parameters
        self.final_orientation_weight = self.get_parameter('final_orientation_weight').value
        self.position_first = self.get_parameter('position_first').value
        
        # DWA Parameters
        self.prediction_time = self.get_parameter('prediction_time').value
        self.dt = self.get_parameter('dt').value
        self.vx_samples = int(self.get_parameter('vx_samples').value)
        self.vy_samples = int(self.get_parameter('vy_samples').value)
        self.w_samples = int(self.get_parameter('w_samples').value)
        
        # NEW: Costmap parameters
        self.costmap_weight = self.get_parameter('costmap_weight').value
        self.costmap_threshold = self.get_parameter('costmap_threshold').value
        self.safety_margin = self.get_parameter('safety_margin').value
        self.use_costmap = self.get_parameter('use_costmap').value
        
        # Debug log when parameters change
        self.get_logger().debug(f"Updated params: vx={self.max_vel_x:.2f}, vy={self.max_vel_y:.2f}, w={self.max_rot_vel:.2f}, costmap_weight={self.costmap_weight:.1f}")
    
    # ========== COSTMAP FUNCTIONS ==========
    def costmap_callback(self, msg):
        """Receive global costmap from global planner"""
        self.costmap_data = list(msg.data)
        self.costmap_resolution = msg.info.resolution
        self.costmap_origin = [msg.info.origin.position.x, msg.info.origin.position.y]
        self.costmap_width = msg.info.width
        self.costmap_height = msg.info.height
        self.costmap_received = True
        self.costmap_stamp = msg.header.stamp
        
        if not hasattr(self, 'costmap_received_once'):
            self.costmap_received_once = True
            self.get_logger().info(f"📊 Costmap received: {self.costmap_width}x{self.costmap_height} cells, "
                                  f"resolution: {self.costmap_resolution:.3f}m/cell")
    
    def world_to_costmap(self, world_x, world_y):
        """Convert world coordinates to costmap grid indices"""
        if not self.costmap_received:
            return None, None
        
        grid_x = int((world_x - self.costmap_origin[0]) / self.costmap_resolution)
        grid_y = int((world_y - self.costmap_origin[1]) / self.costmap_resolution)
        
        # Check bounds
        if 0 <= grid_x < self.costmap_width and 0 <= grid_y < self.costmap_height:
            return grid_x, grid_y
        return None, None
    
    def get_cost_at_world(self, world_x, world_y):
        """Get cost value at world coordinates"""
        if not self.costmap_received or not self.costmap_data:
            return 0
        
        grid_x, grid_y = self.world_to_costmap(world_x, world_y)
        if grid_x is None or grid_y is None:
            return 100  # Assume obstacle outside costmap
        
        index = grid_y * self.costmap_width + grid_x
        if 0 <= index < len(self.costmap_data):
            return self.costmap_data[index]
        return 100  # Assume obstacle if out of bounds
    
    def check_costmap_collision(self, x, y, check_radius=0.1):
        """
        Check collision using costmap.
        Returns True if high-cost area is within check_radius.
        """
        if not self.costmap_received or not self.use_costmap:
            return False
        
        # Sample points around the robot position
        samples = 8
        for i in range(samples):
            angle = 2.0 * math.pi * i / samples
            sample_x = x + check_radius * math.cos(angle)
            sample_y = y + check_radius * math.sin(angle)
            
            cost = self.get_cost_at_world(sample_x, sample_y)
            if cost >= self.costmap_threshold:
                return True
        
        # Also check the center point
        center_cost = self.get_cost_at_world(x, y)
        if center_cost >= self.costmap_threshold:
            return True
        
        return False
    
    def calculate_trajectory_cost(self, trajectory):
        """
        Calculate total cost for a trajectory.
        Higher cost means trajectory goes through dangerous areas.
        """
        if not self.costmap_received or not self.use_costmap:
            return 0.0
        
        total_cost = 0.0
        sample_count = 0
        
        for point in trajectory:
            x, y, _ = point
            cost = self.get_cost_at_world(x, y)
            
            # Convert occupancy cost (0-100) to penalty value
            if cost >= 80:  # Lethal cost
                penalty = 100.0
            elif cost >= 50:  # High cost
                penalty = 10.0
            elif cost >= 30:  # Medium cost
                penalty = 3.0
            elif cost >= 10:  # Low cost
                penalty = 1.0
            else:  # Free space
                penalty = 0.0
            
            total_cost += penalty
            sample_count += 1
        
        if sample_count > 0:
            return total_cost / sample_count  # Average cost
        return 0.0
    
    # ========== UPDATED COLLISION CHECKING ==========
    def check_collision(self, x, y):
        """Collision check using BOTH laser AND costmap"""
        collision_detected = False
        
        # 1. Check laser scan (fast, for immediate obstacles)
        if self.laser_data:
            robot_x, robot_y, robot_theta = self.robot_pose
            
            dx = x - robot_x
            dy = y - robot_y
            
            cos_t = math.cos(-robot_theta)
            sin_t = math.sin(-robot_theta)
            
            point_x = dx * cos_t - dy * sin_t
            point_y = dx * sin_t + dy * cos_t
            
            point_dist = math.hypot(point_x, point_y)
            point_angle = math.atan2(point_y, point_x)
            
            if abs(point_angle) <= 1.57:  # ±90 degrees
                beam_idx = int((point_angle - self.laser_data.angle_min) /
                              self.laser_data.angle_increment)
                
                if 0 <= beam_idx < len(self.laser_data.ranges):
                    laser_dist = self.laser_data.ranges[beam_idx]
                    
                    if (self.laser_data.range_min < laser_dist < self.laser_data.range_max and
                        laser_dist < point_dist - 0.05):  # 5cm safety margin
                        collision_detected = True
        
        # 2. Check costmap (for known static obstacles in narrow spaces)
        if not collision_detected and self.costmap_received and self.use_costmap:
            # Check with safety margin
            collision_detected = self.check_costmap_collision(x, y, self.safety_margin)
        
        return collision_detected
    
    # ========== UPDATED DWA SCORING ==========
    def calculate_path_following_score(self, pred_x, pred_y, pred_theta,
                                      target_x, target_y,
                                      vx, vy, w,
                                      robot_x, robot_y, robot_theta,
                                      heading_error):
        """Score for path following WITH COSTMAP INTEGRATION"""
        
        # 1. Distance to target point
        dist_to_target = math.hypot(target_x - pred_x, target_y - pred_y)
        goal_score = -3.0 * dist_to_target
        
        # 2. Forward motion bias
        forward_score = 2.0 * max(0, vx) / self.max_vel_x
        
        # 3. Heading alignment to target point direction
        target_heading_error = abs(self.normalize_angle(self.target_heading - pred_theta))
        heading_score = -1.5 * target_heading_error
        
        # 4. Progress along path
        if len(self.global_path) > 0:
            final_x, final_y, _ = self.global_path[-1]
            current_dist = math.hypot(final_x - robot_x, final_y - robot_y)
            pred_dist = math.hypot(final_x - pred_x, final_y - pred_y)
            progress_score = 1.5 * max(0, current_dist - pred_dist)
        else:
            progress_score = 0.0
        
        # 5. Sideways penalty (reduced in wide spaces, but still penalized in narrow spaces)
        sideways_penalty = 0.8 * abs(vy) / self.max_vel_y
        
        # 6. Velocity direction vs target direction
        vel_angle = math.atan2(vy, vx) if abs(vx) > 0.01 or abs(vy) > 0.01 else 0.0
        target_angle_rel = self.normalize_angle(heading_error - vel_angle)
        direction_score = -0.5 * abs(target_angle_rel)
        
        # 7. Smoothness
        smooth_penalty = 0.1 * abs(vx - self.robot_vel[0]) + \
                        0.2 * abs(vy - self.robot_vel[1]) + \
                        0.1 * abs(w - self.robot_vel[2])
        
        # 8. NEW: Costmap penalty (CRITICAL for narrow corridors)
        costmap_penalty = 0.0
        if self.costmap_received and self.use_costmap:
            # Get cost at predicted position
            cost = self.get_cost_at_world(pred_x, pred_y)
            
            # Apply penalty based on cost value
            if cost >= 80:  # Lethal
                costmap_penalty = 1000.0  # Very high penalty
            elif cost >= 60:  # Inscribed radius
                costmap_penalty = 100.0
            elif cost >= 40:  # Possibly inscribed
                costmap_penalty = 30.0
            elif cost >= 20:  # Gradient area
                costmap_penalty = 5.0
            
            # Apply weight parameter
            costmap_penalty *= self.costmap_weight
        
        # 9. NEW: Penalize sideways motion in narrow spaces
        narrow_space_penalty = 0.0
        if self.costmap_received and self.use_costmap:
            # Check if we're in a narrow space by sampling left and right
            left_x = pred_x + 0.15 * math.cos(pred_theta + math.pi/2)
            left_y = pred_y + 0.15 * math.sin(pred_theta + math.pi/2)
            right_x = pred_x + 0.15 * math.cos(pred_theta - math.pi/2)
            right_y = pred_y + 0.15 * math.sin(pred_theta - math.pi/2)
            
            left_cost = self.get_cost_at_world(left_x, left_y)
            right_cost = self.get_cost_at_world(right_x, right_y)
            
            # If either side is high cost, we're in narrow space
            if left_cost >= 50 or right_cost >= 50:
                # Strongly penalize sideways velocity in narrow spaces
                narrow_space_penalty = 3.0 * abs(vy) / self.max_vel_y
        
        total_score = (goal_score + forward_score + heading_score + 
                      progress_score + direction_score - 
                      sideways_penalty - smooth_penalty -
                      costmap_penalty - narrow_space_penalty)
        
        return total_score
    
    # ========== UPDATED TRACKING CONTROL ==========
    def tracking_control(self):
        """DWA for path following WITH COSTMAP AWARENESS"""
        robot_x, robot_y, robot_theta = self.robot_pose
        target_x, target_y = self.target_point
        
        # Current velocities
        current_vx, current_vy, current_w = self.robot_vel
        
        # Dynamic window using current parameter values
        vx_min = max(-self.max_vel_x, current_vx - 0.1)
        vx_max = min(self.max_vel_x, current_vx + 0.1)
        vy_min = max(-self.max_vel_y, current_vy - 0.05)
        vy_max = min(self.max_vel_y, current_vy + 0.05)
        w_min = max(-self.max_rot_vel, current_w - 0.2)
        w_max = min(self.max_rot_vel, current_w + 0.2)
        
        # Generate samples using current parameter values
        vx_samples = np.linspace(vx_min, vx_max, self.vx_samples)
        vy_samples = np.linspace(vy_min, vy_max, self.vy_samples)
        w_samples = np.linspace(w_min, w_max, self.w_samples)
        
        best_score = -float('inf')
        best_vx, best_vy, best_w = 0.0, 0.0, 0.0
        
        # Calculate desired velocity direction (to target point)
        desired_dir = math.atan2(target_y - robot_y, target_x - robot_x)
        heading_error = self.normalize_angle(desired_dir - robot_theta)
        
        # NEW: Check if we're in narrow space using costmap
        in_narrow_space = False
        if self.costmap_received and self.use_costmap:
            # Sample left and right
            left_x = robot_x + 0.2 * math.cos(robot_theta + math.pi/2)
            left_y = robot_y + 0.2 * math.sin(robot_theta + math.pi/2)
            right_x = robot_x + 0.2 * math.cos(robot_theta - math.pi/2)
            right_y = robot_y + 0.2 * math.sin(robot_theta - math.pi/2)
            
            left_cost = self.get_cost_at_world(left_x, left_y)
            right_cost = self.get_cost_at_world(right_x, right_y)
            
            in_narrow_space = (left_cost >= 50 or right_cost >= 50)
        
        for vx in vx_samples:
            for vy in vy_samples:
                # STRONG BIAS: Prefer forward motion (vx > 0)
                if vx < 0 and abs(heading_error) < 1.0:  # Don't go backward if roughly aligned
                    continue
                
                # NEW: In narrow spaces, STRICTLY limit sideways motion
                if in_narrow_space:
                    if abs(vy) > 0.02:  # Very small sideways allowed in narrow spaces
                        continue
                # Original logic for open spaces
                elif abs(heading_error) < self.min_heading_error_for_sideways:
                    if abs(vy) > 0.05:  # Minimize sideways when well-aligned
                        continue
                
                for w in w_samples:
                    # Skip if rotating too fast while moving slowly
                    if (abs(vx) + abs(vy)) < 0.05 and abs(w) > 0.5:
                        continue
                    
                    # Predict
                    pred_x, pred_y, pred_theta = self.predict_pose(vx, vy, w)
                    
                    # Check collision (using BOTH laser and costmap)
                    if self.check_collision(pred_x, pred_y):
                        continue
                    
                    # NEW: Additional costmap check for predicted trajectory
                    if self.costmap_received and self.use_costmap:
                        # Get cost at predicted position
                        pred_cost = self.get_cost_at_world(pred_x, pred_y)
                        if pred_cost >= 80:  # Skip lethal areas completely
                            continue
                    
                    # Score this motion - WITH COSTMAP INTEGRATION
                    score = self.calculate_path_following_score(
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
        
        # NEW: If no good trajectory found due to costmap, try pure rotation
        if best_score < -1000:  # All trajectories have high cost
            self.get_logger().warn("High cost area detected, using conservative control")
            # Try to rotate in place to find better heading
            heading_error = self.normalize_angle(self.target_heading - robot_theta)
            kp = 1.0
            best_w = kp * heading_error
            best_w = max(-self.max_rot_vel * 0.5, min(self.max_rot_vel * 0.5, best_w))
            best_vx = 0.0
            best_vy = 0.0
        
        return best_vx, best_vy, best_w
    
    # ========== UPDATED STATUS REPORT ==========
    def status_report(self):
        if not self.global_path:
            self.get_logger().info("Status: No path")
            return
        
        robot_x, robot_y, robot_theta = self.robot_pose
        
        if self.final_goal_pose:
            goal_x, goal_y, goal_theta = self.final_goal_pose
            dist_to_goal = math.hypot(goal_x - robot_x, goal_y - robot_y)
            final_heading_error = abs(self.normalize_angle(goal_theta - robot_theta))
        else:
            goal_x, goal_y = self.global_path[-1][:2]
            dist_to_goal = math.hypot(goal_x - robot_x, goal_y - robot_y)
            final_heading_error = 0.0
        
        heading_error = abs(self.normalize_angle(self.target_heading - robot_theta))
        
        state_names = {
            self.STATE_ALIGNING: "ALIGN",
            self.STATE_TRACKING: "TRACK",
            self.STATE_APPROACHING: "APPROACH",
            self.STATE_FINAL_ALIGNING: "FINAL_ALIGN",
            self.STATE_IDLE: "IDLE"
        }
        
        # NEW: Costmap info
        costmap_info = ""
        if self.costmap_received and self.use_costmap:
            current_cost = self.get_cost_at_world(robot_x, robot_y)
            costmap_info = f"Cost: {current_cost}"
            
            # Check if in narrow space
            left_x = robot_x + 0.2 * math.cos(robot_theta + math.pi/2)
            left_y = robot_y + 0.2 * math.sin(robot_theta + math.pi/2)
            right_x = robot_x + 0.2 * math.cos(robot_theta - math.pi/2)
            right_y = robot_y + 0.2 * math.sin(robot_theta - math.pi/2)
            
            left_cost = self.get_cost_at_world(left_x, left_y)
            right_cost = self.get_cost_at_world(right_x, right_y)
            
            if left_cost >= 50 or right_cost >= 50:
                costmap_info += " [NARROW]"
        
        status = [
            f"State: {state_names.get(self.current_state, 'UNKNOWN')}",
            f"Goal dist: {dist_to_goal:.2f}m",
            f"Pos reached: {self.position_reached}",
            f"Heading err: {math.degrees(heading_error):.1f}°",
            f"Final orient err: {math.degrees(final_heading_error):.1f}°",
            f"Vel: [{self.robot_vel[0]:.2f}, {self.robot_vel[1]:.2f}, {self.robot_vel[2]:.2f}]",
            f"Costmap: {'ON' if self.costmap_received and self.use_costmap else 'OFF'}"
        ]
        
        if costmap_info:
            status.append(costmap_info)
        
        self.get_logger().info(" | ".join(status))

    # ========== ORIGINAL FUNCTIONS (unchanged except for modifications above) ==========
    def pose_callback(self, msg):
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        
        q = msg.pose.pose.orientation
        theta = math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                          1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        
        self.robot_pose = [x, y, theta]
    
    def path_callback(self, msg):
        """Handle new path - WITH ORIENTATION SUPPORT"""
        new_path = []
        
        for pose in msg.poses:
            x = pose.pose.position.x
            y = pose.pose.position.y
            
            # Extract orientation from path
            q = pose.pose.orientation
            theta = math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                             1.0 - 2.0 * (q.y * q.y + q.z * q.z))
            
            new_path.append([x, y, theta])
        
        if len(new_path) < 2:
            return
        
        self.get_logger().info(f"📈 New path: {len(new_path)} points")
        self.global_path = new_path
        self.path_id += 1
        
        # Store final goal pose and orientation
        if new_path:
            self.final_goal_pose = new_path[-1]
            self.goal_orientation = new_path[-1][2]
            self.position_reached = False  # Reset flag untuk path baru
            self.get_logger().info(f"🎯 Final goal: position [{self.final_goal_pose[0]:.2f}, {self.final_goal_pose[1]:.2f}], "
                                 f"orientation: {math.degrees(self.goal_orientation):.1f}°")
        
        # Reset tracking
        self.current_path_index = 0
        self.progress_along_segment = 0.0
        self.current_state = self.STATE_ALIGNING  # Start with alignment
        
        self.find_current_position_on_path()
    
    def find_current_position_on_path(self):
        """Find robot position on path"""
        if not self.global_path or len(self.global_path) < 2:
            return
        
        robot_x, robot_y, _ = self.robot_pose
        
        min_dist = float('inf')
        best_idx = 0
        best_t = 0.0
        
        for i in range(len(self.global_path) - 1):
            x1, y1, _ = self.global_path[i]
            x2, y2, _ = self.global_path[i + 1]
            
            _, dist, t = self.closest_point_on_segment(robot_x, robot_y, x1, y1, x2, y2)
            
            if dist < min_dist:
                min_dist = dist
                best_idx = i
                best_t = t
        
        if best_idx >= len(self.global_path) - 1:
            best_idx = len(self.global_path) - 2 if len(self.global_path) >= 2 else 0
            best_t = 0.0
        
        self.current_path_index = best_idx
        self.progress_along_segment = best_t
        
        # Snap to waypoint if close
        if best_t < 0.1:
            self.progress_along_segment = 0.0
        elif best_t > 0.9 and self.current_path_index < len(self.global_path) - 2:
            self.current_path_index += 1
            self.progress_along_segment = 0.0
    
    def scan_callback(self, msg):
        self.laser_data = msg
    
    def get_next_target_with_heading(self):
        """Get target point AND desired heading - SIMPLIFIED: focus on path following"""
        if not self.global_path or len(self.global_path) < 2:
            return None, 0.0
        
        # Check if we should use final orientation (only if position reached and in final alignment state)
        if (self.position_reached and 
            self.current_state == self.STATE_FINAL_ALIGNING and
            self.final_goal_pose):
            # Return current position with final orientation
            robot_x, robot_y, _ = self.robot_pose
            return [robot_x, robot_y], self.goal_orientation
        
        # NORMAL PATH FOLLOWING: Use segment orientation only
        start_idx = self.current_path_index
        start_progress = self.progress_along_segment
        lookahead_remaining = self.lookahead_distance
        current_idx = start_idx
        
        # Find target point
        if start_progress > 0 and current_idx < len(self.global_path) - 1:
            x1, y1, _ = self.global_path[current_idx]
            x2, y2, _ = self.global_path[current_idx + 1]
            
            segment_len = math.hypot(x2 - x1, y2 - y1)
            if segment_len == 0:
                current_idx += 1
                start_progress = 0.0
                start_x, start_y, _ = self.global_path[current_idx] if current_idx < len(self.global_path) else self.global_path[-1]
            else:
                start_x = x1 + start_progress * (x2 - x1)
                start_y = y1 + start_progress * (y2 - y1)
                
                remaining_on_segment = (1 - start_progress) * segment_len
                
                if remaining_on_segment >= lookahead_remaining:
                    ratio = lookahead_remaining / segment_len
                    target_x = start_x + ratio * (x2 - x1)
                    target_y = start_y + ratio * (y2 - y1)
                    # ALWAYS use segment heading during path following
                    target_heading = math.atan2(y2 - y1, x2 - x1)
                    return [target_x, target_y], target_heading
                else:
                    lookahead_remaining -= remaining_on_segment
                    current_idx += 1
                    
                    if current_idx >= len(self.global_path):
                        target_x, target_y, target_theta = self.global_path[-1]
                        # Near end of path, still use segment heading
                        if current_idx > 0 and current_idx < len(self.global_path):
                            prev_x, prev_y, _ = self.global_path[current_idx-1]
                            target_heading = math.atan2(target_y - prev_y, target_x - prev_x)
                        else:
                            target_heading = target_theta
                        return [target_x, target_y], target_heading
                    
                    start_x, start_y, _ = self.global_path[current_idx]
        else:
            if current_idx < len(self.global_path):
                start_x, start_y, _ = self.global_path[current_idx]
            else:
                target_x, target_y, target_theta = self.global_path[-1]
                # At end of path
                if len(self.global_path) > 1:
                    prev_x, prev_y, _ = self.global_path[-2]
                    target_heading = math.atan2(target_y - prev_y, target_x - prev_x)
                else:
                    target_heading = target_theta
                return [target_x, target_y], target_heading
        
        while current_idx < len(self.global_path) - 1 and lookahead_remaining > 0:
            x1, y1, _ = self.global_path[current_idx]
            x2, y2, _ = self.global_path[current_idx + 1]
            
            segment_len = math.hypot(x2 - x1, y2 - y1)
            if segment_len == 0:
                current_idx += 1
                continue
            
            if segment_len >= lookahead_remaining:
                ratio = lookahead_remaining / segment_len
                target_x = x1 + ratio * (x2 - x1)
                target_y = y1 + ratio * (y2 - y1)
                # ALWAYS use segment heading during path following
                target_heading = math.atan2(y2 - y1, x2 - x1)
                return [target_x, target_y], target_heading
            else:
                lookahead_remaining -= segment_len
                current_idx += 1
        
        # If we reach here, we're at or near the final point
        target_x, target_y, target_theta = self.global_path[-1]
        # Still use segment heading, not final orientation
        if len(self.global_path) > 1:
            prev_x, prev_y, _ = self.global_path[-2]
            target_heading = math.atan2(target_y - prev_y, target_x - prev_x)
        else:
            target_heading = target_theta
            
        return [target_x, target_y], target_heading
    
    def update_tracking_position(self):
        """Update path tracking position"""
        if not self.global_path or len(self.global_path) < 2:
            return
        
        robot_x, robot_y, _ = self.robot_pose
        
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
            
            _, dist, t = self.closest_point_on_segment(robot_x, robot_y, x1, y1, x2, y2)
            
            if dist < min_dist:
                min_dist = dist
                best_idx = i
                best_t = t
        
        if best_idx >= len(self.global_path) - 1:
            best_idx = len(self.global_path) - 2 if len(self.global_path) >= 2 else 0
            best_t = 0.0
        
        if min_dist < self.path_tracking_tolerance:
            self.current_path_index = best_idx
            self.progress_along_segment = best_t
            
            if best_t > 0.85 and self.current_path_index < len(self.global_path) - 2:
                self.current_path_index += 1
                self.progress_along_segment = 0.0
    
    def control_loop(self):
        if not self.laser_data:
            return
        
        if not self.global_path or len(self.global_path) < 2:
            self.stop_robot()
            return
        
        # Update tracking position
        self.update_tracking_position()
        
        # Get target and desired heading
        self.target_point, self.target_heading = self.get_next_target_with_heading()
        if not self.target_point:
            self.stop_robot()
            return
        
        # Check if position goal is reached
        robot_x, robot_y, robot_theta = self.robot_pose
        if self.final_goal_pose:
            goal_x, goal_y, _ = self.final_goal_pose
            dist_to_goal = math.hypot(goal_x - robot_x, goal_y - robot_y)
            
            if dist_to_goal < self.goal_tolerance and not self.position_reached:
                self.position_reached = True
                self.get_logger().info("📍 POSITION GOAL REACHED! STOPPING ROBOT...")
                
                self.get_logger().info("🎉 GOAL REACHED!")
                self.stop_robot()
                self.current_state = self.STATE_IDLE
                return
        self.determine_state()
        # Execute appropriate control
        if self.current_state == self.STATE_ALIGNING:
            vx, vy, w = self.heading_alignment_control()
        elif self.current_state == self.STATE_TRACKING:
            vx, vy, w = self.tracking_control()
        elif self.current_state == self.STATE_APPROACHING:
            vx, vy, w = self.approach_control()
        elif self.current_state == self.STATE_FINAL_ALIGNING:
            vx, vy, w = 0.0, 0.0, 0.0
        else:  # STATE_IDLE
            self.stop_robot()
            return
        
        # Apply command
        self.publish_command(vx, vy, w)
        
        # Publish local plan for visualization
        self.publish_local_plan(self.predict_trajectory(vx, vy, w))
    
    def determine_state(self):
        """Determine current behavior state - WITHOUT FINAL ALIGNING"""
        robot_x, robot_y, robot_theta = self.robot_pose
        target_x, target_y = self.target_point
        
        # Calculate errors
        heading_error = abs(self.normalize_angle(self.target_heading - robot_theta))
        dist_to_target = math.hypot(target_x - robot_x, target_y - robot_y)
        
        # Calculate distance to final goal
        if self.final_goal_pose:
            goal_x, goal_y, _ = self.final_goal_pose
            dist_to_goal = math.hypot(goal_x - robot_x, goal_y - robot_y)
        else:
            dist_to_goal = dist_to_target
        
        # 🚨 PERUBAHAN: State machine tanpa final aligning
        if dist_to_goal < self.goal_tolerance:
            # Langsung idle ketika posisi tercapai
            self.current_state = self.STATE_IDLE
        elif dist_to_goal < self.goal_tolerance * 1.5:  # Getting close to goal
            self.current_state = self.STATE_APPROACHING
        elif heading_error > self.heading_alignment_tolerance:
            self.current_state = self.STATE_ALIGNING
        else:
            self.current_state = self.STATE_TRACKING
        
        # Log state changes
        state_names = {
            self.STATE_ALIGNING: "ALIGNING",
            self.STATE_TRACKING: "TRACKING", 
            self.STATE_APPROACHING: "APPROACHING",
            self.STATE_FINAL_ALIGNING: "FINAL_ALIGNING",
            self.STATE_IDLE: "IDLE"
        }
        self.get_logger().debug(f"State: {state_names[self.current_state]}, "
                            f"Pos reached: {self.position_reached}")
        
    def heading_alignment_control(self):
        """Pure rotation to align with target heading"""
        robot_theta = self.robot_pose[2]
        heading_error = self.normalize_angle(self.target_heading - robot_theta)
        
        # P-control for rotation
        kp = 1.5
        w = kp * heading_error
        w = max(-self.max_rot_vel, min(self.max_rot_vel, w))
        
        # Damping near alignment
        if abs(heading_error) < 0.3:
            w *= 0.7
        
        # No translation during pure alignment
        return 0.0, 0.0, w
    
    def approach_control(self):
        """Final approach to position goal - still ignoring final orientation"""
        robot_x, robot_y, robot_theta = self.robot_pose
        target_x, target_y = self.target_point
        
        # Calculate direction to target
        dx = target_x - robot_x
        dy = target_y - robot_y
        dist = math.hypot(dx, dy)
        
        if dist < 0.05:  # Very close to target
            return 0.0, 0.0, 0.0
        
        # Convert to robot frame
        cos_t = math.cos(robot_theta)
        sin_t = math.sin(robot_theta)
        
        vx_local = dx * cos_t + dy * sin_t
        vy_local = -dx * sin_t + dy * cos_t
        
        # P-control with limits
        kp = 0.5
        vx = kp * vx_local
        vy = kp * vy_local
        
        # Very slow approach
        max_approach_speed = 0.1
        speed = math.hypot(vx, vy)
        if speed > max_approach_speed:
            vx *= max_approach_speed / speed
            vy *= max_approach_speed / speed
        
        # Minimal rotation during approach
        w = 0.0
        
        return vx, vy, w
    
    def final_alignment_control(self):
        """Align to final goal orientation - ONLY AFTER POSITION REACHED"""
        robot_x, robot_y, robot_theta = self.robot_pose
        
        # Check if we're still at the goal position
        if self.final_goal_pose:
            goal_x, goal_y, _ = self.final_goal_pose
            pos_error = math.hypot(goal_x - robot_x, goal_y - robot_y)
            
            if pos_error > self.goal_tolerance * 1.5:
                # Moved away from goal, go back to position first
                self.get_logger().warn("Moved away from goal position, returning...")
                self.position_reached = False
                self.current_state = self.STATE_APPROACHING
                return self.approach_control()
        
        # Calculate heading error to final orientation
        heading_error = self.normalize_angle(self.goal_orientation - robot_theta)
        
        # Check if already aligned
        if abs(heading_error) < self.heading_alignment_tolerance:
            return 0.0, 0.0, 0.0
        
        # P-control for rotation
        kp = 2.0  # Stronger control for final alignment
        w = kp * heading_error
        w = max(-self.max_rot_vel * 0.8, min(self.max_rot_vel * 0.8, w))
        
        # Smooth braking near alignment
        if abs(heading_error) < 0.5:
            w *= abs(heading_error) / 0.5
        
        # Small position correction if needed
        vx, vy = 0.0, 0.0
        if self.final_goal_pose:
            goal_x, goal_y, _ = self.final_goal_pose
            pos_error = math.hypot(goal_x - robot_x, goal_y - robot_y)
            
            if pos_error > self.goal_tolerance:
                # Small correction while rotating
                dx = goal_x - robot_x
                dy = goal_y - robot_y
                
                cos_t = math.cos(robot_theta)
                sin_t = math.sin(robot_theta)
                
                vx = 0.03 * (dx * cos_t + dy * sin_t)
                vy = 0.03 * (-dx * sin_t + dy * cos_t)
                
                # Limit correction speed
                corr_speed = math.hypot(vx, vy)
                if corr_speed > 0.03:
                    vx *= 0.03 / corr_speed
                    vy *= 0.03 / corr_speed
        
        return vx, vy, w
    
    def predict_pose(self, vx, vy, w):
        """Predict pose after prediction_time"""
        x, y, theta = self.robot_pose
        
        steps = int(self.prediction_time / self.dt)
        for _ in range(steps):
            theta += w * self.dt
            dx_world = vx * math.cos(theta) - vy * math.sin(theta)
            dy_world = vx * math.sin(theta) + vy * math.cos(theta)
            x += dx_world * self.dt
            y += dy_world * self.dt
        
        return x, y, theta
    
    def predict_trajectory(self, vx, vy, w):
        """Predict trajectory for visualization"""
        trajectory = []
        x, y, theta = self.robot_pose
        
        steps = int(self.prediction_time / self.dt)
        viz_step = max(1, steps // self.local_plan_size)
        
        for step in range(steps):
            theta += w * self.dt
            dx_world = vx * math.cos(theta) - vy * math.sin(theta)
            dy_world = vx * math.sin(theta) + vy * math.cos(theta)
            x += dx_world * self.dt
            y += dy_world * self.dt
            
            if step % viz_step == 0:
                trajectory.append([x, y, theta])
        
        if steps > 0 and (steps - 1) % viz_step != 0:
            trajectory.append([x, y, theta])
        
        return trajectory
    
    # ========== UTILITIES ==========
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
    
    def is_goal_reached(self):
        """Check if both position and orientation are reached"""
        if not self.global_path or not self.final_goal_pose:
            return False
        
        robot_x, robot_y, robot_theta = self.robot_pose
        goal_x, goal_y, goal_theta = self.final_goal_pose
        
        pos_error = math.hypot(robot_x - goal_x, robot_y - goal_y)
        orient_error = abs(self.normalize_angle(goal_theta - robot_theta))
        
        return (pos_error < self.goal_tolerance and 
                orient_error < self.heading_alignment_tolerance)
    
    def publish_command(self, vx, vy, w):
        twist = Twist()
        twist.linear.x = float(vx)
        twist.linear.y = float(vy)
        twist.angular.z = float(w)
        self.cmd_pub.publish(twist)
        
        self.robot_vel = [vx, vy, w]
    
    def publish_local_plan(self, trajectory):
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
    
    def stop_robot(self):
        twist = Twist()
        twist.linear.x = 0.0
        twist.linear.y = 0.0
        twist.angular.z = 0.0
        self.cmd_pub.publish(twist)
        
        self.robot_vel = [0.0, 0.0, 0.0]
        
        empty_path = Path()
        empty_path.header.stamp = self.get_clock().now().to_msg()
        empty_path.header.frame_id = 'map'
        self.local_plan_pub.publish(empty_path)
    
    def normalize_angle(self, angle):
        while angle > math.pi:
            angle -= 2 * math.pi
        while angle < -math.pi:
            angle += 2 * math.pi
        return angle

def main(args=None):
    rclpy.init(args=args)
    
    try:
        node = HeadingAlignedDWATracker()
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Stopped")
    finally:
        node.stop_robot()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()