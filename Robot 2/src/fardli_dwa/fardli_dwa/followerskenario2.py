#!/usr/bin/env python3
"""
ROBOT2 FOLLOW CONTROLLER - DUAL SEGMENT COMPATIBLE VERSION
UPDATE: Compatible dengan AdvancedMasterCoordinator v8
Menerima: 'goal_command', 'sync_goal', 'holonomic_velocity_command'
Mengirim: 'path_info' dengan total_path, waypoint_remaining, goal_remaining
"""

import rclpy
from rclpy.node import Node
import socket
import threading
import time
import json
from std_msgs.msg import Float32, String
from geometry_msgs.msg import PoseStamped
from rcl_interfaces.srv import SetParameters
from rcl_interfaces.msg import Parameter, ParameterValue
from rclpy.parameter import ParameterType

class Robot2FollowController(Node):
    def __init__(self):
        super().__init__('robot2_follow_controller')
        
        self.get_logger().info("=" * 60)
        self.get_logger().info("🤖 ROBOT2 FOLLOW CONTROLLER - DUAL SEGMENT VERSION")
        self.get_logger().info("   Compatible dengan AdvancedMaster v8")
        self.get_logger().info("=" * 60)
        
        # ========== NETWORK SETUP ==========
        self.robot1_ip = '192.168.0.91'  # IP Robot1 (Master)
        self.robot1_port = 8892
        
        # UDP Sender
        self.udp_sender = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        
        # UDP Receiver
        self.udp_receiver = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.udp_receiver.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.udp_receiver.bind(('0.0.0.0', 8891))
        self.udp_receiver.settimeout(0.1)
        
        # ========== DUAL SEGMENT STATE VARIABLES ==========
        # Goal states
        self.current_goal = None
        self.current_waypoint = None
        self.final_goal = None  # Untuk menyimpan goal akhir
        
        # Path information untuk dual segment
        self.current_path_length = 0.0          # Total path length
        self.waypoint_remaining = 0.0          # Jarak tersisa ke waypoint
        self.goal_remaining = 0.0              # Jarak tersisa ke goal
        self.waypoint_reached = False          # Flag waypoint sudah tercapai
        
        # Segment tracking
        self.current_segment = "waypoint"      # "waypoint" atau "goal"
        self.segment_start_time = None
        
        # Timing untuk pengiriman data
        self.last_info_sent_time = 0
        self.info_send_interval = 0.3          # Kirim info lebih sering
        
        # Velocity state
        self.current_vel_x = 0.01
        self.current_vel_y = 0.01
        self.current_rot_vel = 0.01
        
        # Service call state
        self.service_available = False
        self.service_request_in_progress = False
        self.last_service_call_time = 0
        self.service_cooldown = 0.5
        
        # Mission state
        self.mission_active = False
        self.time_remaining_total = 0.0
        self.time_remaining_to_waypoint = 0.0
        
        # ========== ROS SERVICE CLIENT ==========
        self.dwa_param_client = None
        
        # ========== PUBLISHERS ==========
        self.goal_publisher = self.create_publisher(
            PoseStamped,
            '/robot2/goal_pose',
            10
        )
        
        # Publisher untuk waypoint status (ke path planner)
        self.waypoint_status_pub = self.create_publisher(
            String,
            '/robot2/waypoint_status',
            10
        )
        
        # ========== SUBSCRIBERS ==========
        # Subscribe ke PATH LENGTH dari global_path.py
        self.create_subscription(
            Float32,
            '/robot2/path_length',
            self.path_length_callback,
            10
        )
        
        # Subscribe ke WAYPOINT REMAINING
        self.create_subscription(
            Float32,
            '/robot2/waypoint_remaining',
            self.waypoint_remaining_callback,
            10
        )
        
        # Subscribe ke GOAL REMAINING
        self.create_subscription(
            Float32,
            '/robot2/remaining_length',
            self.goal_remaining_callback,
            10
        )
        
        # ========== TIMERS ==========
        self.create_timer(0.05, self.udp_receiver_listener)
        self.create_timer(0.5, self.send_path_info_update)
        self.create_timer(0.5, self.check_service_health)
        self.create_timer(1.0, self.send_heartbeat)
        
        # ========== INITIALIZE ==========
        self.init_dwa_service_client()
        
        # Start UDP receiver thread
        self.receiver_thread = threading.Thread(
            target=self.udp_receiver_thread, 
            daemon=True
        )
        self.receiver_thread.start()
        
        self.get_logger().info("✅ Robot2 Follow Controller Ready!")
        self.get_logger().info(f"📡 Listening on port 8891")
        self.get_logger().info(f"📤 Sending dual segment info to {self.robot1_ip}:{self.robot1_port}")
    
    def init_dwa_service_client(self):
        """Initialize ROS service client untuk DWA holonomic"""
        try:
            self.dwa_param_client = self.create_client(
                SetParameters, 
                '/heading_aligned_dwa_tracker/set_parameters'
            )
            
            self.get_logger().info("🔄 Waiting for DWA holonomic parameter service...")
            
            # Check service connection
            self.create_timer(1.0, self.check_service_connection)
                
        except Exception as e:
            self.get_logger().error(f"Service client creation failed: {e}")
            self.service_available = False
    
    def check_service_connection(self):
        """Periodic check untuk service connection"""
        if self.dwa_param_client is None:
            return
            
        if not self.service_available:
            try:
                if self.dwa_param_client.wait_for_service(timeout_sec=0.5):
                    self.service_available = True
                    self.get_logger().info("✅ Connected to DWA Holonomic Parameter Service")
                else:
                    self.get_logger().debug("⏳ Waiting for DWA service...")
            except Exception as e:
                self.get_logger().debug(f"Service check: {e}")
    
    def check_service_health(self):
        """Periodic health check untuk service"""
        if self.dwa_param_client is None:
            return
            
        try:
            if not self.dwa_param_client.service_is_ready():
                self.service_available = False
                self.get_logger().warn("⚠️ DWA service disconnected")
        except Exception as e:
            self.get_logger().debug(f"Service health check error: {e}")
    
    # ========== PATH INFORMATION CALLBACKS ==========
    def path_length_callback(self, msg):
        """Total path length dari global_path.py"""
        self.current_path_length = msg.data
        
        # Log dengan throttle
        if int(time.time()) % 10 == 0:
            self.get_logger().debug(f"📏 Total path: {msg.data:.2f}m")
    
    def waypoint_remaining_callback(self, msg):
        """Jarak tersisa ke waypoint"""
        self.waypoint_remaining = msg.data
        
        # Cek jika waypoint sudah tercapai
        if not self.waypoint_reached and self.waypoint_remaining <= 0.5:  # Threshold 0.5m
            self.waypoint_reached = True
            
            # Publish status ke path planner
            status_msg = String()
            status_msg.data = "WAYPOINT_REACHED"
            self.waypoint_status_pub.publish(status_msg)
            
            self.get_logger().info("✅ Waypoint reached! Publishing status...")
            
            # PERBAIKAN: Switch ke final goal jika ada
            if self.final_goal and self.current_segment == "waypoint":
                self.switch_to_final_goal()
        
        # Log dengan throttle
        if int(time.time()) % 10 == 0:
            self.get_logger().debug(f"📍 Waypoint remaining: {msg.data:.2f}m")
    
    def goal_remaining_callback(self, msg):
        """Jarak tersisa ke goal"""
        self.goal_remaining = msg.data
        
        # Log dengan throttle
        if int(time.time()) % 10 == 0:
            self.get_logger().debug(f"🎯 Goal remaining: {msg.data:.2f}m")
    
    def send_path_info_update(self):
        """Kirim semua informasi path ke Master"""
        current_time = time.time()
        
        # Limit sending frequency
        if current_time - self.last_info_sent_time < self.info_send_interval:
            return
        
        try:
            # Buat message dengan semua info path
            path_info = {
                'type': 'path_info',
                'timestamp': current_time,
                'robot_id': 'robot2',
                'total_path': float(self.current_path_length),
                'waypoint_remaining': float(self.waypoint_remaining),
                'goal_remaining': float(self.goal_remaining),
                'waypoint_reached': bool(self.waypoint_reached),
                'current_segment': str(self.current_segment),
                'mission_active': bool(self.mission_active)
            }
            
            # Tambahkan timing info jika ada
            if self.time_remaining_total > 0:
                path_info['time_remaining_total'] = float(self.time_remaining_total)
            if self.time_remaining_to_waypoint > 0:
                path_info['time_remaining_to_waypoint'] = float(self.time_remaining_to_waypoint)
            
            # Kirim ke Master
            if self.send_to_robot1(path_info):
                self.last_info_sent_time = current_time
                
                # Log dengan throttle
                if int(time.time()) % 5 == 0:
                    self.get_logger().info(
                        f"📤 Path info sent: "
                        f"WP={self.waypoint_remaining:.2f}m, "
                        f"Goal={self.goal_remaining:.2f}m, "
                        f"Segment={self.current_segment}"
                    )
            
        except Exception as e:
            self.get_logger().error(f"Failed to send path info: {e}")
    
    # ========== UDP COMMUNICATION ==========
    def udp_receiver_listener(self):
        """Process incoming UDP messages dari Master"""
        try:
            data, addr = self.udp_receiver.recvfrom(65535)
            self.process_incoming_message(data, addr)
        except socket.timeout:
            pass
        except Exception as e:
            if "timed out" not in str(e):
                self.get_logger().error(f"UDP receiver error: {e}")
    
    def process_incoming_message(self, data, addr):
        """Process incoming UDP message dari Master v8"""
        try:
            message_str = data.decode('utf-8', errors='ignore')
            
            if message_str.startswith('{'):
                message = json.loads(message_str)
                msg_type = message.get('type', '')
                
                # ========== PROCESS MESSAGES FROM MASTER ==========
                if msg_type == 'goal_command':
                    self.handle_goal_message(message, msg_type)
                elif msg_type == 'sync_goal':
                    self.handle_goal_message(message, msg_type)
                elif msg_type == 'holonomic_velocity_command':
                    self.handle_holonomic_velocity_command_v8(message)
                elif msg_type == 'velocity_command':
                    self.handle_velocity_command(message)
                elif msg_type == 'heartbeat':
                    self.handle_heartbeat_v8(message)
                else:
                    self.get_logger().debug(f"Received unknown message type: {msg_type}")
                
        except json.JSONDecodeError:
            self.get_logger().warn("⚠️ Received non-JSON message")
        except Exception as e:
            self.get_logger().error(f"Message processing error: {e}")
    
    def handle_goal_message(self, message, msg_type):
        """Handle goal message - supports both 'goal_command' dan 'sync_goal'"""
        try:
            # Ekstrak goal data
            goal_data = message.get('goal') or message.get('command') or {}
            
            if not goal_data:
                self.get_logger().warn(f"⚠️ No goal data in {msg_type} message")
                return
            
            # ========== 1. CHECK FOR WAYPOINT ==========
            has_waypoint = 'waypoint' in message
            waypoint_data = message.get('waypoint', {})
            
            if has_waypoint:
                self.get_logger().info("📍 Dual segment mission detected (waypoint + goal)")
                
                # Simpan waypoint position
                if 'position' in waypoint_data:
                    wp_pos = waypoint_data.get('position', {})
                    waypoint_x = wp_pos.get('x', 0.0)
                    waypoint_y = wp_pos.get('y', 0.0)
                    
                    self.get_logger().info(f"   Waypoint: ({waypoint_x:.2f}, {waypoint_y:.2f})")
                    self.current_waypoint = {'x': waypoint_x, 'y': waypoint_y}
                else:
                    self.get_logger().warn("⚠️ Waypoint data incomplete")
            else:
                self.get_logger().info("🎯 Single goal mission")
                self.current_waypoint = None
            
            # ========== 2. SIMPAN FINAL GOAL ==========
            # Simpan goal akhir untuk digunakan nanti
            if 'position' in goal_data:
                pos = goal_data.get('position', {})
                self.final_goal = {
                    'x': pos.get('x', 0.0),
                    'y': pos.get('y', 0.0),
                    'z': pos.get('z', 0.0)
                }
                self.get_logger().info(f"   Final goal saved: ({self.final_goal['x']:.2f}, {self.final_goal['y']:.2f})")
            
            # ========== 3. RESET SEGMENT STATE ==========
            self.current_segment = "waypoint" if has_waypoint else "goal"
            self.waypoint_reached = False
            self.mission_active = True
            self.segment_start_time = time.time()
            
            # ========== 4. PUBLISH BOTH WAYPOINT AND GOAL SIMULTANEOUSLY ==========
            # PERBAIKAN: Kirim BOTH waypoint dan goal ke global path planner
            
            goal_published = False
            
            # Publish waypoint terlebih dahulu jika ada
            if has_waypoint and 'position' in waypoint_data:
                # Buat pose untuk waypoint
                wp_pose_msg = PoseStamped()
                wp_pose_msg.header.stamp = self.get_clock().now().to_msg()
                wp_pose_msg.header.frame_id = message.get('frame_id', 'map')
                
                wp_pos = waypoint_data.get('position', {})
                wp_pose_msg.pose.position.x = wp_pos.get('x', 0.0)
                wp_pose_msg.pose.position.y = wp_pos.get('y', 0.0)
                wp_pose_msg.pose.position.z = wp_pos.get('z', 0.0)
                
                # Set orientation untuk waypoint
                if 'orientation' in waypoint_data:
                    wp_orient = waypoint_data.get('orientation', {})
                    wp_pose_msg.pose.orientation.x = wp_orient.get('x', 0.0)
                    wp_pose_msg.pose.orientation.y = wp_orient.get('y', 0.0)
                    wp_pose_msg.pose.orientation.z = wp_orient.get('z', 0.0)
                    wp_pose_msg.pose.orientation.w = wp_orient.get('w', 1.0)
                else:
                    wp_pose_msg.pose.orientation.w = 1.0
                
                # Publish waypoint ke topic yang sama (global path akan bedakan)
                self.goal_publisher.publish(wp_pose_msg)
                self.get_logger().info(f"📤 Published WAYPOINT to path planner: ({wp_pose_msg.pose.position.x:.2f}, {wp_pose_msg.pose.position.y:.2f})")
                self.current_goal = wp_pose_msg
                
                # Delay kecil untuk memastikan waypoint diproses sebelum goal
                time.sleep(0.1)  # PERBAIKAN: Tambah delay
            
            # Publish final goal (selalu publish, baik single atau dual segment)
            if self.final_goal:
                goal_pose_msg = PoseStamped()
                goal_pose_msg.header.stamp = self.get_clock().now().to_msg()
                goal_pose_msg.header.frame_id = message.get('frame_id', 'map')
                
                goal_pose_msg.pose.position.x = self.final_goal['x']
                goal_pose_msg.pose.position.y = self.final_goal['y']
                goal_pose_msg.pose.position.z = self.final_goal['z']
                
                # Set orientation untuk goal
                if 'orientation' in goal_data:
                    goal_orient = goal_data.get('orientation', {})
                    goal_pose_msg.pose.orientation.x = goal_orient.get('x', 0.0)
                    goal_pose_msg.pose.orientation.y = goal_orient.get('y', 0.0)
                    goal_pose_msg.pose.orientation.z = goal_orient.get('z', 0.0)
                    goal_pose_msg.pose.orientation.w = goal_orient.get('w', 1.0)
                else:
                    goal_pose_msg.pose.orientation.w = 1.0
                
                # Publish final goal ke topic yang sama (global path akan tahu ini goal)
                self.goal_publisher.publish(goal_pose_msg)
                goal_published = True
                self.get_logger().info(f"📤 Published FINAL GOAL to path planner: ({goal_pose_msg.pose.position.x:.2f}, {goal_pose_msg.pose.position.y:.2f})")
                
                # Simpan sebagai current_goal jika waypoint tidak ada
                if not has_waypoint:
                    self.current_goal = goal_pose_msg
            
            # Log lengkap
            if has_waypoint and goal_published:
                self.get_logger().info(
                    f"✅ Published BOTH waypoint and goal to path planner (delay 0.1s)"
                )
            
            self.get_logger().info(
                f"🎯 Received {msg_type}: "
                f"[Segment: {self.current_segment}]"
            )
            
            # ========== 5. SEND ACKNOWLEDGMENT ==========
            ack = {
                'type': 'goal_ack',
                'timestamp': time.time(),
                'status': 'received',
                'received_type': msg_type,
                'robot_id': 'robot2',
                'has_waypoint': has_waypoint,
                'current_segment': self.current_segment,
                'goal_position': {
                    'x': self.final_goal['x'] if self.final_goal else 0.0,
                    'y': self.final_goal['y'] if self.final_goal else 0.0
                }
            }
            
            # Tambahkan waypoint info jika ada
            if has_waypoint and self.current_waypoint:
                ack['waypoint_position'] = {
                    'x': self.current_waypoint['x'],
                    'y': self.current_waypoint['y']
                }
            
            self.send_to_robot1(ack)
            
        except Exception as e:
            self.get_logger().error(f"Failed to handle goal message ({msg_type}): {e}")
    def switch_to_final_goal(self):
        """Switch dari waypoint ke final goal"""
        if not self.final_goal:
            self.get_logger().error("❌ Cannot switch to final goal: no final goal stored")
            return
        
        self.get_logger().info("🔄 Switching from waypoint to final goal")
        
        # Buat pose untuk final goal
        pose_msg = PoseStamped()
        pose_msg.header.stamp = self.get_clock().now().to_msg()
        pose_msg.header.frame_id = 'map'
        
        pose_msg.pose.position.x = self.final_goal['x']
        pose_msg.pose.position.y = self.final_goal['y']
        pose_msg.pose.position.z = self.final_goal['z']
        pose_msg.pose.orientation.w = 1.0  # Default orientation
        
        # Publish goal baru
        self.goal_publisher.publish(pose_msg)
        self.current_goal = pose_msg
        
        # Update segment state
        self.current_segment = "goal"
        
        self.get_logger().info(
            f"📤 Published FINAL GOAL: ({pose_msg.pose.position.x:.2f}, {pose_msg.pose.position.y:.2f})"
        )
    
    def handle_holonomic_velocity_command_v8(self, message):
        """Menerima dan menerapkan kecepatan holonomic dari Master v8 (dengan segment info)"""
        try:
            command = message.get('command', {})
            segment = message.get('segment', 'waypoint')
            current_segment = message.get('current_segment', 'waypoint')
            
            # Ekstrak parameter kecepatan
            max_vel_x = command.get('max_vel_x', self.current_vel_x)
            max_vel_y = command.get('max_vel_y', self.current_vel_y)
            max_rot_vel = command.get('max_rot_vel', self.current_rot_vel)
            target_speed = command.get('target_speed', 0.0)
            
            # Update timing info dari master
            self.time_remaining_total = command.get('time_remaining_total', 0.0)
            self.time_remaining_to_waypoint = command.get('time_remaining_to_waypoint', 0.0)
            
            # Update segment info
            if current_segment != self.current_segment:
                self.current_segment = current_segment
                self.get_logger().info(f"🔄 Segment changed to: {current_segment}")
            
            self.get_logger().info(
                f"📥 {segment.upper()} command: "
                f"vx={max_vel_x:.2f}m/s, vy={max_vel_y:.2f}m/s, "
                f"segment={current_segment}, "
                f"time_total={self.time_remaining_total:.1f}s"
            )
            
            # Update state untuk reporting
            self.current_vel_x = max_vel_x
            self.current_vel_y = max_vel_y
            self.current_rot_vel = max_rot_vel
            
            # Update DWA parameters jika service tersedia
            if self.service_available:
                # Cek cooldown
                current_time = time.time()
                if current_time - self.last_service_call_time < self.service_cooldown:
                    self.get_logger().debug("⏳ Service call cooldown, skipping...")
                    return
                
                # Cek apakah sedang ada request yang berjalan
                if self.service_request_in_progress:
                    self.get_logger().debug("⚠️ Service request already in progress, skipping...")
                    return
                
                # Jalankan update
                self.execute_dwa_update_safe(max_vel_x, max_vel_y, max_rot_vel)
            else:
                self.get_logger().warn("⚠️ Service not available, cannot update DWA parameters")
                
        except Exception as e:
            self.get_logger().error(f"Failed to handle velocity command v8: {e}")
    
    def handle_heartbeat_v8(self, message):
        """Handle heartbeat dari Master v8 (dengan segment info)"""
        try:
            mission_active = message.get('mission_active', False)
            current_segment = message.get('current_segment', 'waypoint')
            segment_changes = message.get('segment_changes', 0)
            
            if not mission_active and self.mission_active:
                self.get_logger().info("🛑 Mission ended by master")
                self.mission_active = False
            
            # PERBAIKAN: Jika master sudah switch ke goal, follower juga harus switch
            if (current_segment == "goal" and 
                self.current_segment == "waypoint" and 
                self.waypoint_reached and 
                self.final_goal):
                
                self.get_logger().info("🔄 Master switched to goal segment, switching follower...")
                self.switch_to_final_goal()
            
            # Update segment info
            self.current_segment = current_segment
            
        except Exception as e:
            self.get_logger().debug(f"Heartbeat processing error: {e}")
    
    def handle_velocity_command(self, message):
        """Menerima dan menerapkan kecepatan (legacy)"""
        try:
            command = message.get('command', {})
            
            max_vel_x = command.get('max_vel_x', self.current_vel_x)
            max_vel_y = command.get('max_vel_y', self.current_vel_y)
            max_rot_vel = command.get('max_rot_vel', self.current_rot_vel)
            
            if self.service_available:
                self.execute_dwa_update_safe(max_vel_x, max_vel_y, max_rot_vel)
                
        except Exception as e:
            self.get_logger().error(f"Failed to handle legacy velocity command: {e}")
    
    def execute_dwa_update_safe(self, max_vel_x, max_vel_y, max_rot_vel):
        """Thread-safe execution of DWA parameter update"""
        try:
            self.service_request_in_progress = True
            self.last_service_call_time = time.time()
            
            request = SetParameters.Request()
            
            # Parameter utama
            parameters = [
                ('max_vel_x', max_vel_x),
                ('max_vel_y', max_vel_y),
                ('max_rot_vel', max_rot_vel),
            ]
            
            for name, value in parameters:
                request.parameters.append(
                    Parameter(
                        name=name,
                        value=ParameterValue(
                            type=ParameterType.PARAMETER_DOUBLE, 
                            double_value=float(value)
                        )
                    )
                )
            
            # Async call
            future = self.dwa_param_client.call_async(request)
            future.add_done_callback(self.handle_dwa_update_response)
            
        except Exception as e:
            self.get_logger().error(f"DWA update setup failed: {e}")
            self.service_request_in_progress = False
    
    def handle_dwa_update_response(self, future):
        """Handle response dari DWA service call"""
        try:
            self.service_request_in_progress = False
            
            if future.done():
                try:
                    result = future.result()
                    if result and hasattr(result, 'results'):
                        success = all(r.successful for r in result.results)
                        if success:
                            self.get_logger().debug("✅ DWA parameters updated")
                        else:
                            self.get_logger().warn("⚠️ DWA update failed")
                except Exception as e:
                    self.get_logger().error(f"Error processing DWA response: {e}")
        except Exception as e:
            self.get_logger().error(f"Response handler error: {e}")
            self.service_request_in_progress = False
    
    def send_to_robot1(self, data):
        """Send data ke Master (Robot1) via UDP"""
        try:
            if isinstance(data, dict):
                data_str = json.dumps(data)
            else:
                data_str = str(data)
            
            self.udp_sender.sendto(
                data_str.encode('utf-8'),
                (self.robot1_ip, self.robot1_port)
            )
            
            return True
            
        except Exception as e:
            self.get_logger().error(f"Failed to send to Robot1: {e}")
            return False
    
    def send_heartbeat(self):
        """Kirim heartbeat ke Master"""
        try:
            heartbeat = {
                'type': 'heartbeat',
                'timestamp': time.time(),
                'robot_id': 'robot2_follower',
                'status': 'active',
                'mission_active': self.mission_active,
                'current_segment': self.current_segment,
                'waypoint_reached': self.waypoint_reached,
                'dwa_service_available': self.service_available
            }
            
            self.send_to_robot1(heartbeat)
            
        except Exception as e:
            self.get_logger().debug(f"Heartbeat send error: {e}")
    
    def udp_receiver_thread(self):
        """Background thread untuk UDP listening"""
        while rclpy.ok():
            try:
                data, addr = self.udp_receiver.recvfrom(65535)
                self.process_incoming_message(data, addr)
            except socket.timeout:
                continue
            except Exception as e:
                self.get_logger().error(f"UDP thread error: {e}")
                time.sleep(0.05)

def main(args=None):
    rclpy.init(args=args)
    node = None
    
    try:
        node = Robot2FollowController()
        rclpy.spin(node)
    except KeyboardInterrupt:
        if node:
            node.get_logger().info("\n" + "="*60)
            node.get_logger().info("🛑 Robot2 Follow Controller Stopped")
            node.get_logger().info("="*60)
    except Exception as e:
        if node:
            node.get_logger().error(f"Error: {e}")
    finally:
        if node:
            node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()