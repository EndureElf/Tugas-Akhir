#!/usr/bin/env python3
"""
ROBOT2 FOLLOW CONTROLLER - HOLONOMIC COMPATIBLE V2 - COMPATIBLE VERSION
UPDATE: Menerima BOTH 'goal_command' dan 'sync_goal' types
"""

import rclpy
from rclpy.node import Node
import socket
import threading
import time
import json
from std_msgs.msg import Float32
from geometry_msgs.msg import PoseStamped
from rcl_interfaces.srv import SetParameters
from rcl_interfaces.msg import Parameter, ParameterValue
from rclpy.parameter import ParameterType

class Robot2FollowController(Node):
    def __init__(self):
        super().__init__('robot2_follow_controller')
        
        self.get_logger().info("=" * 60)
        self.get_logger().info("🤖 ROBOT2 FOLLOW CONTROLLER - COMPATIBLE VERSION")
        self.get_logger().info("   Menerima BOTH 'goal_command' dan 'sync_goal'")
        self.get_logger().info("=" * 60)
        
        # ========== NETWORK SETUP ==========
        self.robot1_ip = '10.42.0.246'  # IP Robot1
        self.robot1_port = 8892
        
        # UDP Sender
        self.udp_sender = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        
        # UDP Receiver
        self.udp_receiver = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.udp_receiver.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.udp_receiver.bind(('0.0.0.0', 8891))
        self.udp_receiver.settimeout(0.1)
        
        # ========== STATE VARIABLES ==========
        self.current_goal = None
        self.current_path_length = 0.0
        self.last_length_sent_time = 0
        self.length_send_interval = 0.5
        
        # Velocity state
        self.current_vel_x = 0.01
        self.current_vel_y = 0.01
        self.current_rot_vel = 0.01
        
        # Service call state
        self.service_available = False
        self.service_request_in_progress = False
        self.last_service_call_time = 0
        self.service_cooldown = 0.5  # Lebih panjang: 500ms
        
        # ========== ROS SERVICE CLIENT ==========
        self.dwa_param_client = None
        
        # ========== PUBLISHERS ==========
        self.goal_publisher = self.create_publisher(
            PoseStamped,
            '/robot2/goal_pose',
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
        
        # ========== TIMERS ==========
        self.create_timer(0.05, self.udp_receiver_listener)
        self.create_timer(1.0, self.send_status_update)
        self.create_timer(0.5, self.check_service_health)
        
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
        self.get_logger().info(f"📤 Sending path length to {self.robot1_ip}:{self.robot1_port}")
    
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
    
    # ========== PATH LENGTH HANDLING ==========
    def path_length_callback(self, msg):
        """Terima panjang path dari global_path.py, kirim ke Robot1"""
        current_time = time.time()
        
        # Limit sending frequency
        if current_time - self.last_length_sent_time < self.length_send_interval:
            return
        
        self.current_path_length = msg.data
        
        # Kirim ke Robot1
        self.send_path_length_to_robot1(msg.data)
        self.last_length_sent_time = current_time
        
        # Log dengan throttle
        if int(time.time()) % 5 == 0:
            self.get_logger().info(f"📏 Path length: {msg.data:.2f}m")
    
    def send_path_length_to_robot1(self, length):
        """Kirim PANJANG PATH saja ke Robot1"""
        try:
            data = {
                'type': 'path_length',
                'timestamp': time.time(),
                'length': float(length),
                'robot_id': 'robot2'
            }
            
            return self.send_to_robot1(data)
            
        except Exception as e:
            self.get_logger().error(f"Failed to send path length: {e}")
            return False
    
    # ========== UDP COMMUNICATION ==========
    def udp_receiver_listener(self):
        """Process incoming UDP messages dari Robot1"""
        try:
            data, addr = self.udp_receiver.recvfrom(65535)
            self.process_incoming_message(data, addr)
        except socket.timeout:
            pass
        except Exception as e:
            if "timed out" not in str(e):
                self.get_logger().error(f"UDP receiver error: {e}")
    
    def process_incoming_message(self, data, addr):
        """Process incoming UDP message dari Robot1 V4 - COMPATIBLE VERSION"""
        try:
            message_str = data.decode('utf-8', errors='ignore')
            
            if message_str.startswith('{'):
                message = json.loads(message_str)
                msg_type = message.get('type', '')
                
                # ========== CRITICAL FIX: ACCEPT BOTH TYPES ==========
                if msg_type == 'goal_command' or msg_type == 'sync_goal':
                    self.handle_goal_message(message, msg_type)
                elif msg_type == 'holonomic_velocity_command':
                    self.handle_holonomic_velocity_command(message)
                elif msg_type == 'velocity_command':
                    self.handle_velocity_command(message)
                elif msg_type == 'heartbeat':
                    self.handle_heartbeat(message)
                
        except json.JSONDecodeError:
            pass
        except Exception as e:
            self.get_logger().error(f"Message processing error: {e}")
    
    def handle_goal_message(self, message, msg_type):
        """Handle goal message - supports both 'goal_command' and 'sync_goal'"""
        try:
            # Untuk compatibility, coba ambil goal dari beberapa kemungkinan key
            goal_data = message.get('goal') or message.get('command') or {}
            
            if not goal_data:
                self.get_logger().warn(f"⚠️ No goal data in {msg_type} message")
                return
            
            # Create PoseStamped
            pose_msg = PoseStamped()
            pose_msg.header.stamp = self.get_clock().now().to_msg()
            pose_msg.header.frame_id = message.get('frame_id', 'map')
            
            # Set position - coba dari beberapa kemungkinan struktur
            if 'position' in goal_data:
                pos = goal_data.get('position', {})
                pose_msg.pose.position.x = pos.get('x', 0.0)
                pose_msg.pose.position.y = pos.get('y', 0.0)
                pose_msg.pose.position.z = pos.get('z', 0.0)
            elif 'x' in goal_data:  # Direct coordinates
                pose_msg.pose.position.x = goal_data.get('x', 0.0)
                pose_msg.pose.position.y = goal_data.get('y', 0.0)
                pose_msg.pose.position.z = goal_data.get('z', 0.0)
            
            # Set orientation
            if 'orientation' in goal_data:
                orient = goal_data.get('orientation', {})
                pose_msg.pose.orientation.x = orient.get('x', 0.0)
                pose_msg.pose.orientation.y = orient.get('y', 0.0)
                pose_msg.pose.orientation.z = orient.get('z', 0.0)
                pose_msg.pose.orientation.w = orient.get('w', 1.0)
            else:
                # Default orientation
                pose_msg.pose.orientation.w = 1.0
            
            # Publish goal
            self.goal_publisher.publish(pose_msg)
            self.current_goal = pose_msg
            
            self.get_logger().info(
                f"🎯 Received {msg_type}: ({pose_msg.pose.position.x:.2f}, {pose_msg.pose.position.y:.2f})"
            )
            
            # Kirim acknowledgment
            ack = {
                'type': 'goal_ack',
                'timestamp': time.time(),
                'status': 'received',
                'received_type': msg_type,
                'goal_position': {
                    'x': pose_msg.pose.position.x,
                    'y': pose_msg.pose.position.y
                }
            }
            self.send_to_robot1(ack)
            
        except Exception as e:
            self.get_logger().error(f"Failed to handle goal message ({msg_type}): {e}")
    
    def handle_holonomic_velocity_command(self, message):
        """Menerima dan menerapkan kecepatan holonomic dari Robot1 V4"""
        try:
            command = message.get('command', {})
            
            # Ekstrak parameter kecepatan holonomic
            max_vel_x = command.get('max_vel_x')
            max_vel_y = command.get('max_vel_y')
            max_rot_vel = command.get('max_rot_vel')
            target_speed = command.get('target_speed', 0.0)
            
            # Validasi nilai
            if max_vel_x is None or max_vel_y is None or max_rot_vel is None:
                self.get_logger().warn("⚠️ Invalid velocity command received")
                return
            
            self.get_logger().info(
                f"📥 Received velocity command: "
                f"vx={max_vel_x:.2f}, vy={max_vel_y:.2f}, w={max_rot_vel:.2f}"
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
            self.get_logger().error(f"Failed to handle velocity command: {e}")
    
    def execute_dwa_update_safe(self, max_vel_x, max_vel_y, max_rot_vel):
        """Thread-safe execution of DWA parameter update - SIMPLIFIED"""
        try:
            self.service_request_in_progress = True
            self.last_service_call_time = time.time()
            
            # SIMPLIFIED: Hanya update parameter utama
            request = SetParameters.Request()
            
            # Hanya parameter utama saja
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
            
            # Set callback
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
                            self.get_logger().info("✅ DWA parameters updated")
                        else:
                            self.get_logger().warn("⚠️ DWA update failed")
                except Exception as e:
                    self.get_logger().error(f"Error processing DWA response: {e}")
        except Exception as e:
            self.get_logger().error(f"Response handler error: {e}")
            self.service_request_in_progress = False
    
    def handle_velocity_command(self, message):
        """Menerima dan menerapkan kecepatan (legacy) dari Robot1"""
        try:
            command = message.get('command', {})
            
            max_vel_x = command.get('max_vel_x')
            max_vel_y = command.get('max_vel_y')
            max_rot_vel = command.get('max_rot_vel')
            
            if (max_vel_x is not None and max_vel_y is not None and 
                max_rot_vel is not None and self.service_available):
                
                self.execute_dwa_update_safe(max_vel_x, max_vel_y, max_rot_vel)
                
        except Exception as e:
            self.get_logger().error(f"Failed to handle velocity command: {e}")
    
    def handle_heartbeat(self, message):
        """Handle heartbeat dari Robot1"""
        # Simple acknowledgment
        pass
    
    def send_to_robot1(self, data):
        """Send data ke Robot1 via UDP"""
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
    
    # ========== STATUS REPORTING ==========
    def send_status_update(self):
        """Kirim status periodik ke Robot1"""
        try:
            status = {
                'type': 'robot2_status',
                'timestamp': time.time(),
                'has_goal': self.current_goal is not None,
                'path_length': self.current_path_length,
                'current_velocities': {
                    'max_vel_x': self.current_vel_x,
                    'max_vel_y': self.current_vel_y,
                    'max_rot_vel': self.current_rot_vel
                },
                'dwa_service_available': self.service_available
            }

            heartbeat = {
                'type': 'heartbeat',
                'timestamp': time.time(),
                'robot_id': 'robot2_follower',
                'status': 'active'
            }

            # Kirim status dan heartbeat
            self.send_to_robot1(status)
            self.send_to_robot1(heartbeat)
            
            # Log status
            if int(time.time()) % 5 == 0:
                self.get_logger().info(
                    f"📊 Status: Goal={self.current_goal is not None}, "
                    f"Path={self.current_path_length:.2f}m"
                )
                
        except Exception as e:
            self.get_logger().error(f"Status update failed: {e}")
    
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