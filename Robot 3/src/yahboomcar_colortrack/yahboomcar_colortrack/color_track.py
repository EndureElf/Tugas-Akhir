import rclpy
from rclpy.node import Node

from std_msgs.msg import Bool
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist

from yahboomcar_msgs.msg import Position
from yahboomcar_colortrack.astra_common import *

from cv_bridge import CvBridge

class Color_Track(Node):
    def __init__(self, name):
        super().__init__(name)
        self.pub_cmdVel = self.create_publisher(Twist,'/cmd_vel',10)

        self.sub_depth = self.create_subscription(Image, "/camera/depth/image_raw", self.DepthCallback, 1)
        self.sub_position = self.create_subscription(Position, "/Current_point", self.PositionCallback, 1)
        self.sub_JoyState = self.create_subscription(Bool,'/JoyState',  self.JoyStateCallback,1)

        self.Center_x = 0
        self.Center_y = 0
        self.Center_r = 0
        
        self.bridge = CvBridge()
        self.Joy_active = False

        self.declare_param()

    def declare_param(self):
        self.declare_parameter("linear_Kp",3.0)
        self.linear_Kp = self.get_parameter('linear_Kp').get_parameter_value().double_value
        self.declare_parameter("linear_Ki",0.0)
        self.linear_Ki = self.get_parameter('linear_Ki').get_parameter_value().double_value
        self.declare_parameter("linear_Kd",2.0)
        self.linear_Kd = self.get_parameter('linear_Kd').get_parameter_value().double_value
        self.declare_parameter("angular_Kp",0.5)
        self.angular_Kp = self.get_parameter('angular_Kp').get_parameter_value().double_value
        self.declare_parameter("angular_Ki",0.0)
        self.angular_Ki = self.get_parameter('angular_Ki').get_parameter_value().double_value
        self.declare_parameter("angular_Kd",2.0)
        self.angular_Kd = self.get_parameter('angular_Kd').get_parameter_value().double_value
        self.declare_parameter("minDistance",1.0)
        self.minDistance = self.get_parameter('minDistance').get_parameter_value().double_value
        self.minDist = self.minDistance * 1000.0
        self.linear_pid = simplePID(self.linear_Kp, self.linear_Ki, self.linear_Kd)
        self.angular_pid = simplePID(self.angular_Kp, self.angular_Ki, self.angular_Kd)
        
    def get_param(self):
        self.linear_Kp = self.get_parameter('linear_Kp').get_parameter_value().double_value
        self.linear_Ki = self.get_parameter('linear_Ki').get_parameter_value().double_value
        self.linear_Kd = self.get_parameter('linear_Kd').get_parameter_value().double_value
        self.angular_Kp = self.get_parameter('angular_Kp').get_parameter_value().double_value
        self.angular_Ki = self.get_parameter('angular_Ki').get_parameter_value().double_value
        self.angular_Kd = self.get_parameter('angular_Kd').get_parameter_value().double_value
        self.minDistance = self.get_parameter('minDistance').get_parameter_value().double_value
        self.linear_pid = simplePID(self.linear_Kp, self.linear_Ki, self.linear_Kd)
        self.angular_pid = simplePID(self.angular_Kp, self.angular_Ki, self.angular_Kd)
        self.minDist = self.minDistance * 1000.0

    def DepthCallback(self, msg):
        depthFrame = self.bridge.imgmsg_to_cv2(msg, "16UC1")
        if self.Center_r != 0:
            distance = [0, 0, 0, 0, 0]
            if 0 < int(self.Center_y - 3) and int(self.Center_y + 3) < 480 and 0 < int(
                self.Center_x - 3) and int(self.Center_x + 3) < 640:
                distance[0] = depthFrame[int(self.Center_y - 3)][int(self.Center_x - 3)]
                distance[1] = depthFrame[int(self.Center_y + 3)][int(self.Center_x - 3)]
                distance[2] = depthFrame[int(self.Center_y - 3)][int(self.Center_x + 3)]
                distance[3] = depthFrame[int(self.Center_y + 3)][int(self.Center_x + 3)]
                distance[4] = depthFrame[int(self.Center_y)][int(self.Center_x)]
                # print("distance: ", distance)
                num_depth_points = 5
                dist = 0
                for i in range(5):
                    if 40 < distance[i] < 8000: dist += distance[i]
                    else: num_depth_points -= 1
                if num_depth_points == 0: dist = self.minDist
                else: dist /= num_depth_points
                self.execute(self.Center_x, dist)

    def execute(self, point_x, dist):
        if self.Joy_active: return
        self.get_param()
        linear_x = self.linear_pid.compute(dist, self.minDist) / 1000.0
        angular_z = self.angular_pid.compute(320, point_x) / 1000.0
        if angular_z > 1.0: angular_z = 1.0
        if angular_z < -1.0: angular_z = -1.0
        if linear_x > 0.3: linear_x = 0.3
        if linear_x < -0.3: linear_x = -0.3
        if abs(dist - self.minDist) < 30: linear_x = 0
        if abs(point_x - 320.0) < 30: angular_z = 0
        twist = Twist()
        twist.linear.x = linear_x * 1.0
        twist.angular.z = angular_z * 1.0
        print("twist.linear.x: ",twist.linear.x)
        print("twist.angular.z: ",twist.angular.z)
        self.pub_cmdVel.publish(twist)

    def PositionCallback(self, msg):
        self.Center_x = msg.anglex
        self.Center_y = msg.angley
        self.Center_r = msg.distance
    
    def JoyStateCallback(self, msg):
        self.Joy_active = msg.data
        self.pub_cmdVel.publish(Twist())


def main():
    rclpy.init()
    color_track = Color_Track("color_track")
    rclpy.spin(color_track)
