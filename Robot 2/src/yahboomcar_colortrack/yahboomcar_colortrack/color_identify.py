#ros lib
import rclpy
from rclpy.node import Node

from std_msgs.msg import Char
from geometry_msgs.msg import Twist, Polygon
from sensor_msgs.msg import Image
from yahboomcar_msgs.msg import Position
from yahboomcar_colortrack.astra_common import *

from cv_bridge import CvBridge

print("import done")

cv_edition = cv.__version__
print("cv_edition: ",cv_edition)

class Color_Identify(Node):
    def __init__(self, name):
        super().__init__(name)
        self.pub_position = self.create_publisher(Position,"/Current_point", 10)
        self.pub_cmdVel = self.create_publisher(Twist, '/cmd_vel', 10)
        self.pub_binary = self.create_publisher(Image, '/astra_binary', 10)
        self.pub_roi = self.create_publisher(Image, '/astra_roi', 10)
        self.pub_image = self.create_publisher(Image, '/camera/color/image_raw', 10)
        
        # self.sub_image = self.create_subscription(Image, '/camera/color/image_raw', self.ImageProcess, 1)
        self.sub_mouse = self.create_subscription(Polygon, '/mouse_pos', self.MouseCallback, 1)
        self.sub_keyboard = self.create_subscription(Char, '/key_value', self.KeyCallback, 1)

        self.Track_state = 'identify'
        self.mouse_active = False
        self.Roi_init = ()
        self.hsv_range = ()
        self.circle = (0, 0, 0)
        self.point_pose = (0, 0, 0)

        self.bridge = CvBridge()
        self.color = color_follow()
        self.declare_param()

        self.capture = cv.VideoCapture(0)
        if cv_edition[0]=='3': self.capture.set(cv.CAP_PROP_FOURCC, cv.VideoWriter_fourcc(*'XVID'))
        else: self.capture.set(cv.CAP_PROP_FOURCC, cv.VideoWriter.fourcc('M', 'J', 'P', 'G'))
        self.timer = self.create_timer(0.1, self.on_timer)

    def declare_param(self):
        #HSV
        self.declare_parameter('Hmin',0)
        self.Hmin = self.get_parameter('Hmin').get_parameter_value().integer_value
        self.declare_parameter('Smin',85)
        self.Smin = self.get_parameter('Smin').get_parameter_value().integer_value
        self.declare_parameter('Vmin',126)
        self.Vmin = self.get_parameter('Vmin').get_parameter_value().integer_value
        self.declare_parameter('Hmax',9)
        self.Hmax = self.get_parameter('Hmax').get_parameter_value().integer_value
        self.declare_parameter('Smax',253)
        self.Smax = self.get_parameter('Smax').get_parameter_value().integer_value
        self.declare_parameter('Vmax',253)
        self.Vmax = self.get_parameter('Vmax').get_parameter_value().integer_value

    def get_param(self):
        #hsv
        self.Hmin = self.get_parameter('Hmin').get_parameter_value().integer_value
        self.Smin = self.get_parameter('Smin').get_parameter_value().integer_value
        self.Vmin = self.get_parameter('Vmin').get_parameter_value().integer_value
        self.Hmax = self.get_parameter('Hmax').get_parameter_value().integer_value
        self.Smax = self.get_parameter('Smax').get_parameter_value().integer_value
        self.Vmax = self.get_parameter('Vmax').get_parameter_value().integer_value
    
    def on_timer(self):
        ret, rgbFrame = self.capture.read()
        self.pub_image.publish(self.bridge.cv2_to_imgmsg(rgbFrame, "bgr8"))
        if self.Track_state == 'identify':
            if self.mouse_active:
                cv_image, self.hsv_range = self.color.Roi_hsv(rgbFrame, self.Roi_init)
                if len(self.hsv_range) != 0:
                    self.Hmin = rclpy.parameter.Parameter('Hmin',rclpy.Parameter.Type.INTEGER,self.hsv_range[0][0])
                    self.Smin = rclpy.parameter.Parameter('Smin',rclpy.Parameter.Type.INTEGER,self.hsv_range[0][1])
                    self.Vmin = rclpy.parameter.Parameter('Vmin',rclpy.Parameter.Type.INTEGER,self.hsv_range[0][2])
                    self.Hmax = rclpy.parameter.Parameter('Hmax',rclpy.Parameter.Type.INTEGER,self.hsv_range[1][0])
                    self.Smax = rclpy.parameter.Parameter('Smax',rclpy.Parameter.Type.INTEGER,self.hsv_range[1][1])
                    self.Vmax = rclpy.parameter.Parameter('Vmax',rclpy.Parameter.Type.INTEGER,self.hsv_range[1][2])
                    all_new_parameters = [self.Hmin,self.Smin,self.Vmin,self.Hmax,self.Smax,self.Vmax]
                    self.set_parameters(all_new_parameters)
                    # self.pub_roi.publish(self.bridge.cv2_to_imgmsg(cv_image, "bgr8"))
                    rgbFrame, binary, self.circle = self.color.object_follow(rgbFrame, self.hsv_range)
                    self.pub_binary.publish(self.bridge.cv2_to_imgmsg(binary, "mono8"))
                    self.pub_roi.publish(self.bridge.cv2_to_imgmsg(rgbFrame, "bgr8"))
                self.mouse_active = False
                # print(f'HSV: ({self.Hmin},{self.Hmax}), ({self.Smin},{self.Smax}), ({self.Vmin},{self.Vmax})')
            else: 
                self.get_param()
        
        elif self.Track_state == 'tracking':
            if len(self.hsv_range) != 0:
                rgbFrame, binary, self.circle = self.color.object_follow(rgbFrame, self.hsv_range)
                self.pub_binary.publish(self.bridge.cv2_to_imgmsg(binary, "mono8"))
                self.pub_roi.publish(self.bridge.cv2_to_imgmsg(rgbFrame, "bgr8"))
                if self.circle[2] != 0: 
                    self.execute(self.circle[0], self.circle[1], self.circle[2])
        
        elif self.Track_state == 'reset':
            cv.putText(rgbFrame, 'HSV : null', (275, 30), cv.FONT_HERSHEY_SIMPLEX, 0.75, (0, 0, 255), 2)
            self.pub_roi.publish(self.bridge.cv2_to_imgmsg(rgbFrame, "bgr8"))
            self.pub_binary.publish(self.bridge.cv2_to_imgmsg(rgbFrame, "bgr8"))

    # def ImageProcess(self, msg):
    #     rgbFrame = self.bridge.imgmsg_to_cv2(msg, "bgr8")
    #     if self.Track_state == 'identify':
    #         if self.mouse_active:
    #             cv_image, self.hsv_range = self.color.Roi_hsv(rgbFrame, self.Roi_init)
    #             if len(self.hsv_range) != 0:
    #                 self.Hmin = rclpy.parameter.Parameter('Hmin',rclpy.Parameter.Type.INTEGER,self.hsv_range[0][0])
    #                 self.Smin = rclpy.parameter.Parameter('Smin',rclpy.Parameter.Type.INTEGER,self.hsv_range[0][1])
    #                 self.Vmin = rclpy.parameter.Parameter('Vmin',rclpy.Parameter.Type.INTEGER,self.hsv_range[0][2])
    #                 self.Hmax = rclpy.parameter.Parameter('Hmax',rclpy.Parameter.Type.INTEGER,self.hsv_range[1][0])
    #                 self.Smax = rclpy.parameter.Parameter('Smax',rclpy.Parameter.Type.INTEGER,self.hsv_range[1][1])
    #                 self.Vmax = rclpy.parameter.Parameter('Vmax',rclpy.Parameter.Type.INTEGER,self.hsv_range[1][2])
    #                 all_new_parameters = [self.Hmin,self.Smin,self.Vmin,self.Hmax,self.Smax,self.Vmax]
    #                 self.set_parameters(all_new_parameters)
    #                 self.pub_roi.publish(self.bridge.cv2_to_imgmsg(cv_image, "bgr8"))
    #                 rgbFrame, binary, self.circle = self.color.object_follow(rgbFrame, self.hsv_range)
    #                 self.pub_binary.publish(self.bridge.cv2_to_imgmsg(binary, "mono8"))
    #             self.mouse_active = False
    #             # print(f'HSV: ({self.Hmin},{self.Hmax}), ({self.Smin},{self.Smax}), ({self.Vmin},{self.Vmax})')
    #         else: 
    #             self.get_param()
        
    #     elif self.Track_state == 'tracking':
    #         if len(self.hsv_range) != 0:
    #             rgbFrame, binary, self.circle = self.color.object_follow(rgbFrame, self.hsv_range)
    #             self.pub_binary.publish(self.bridge.cv2_to_imgmsg(binary, "mono8"))
    #             if self.circle[2] != 0: 
    #                 self.execute(self.circle[0], self.circle[1], self.circle[2])
        
    #     elif self.Track_state == 'reset':
    #         cv.putText(rgbFrame, 'HSV : null', (275, 30), cv.FONT_HERSHEY_SIMPLEX, 0.75, (0, 0, 255), 2)
    #         self.pub_roi.publish(self.bridge.cv2_to_imgmsg(rgbFrame, "bgr8"))
    #         self.pub_binary.publish(self.bridge.cv2_to_imgmsg(rgbFrame, "bgr8"))


    def execute(self, x, y, z):
        position = Position()
        position.anglex = x * 1.0
        position.angley = y * 1.0
        position.distance = z * 1.0
        self.pub_position.publish(position)

    def MouseCallback(self, msg):
        start_cols = min(int(msg.points[0].x), int(msg.points[1].x))
        start_rows = min(int(msg.points[0].y), int(msg.points[1].y))
        end_cols = max(int(msg.points[0].x), int(msg.points[1].x))
        end_rows = max(int(msg.points[0].y), int(msg.points[1].y))
        # self.get_logger().info(f'Roi: {start_cols}, {start_rows}, {end_cols}, {end_rows}')
        self.Roi_init = (start_cols, start_rows, end_cols, end_rows)
        self.mouse_active = True

    def KeyCallback(self, key):
        data = key.data
        if data == 32: 
            self.Track_state = 'tracking'

        elif data == ord('i') or data == ord('I'): 
            self.Track_state = 'identify'
            self.pub_cmdVel.publish(Twist())

        elif data == ord('r') or data == ord('R'): 
            self.Track_state = 'reset'
            self.pub_cmdVel.publish(Twist())
            for _ in range(3): 
                self.pub_position.publish(Position())
            self.hsv_range = ()

        elif data == ord('q') or data == ord('Q'): 
            self.Track_state = 'cancle'
            self.pub_cmdVel.publish(Twist())
            for _ in range(3): 
                self.pub_position.publish(Position())
        # self.get_logger().info(f'Track_state: {data}, {self.Track_state}')

    
def main(args=None):
    rclpy.init()
    color_identify = Color_Identify("color_identify")
    rclpy.spin(color_identify)

