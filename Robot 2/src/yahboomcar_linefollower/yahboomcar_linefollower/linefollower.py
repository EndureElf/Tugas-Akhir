#!/usr/bin/env python
# -*- coding: utf-8 -*-

# Copyright (c) 2022, www.guyuehome.com
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import rclpy, cv2, cv_bridge, numpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist


def read_HSV(rf_path):
    rf = open(rf_path, "r+")
    line = rf.readline()
    if len(line) == 0: return ()
    list = line.split(',')
    if len(list) != 6: return ()
    hsv = ((int(list[0]), int(list[1]), int(list[2])),
           (int(list[3]), int(list[4]), int(list[5])))
    rf.flush()
    return hsv

class Follower(Node):
    def __init__(self):
        super().__init__('line_follower')
        self.get_logger().info("Start line follower.")
        self.hsv_txt = '/home/sunrise/yahboomcar_ws/src/yahboomcar_linefollower/yahboomcar_linefollower/LineHSV.txt'
        self.bridge = cv_bridge.CvBridge()

        self.image_sub = self.create_subscription(Image, '/image_raw', self.image_callback, 10)
        self.cmd_vel_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.pub = self.create_publisher(Image, '/camera/process_image', 10)
        self.twist = Twist()

    def image_callback(self, msg):
        image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        hsv_range = read_HSV(self.hsv_txt)
        # black: 90,0,0,180,255,100
        # yellow: 26,43,46,34,255,255
        h_min = hsv_range[0][0]
        s_min = hsv_range[0][1]
        v_min = hsv_range[0][2]
        h_max = hsv_range[1][0]
        s_max = hsv_range[1][1]
        v_max = hsv_range[1][2]
        lower_yellow = numpy.array([h_min, s_min, v_min])
        upper_yellow = numpy.array([h_max, s_max, v_max])
        mask = cv2.inRange(hsv, lower_yellow, upper_yellow)

        h, w, d = image.shape
        search_top = int(h/2)
        search_bot = int(h/2 + 20)
        mask[0:search_top, 0:w] = 0
        mask[search_bot:h, 0:w] = 0
        M = cv2.moments(mask)

        if M['m00'] > 0:
            cx = int(M['m10']/M['m00'])
            cy = int(M['m01']/M['m00'])
            cv2.circle(image, (cx, cy), 20, (0,0,255), -1)

            # 基于检测的目标中心点，计算机器人的控制参数
            self.twist.linear.x = 0.1
            self.twist.angular.z = -1.0 * (cx - 480) / 600.0
            self.cmd_vel_pub.publish(self.twist)
            
        self.pub.publish(self.bridge.cv2_to_imgmsg(image, 'bgr8'))

def main(args=None):
    rclpy.init(args=args)    
    follower = Follower()
    rclpy.spin(follower)
