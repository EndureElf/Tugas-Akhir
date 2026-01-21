#!/usr/bin/env python3
import os
import cv2 as cv
import time
import numpy as np

# Camera API libs
from hobot_vio import libsrcampy as srcampy

class Mipi_Camera(object):
    
    DEFAULT_WIDTH = 1920
    DEFAULT_HEIGHT = 1080
    DEFAULT_FPS = 30
    
    def __init__(self, width=DEFAULT_WIDTH, height=DEFAULT_HEIGHT, debug=False):
        self.__debug = debug
        self.__state = False
        self.__width = width
        self.__height = height

        self.__camera = srcampy.Camera()
        error = self.__camera.open_cam(0, -1, self.DEFAULT_FPS, self.__width, self.__height)
        if error == 0:
            self.__state = True
            if self.__debug:
                print("Open CSI Camera OK")
        else:
            self.__state = False
            if self.__debug:
                print("Fail to Open CSI Camera")

    def __del__(self):
        self.__clear()
        if self.__debug:
            print("---Mipi_Camera Del---")

    # nv12图像转化成bgr图像
    def __nv12_to_bgr_opencv(self, image, width, height):
        frame = np.frombuffer(image, dtype=np.uint8)
        # img_bgr = cv.cvtColor(frame.reshape((height * 3 // 2, width)), cv.COLOR_YUV2BGR_NV12)
        img_bgr = cv.cvtColor(frame.reshape(1620, 1920), cv.COLOR_YUV2BGR_NV12)
        return img_bgr
    
    # 获取图像的公共逻辑
    def __get_image(self):
        image = self.__camera.get_img(2)
        return image

    # 摄像头是否打开成功
    def isOpened(self):
        return self.__state

    # 释放摄像头
    def __clear(self):
        if self.__state:
            self.__camera.close_cam()
            self.__state = False

    # 获取摄像头的一帧图片
    def get_frame(self):
        image = self.__get_image()
        if image is None:
            return False, bytes({1})
        image = self.__nv12_to_bgr_opencv(image, self.__width, self.__height)
        return True, image

    # 获取摄像头的jpg图片
    def get_frame_jpg(self, text="", color=(0, 255, 0)):
        image = self.__get_image()
        if image is None:
            return False, bytes({1})
        image = self.__nv12_to_bgr_opencv(image, self.__width, self.__height)
        success = True
        if text != "":
            cv.putText(image, str(text), (10, 20), cv.FONT_HERSHEY_SIMPLEX, 0.5, color, 1)
        success, jpeg = cv.imencode('.jpg', image)
        return success, jpeg.tobytes()

    # 获取摄像头的一帧图片 
    def read(self):
        return self.get_frame()
    
    # 释放摄像头设备总线 
    def release(self):
        self.__clear()

if __name__ == '__main__':
    img_width = 640
    img_height = 480
    g_debug = True
    camera = Mipi_Camera(width=img_width, height=img_height, debug=g_debug)

    state = camera.isOpened()
    print("CAM Opened:", state)
    
    try:
        m_fps = 0
        t_start = time.time()
        while state:
            ret, img = camera.get_frame()
            m_fps += 1
            fps = m_fps / (time.time() - t_start)
            if ret:
                text = "FPS:" + str(int(fps))
                cv.putText(img, text, (20, 30), cv.FONT_HERSHEY_SIMPLEX, 0.9, (0, 200, 0), 1)
                print("show opencv, ", text)
                cv.imshow('cv_img', cv.resize(img, (640, 480)))
            else:
                print("read image failed")
                break
            k = cv.waitKey(1) & 0xFF
            if k == 27 or k == ord('q'):
                break
    except Exception as e:
        print("Exception occurred:", e)
    finally:
        del camera
        print("----------Program End------------")