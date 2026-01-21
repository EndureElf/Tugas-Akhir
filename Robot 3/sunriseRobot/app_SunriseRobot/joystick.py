#!/usr/bin/env python3
# coding=utf-8
import os, struct, sys

import time
import threading

from SunriseRobotLib import SunriseRobot as Robot

# V1.0.4
class Joystick(object):

    def __init__(self, robot:Robot, js_id=0, debug=False):
        self.__debug = debug
        self.__js_id = int(js_id)
        self.__js_isOpen = False
        self.__ignore_count = 24
        self.__robot = robot

        self.STATE_OK = 0
        self.STATE_NO_OPEN = 1
        self.STATE_DISCONNECT = 2
        self.STATE_KEY_BREAK = 3

        self.__speed_ctrl = 0.7
        self.__speed_x = 0
        self.__speed_y = 0
        self.__speed_z = 0

        # Find the joystick device.
        print('Joystick Available devices:')
        # Shows the joystick list of the Controler, for example: /dev/input/js0
        for fn in os.listdir('/dev/input'):
            if fn.startswith('js'):
                print('    /dev/input/%s' % (fn))

        # Open the joystick device.
        try:
            js = '/dev/input/js' + str(self.__js_id)
            self.__jsdev = open(js, 'rb')
            self.__js_isOpen = True
            print('---Opening %s Succeeded---' % js)
        except:
            self.__js_isOpen = False
            print('---Failed To Open %s---' % js)
        
        # Defining Functional List
        # Red LED Mode
        self.__function_names = {
            # BUTTON FUNCTION
            0x0100: 'A',
            0x0101: 'B',
            0x0103: 'X',
            0x0104: 'Y',
            0x0106: 'L1',
            0x0107: 'R1',
            0x0108: 'L2_1',
            0x0109: 'R2_1',
            0x010A: 'SELECT',
            0x010B: 'START',
            0x010D: 'BTN_RK1',
            0x010E: 'BTN_RK2',

            # AXIS FUNCTION
            0x0200: 'RK1_LEFT_RIGHT',
            0x0201: 'RK1_UP_DOWN',
            0x0202: 'RK2_LEFT_RIGHT',
            0x0203: 'RK2_UP_DOWN',
            0x0204: 'R2',
            0x0205: 'L2',
            0x0206: 'WSAD_LEFT_RIGHT',
            0x0207: 'WSAD_UP_DOWN',
        }

        # Green LED Mode
        # self.KEYS = {
        #     # BUTTON FUNCTION
        #     0x0100 : 'A',
        #     0x0101 : 'B',
        #     0x0102 : 'X',
        #     0x0103 : 'Y',
        #     0x0104 : 'L1',
        #     0x0105 : 'R1',
        #     0x0106 : 'SELECT',
        #     0x0107 : 'START',
        #     0x0108 : 'MODE',
        #     0x0109 : 'BTN_RK1',
        #     0x010A : 'BTN_RK2',

        #     # AXIS FUNCTION
        #     0x0200 : 'RK1_LEFT_RIGHT',
        #     0x0201 : 'RK1_UP_DOWN',
        #     0x0202 : 'L2',
        #     0x0203 : 'RK2_LEFT_RIGHT',
        #     0x0204 : 'RK2_UP_DOWN',
        #     0x0205 : 'R2',
        #     0x0206 : 'WSAD_LEFT_RIGHT',
        #     0x0207 : 'WSAD_UP_DOWN',
        # }

    def __del__(self):
        if self.__js_isOpen:
            self.__jsdev.close()
        if self.__debug:
            print("\n---Joystick DEL---\n")

    # Return joystick state
    def is_Opened(self):
        return self.__js_isOpen
    
    # transform data
    def __my_map(self, x, in_min, in_max, out_min, out_max):
        return (out_max - out_min) * (x - in_min) / (in_max - in_min) + out_min


    # Control robot
    def __data_processing(self, name, value):
        
        if name=="RK1_LEFT_RIGHT":
            value = -value / 32767
            if self.__debug:
                print ("%s : %.3f" % (name, value))
            self.__speed_y = value * self.__speed_ctrl
            self.__robot.set_car_motion(self.__speed_x, self.__speed_y, self.__speed_z)
        
        elif name == 'RK1_UP_DOWN':
            value = -value / 32767
            if self.__debug:
                print ("%s : %.3f" % (name, value))
            self.__speed_x = value * self.__speed_ctrl
            self.__robot.set_car_motion(self.__speed_x, self.__speed_y, self.__speed_z)

        elif name == 'RK2_LEFT_RIGHT':
            value = -value / 32767
            if self.__debug:
                print ("%s : %.3f" % (name, value))
            self.__speed_z = value * 5 * self.__speed_ctrl
            self.__robot.set_car_motion(self.__speed_x, self.__speed_y, self.__speed_z)
            
        elif name == 'RK2_UP_DOWN':
            value = -value / 32767
            if self.__debug:
                print ("%s : %.3f" % (name, value))

        elif name == 'A':
            if self.__debug:
                print (name, ":", value)

        elif name == 'B':
            if self.__debug:
                print (name, ":", value)
            if value == 1:
                self.__robot.set_car_motion(0, 0, -self.__speed_ctrl*5)
            else:
                self.__robot.set_car_motion(0, 0, 0)

        elif name == 'X':
            if self.__debug:
                print (name, ":", value)
            if value == 1:
                self.__robot.set_car_motion(0, 0, self.__speed_ctrl*5)
            else:
                self.__robot.set_car_motion(0, 0, 0)
        
        elif name == 'Y':
            if self.__debug:
                print (name, ":", value)

        elif name == 'L1':
            if self.__debug:
                print (name, ":", value)

        elif name == 'R1':
            if self.__debug:
                print (name, ":", value)

        elif name == 'SELECT':
            if self.__debug:
                print (name, ":", value)

        elif name == 'START':
            if self.__debug:
                print (name, ":", value)
            if value == 1:
                self.__robot.set_beep(1)
            else:
                self.__robot.set_beep(0)

        elif name == 'MODE':
            if self.__debug:
                print (name, ":", value)
        elif name == 'BTN_RK1':
            if self.__debug:
                print (name, ":", value)
            if value == 1:
                self.__speed_ctrl = self.__speed_ctrl + 0.3
                if self.__speed_ctrl > 1:
                    self.__speed_ctrl = 0.4

        elif name == 'BTN_RK2':
            if self.__debug:
                print (name, ":", value)
        
        elif name == "L2":
            value = ((value/32767)+1)/2
            if self.__debug:
                print ("%s : %.3f" % (name, value))

        elif name == "R2":
            value = ((value/32767)+1)/2
            if self.__debug:
                print ("%s : %.3f" % (name, value))

            
        elif name == 'WSAD_LEFT_RIGHT':
            value = -value / 32767
            if self.__debug:
                print ("%s : %.3f" % (name, value))
            if value > 0:
                self.__robot.set_car_motion(0, self.__speed_ctrl, 0)
            elif value < 0:
                self.__robot.set_car_motion(0, -self.__speed_ctrl, 0)
            else:
                self.__robot.set_car_motion(0, 0, 0)
            
        elif name == 'WSAD_UP_DOWN':
            value = -value / 32767
            if self.__debug:
                print ("%s : %.3f" % (name, value))
            if value > 0:
                self.__robot.set_car_motion(self.__speed_ctrl, 0, 0)
            elif value < 0:
                self.__robot.set_car_motion(-self.__speed_ctrl, 0, 0)
            else:
                self.__robot.set_car_motion(0, 0, 0)

        else:
            pass

    # Handles events for joystick
    def joystick_handle(self):
        if not self.__js_isOpen:
            # if self.__debug:
            #     print('Failed To Open Joystick')
            return self.STATE_NO_OPEN
        try:
            evbuf = self.__jsdev.read(8)
            if evbuf:
                time, value, type, number = struct.unpack('IhBB', evbuf)
                func = type << 8 | number
                name = self.__function_names.get(func)
                # print("evbuf:", time, value, type, number)
                # if self.__debug:
                #     print("func:0x%04X, %s, %d" % (func, name, value))
                if name != None:
                    self.__data_processing(name, value)
                else:
                    if self.__ignore_count > 0:
                        self.__ignore_count = self.__ignore_count - 1
                    if self.__debug and self.__ignore_count == 0:
                        print("Key Value Invalid")
            return self.STATE_OK
        except KeyboardInterrupt:
            if self.__debug:
                print('Key Break Joystick')
            return self.STATE_KEY_BREAK
        except:
            self.__js_isOpen = False
            print('---Joystick Disconnected---')
            return self.STATE_DISCONNECT

    # reconnect Joystick
    def reconnect(self):
        try:
            js = '/dev/input/js' + str(self.__js_id)
            self.__jsdev = open(js, 'rb')
            self.__js_isOpen = True
            self.__ignore_count = 24
            print('---Opening %s Succeeded---' % js)
            return True
        except:
            self.__js_isOpen = False
            # if self.__debug:
            #     print('Failed To Open %s' % js)
            return False


if __name__ == '__main__':
    g_debug = False
    if len(sys.argv) > 1:
        if str(sys.argv[1]) == "debug":
            g_debug = True
    print("debug=", g_debug)

    g_robot = Robot()
    js = Joystick(g_robot, debug=g_debug)
    try:
        while True:
            state = js.joystick_handle()
            if state != js.STATE_OK:
                if state == js.STATE_KEY_BREAK:
                    break
                time.sleep(1)
                js.reconnect()
    except KeyboardInterrupt:
        pass
    del js
