#!/usr/bin/env python

# Copyright (c) Facebook, Inc. and its affiliates.

# This source code is licensed under the MIT license found in the
# LICENSE file in the root directory of this source tree.

"""
Client and server for keyboard teleoperation
"""
import copy
import os
import sys
import termios
import time
import tty

import numpy as np
import rospy
import yaml
from geometry_msgs.msg import Twist
from pyrobot import Robot
from std_msgs.msg import Empty, String, Bool
from tf.transformations import euler_from_quaternion

KEY_CTRL_C = '\x03'
KEY_BASE_FORWARD = '\x1b[A'
KEY_BASE_BACKWARD = '\x1b[B'
KEY_BASE_TURNRIGHT = '\x1b[C'
KEY_BASE_TURNLEFT = '\x1b[D'

HELP_MSG = """
Teleoperate your LoCoBotBase!
---------------------------
Pre-requisite:
roslaunch locobotbase_control teleoperation.launch

Moving around:
5) Pan-Tilt Camera
    u (+pan)  i (-pan)  o (+tilt)  p (-tilt)

6) Move Base (Arrow Keys)
    up (move forward) down (move backward)
    left (turn left)  right (turn right) 
    
CTRL-C to quit
"""


class RobotTeleoperationServer():
    def __init__(self, cfg_file='teleop.yaml'):

        # Load config file
        dir_path = os.path.dirname(os.path.realpath(__file__))
        root_path = os.path.dirname(os.path.dirname(dir_path))
        cfg_path = os.path.join(root_path, 'config', cfg_file)
        with open(cfg_path, 'r') as f:
            self.cfg = yaml.load(f)

        self.use_camera = rospy.get_param('use_camera')
        self.use_base = rospy.get_param('use_base')

        self.bot = Robot('locobotbase',
                         use_base=self.use_base,
                         use_camera=self.use_camera,
                         use_arm=False,
                         use_gripper=False)

        # Subscribers
        rospy.Subscriber(self.cfg['ROSTOPIC_TELEOP'],
                         String,
                         self._callback_cmd_vel)
        rospy.Subscriber(self.cfg['ROSTOPIC_HARDWARE_STATUS'], Bool,
                         self._callback_hardware_status)

        # Publishers
        self.pub_stop = rospy.Publisher(
            self.cfg['ROSTOPIC_STOP_EXECUTION'], Empty, queue_size=1)
        self.base_cmd_pub = rospy.Publisher(self.cfg['ROSTOPIC_BASE_COMMAND'],
                                            Twist, queue_size=1)
        self.safety_status_pub = rospy.Publisher(self.cfg['ROSTOPIC_SAFETY_STATUS'],
                                                 Bool, queue_size=1)

        # Initialize pan-tilt camera
        # NOTE - Do not remove the following steps!
        if self.use_camera:
            self.bot.camera.set_pan(self.cfg['START_PAN'])
            self.bot.camera.set_tilt(self.cfg['START_TILT'])

        self.cmd = None
        self.status = True
        self.cmd_prev = None
        self.is_robot_moving = False
        self.base_linear_speed = 0.05
        self.base_angular_speed = 0.5
        self.start_time = time.time()

        rospy.loginfo('Teleop server ready to accept requests...')

    def _callback_cmd_vel(self, data):
        print('cb_cmd: ', data)
        self.cmd = data.data

    def _callback_hardware_status(self, data):
        self.status = data.data

    def move_pan(self, increment=True):
        sign = 1 if increment else -1
        assert self.use_camera, 'Please set `use_camera:=true` when you launch the robot driver'
        self.bot.camera.set_pan(self.bot.camera.get_pan()
                                + sign * self.cfg['DELTA_ANGLE_CAMERA'],
                                wait=False)

    def move_tilt(self, increment=True):
        sign = 1 if increment else -1
        assert self.use_camera, 'Please set `use_camera:=true` when you launch the robot driver'
        self.bot.camera.set_tilt(self.bot.camera.get_tilt()
                                 + sign * self.cfg['DELTA_ANGLE_CAMERA'],
                                 wait=False)

    def check_safety(self, key):
        """
        Simple heuristics for avoiding collisions
        """

        self.safety_status_pub.publish(True)
        return True

    def move_base(self, lin_speed, ang_speed):
        if self.use_base:
            if np.fabs(lin_speed) > 0.01 or np.fabs(ang_speed) > 0.01:
                self.is_robot_moving = True
                self.bot.base.set_vel(fwd_speed=lin_speed,
                                      turn_speed=ang_speed,
                                      exe_time=0.5)
            else:
                self.bot.base.stop()

    def reset(self):
        self.set_joint(np.zeros(5))

    def run(self):
        rospy.sleep(1)
        while True:
            key = copy.deepcopy(self.cmd)
            if key is None:
                if self.is_robot_moving and \
                        time.time() - self.start_time > self.cfg['WAIT_TIME']:
                    self.move_base(0, 0)
                    self.is_robot_moving = False
                continue

            delta = np.zeros(3)

            if not self.status:
                rospy.logerr('there is some error, check logs')
                self.exit()

            elif key == KEY_CTRL_C:
                self.exit()
            elif key == self.cfg['KEY_POS_PAN']:
                print('POS_PAN')
                self.move_pan()
            elif key == self.cfg['KEY_NEG_PAN']:
                print('NEG_PAN')
                self.move_pan(False)
            elif key == self.cfg['KEY_POS_TILT']:
                print('POS_TILT')
                self.move_tilt()
            elif key == self.cfg['KEY_NEG_TILT']:
                print('NEG_TILT')
                self.move_tilt(False)
            elif key == KEY_BASE_FORWARD:
                self.move_base(self.base_linear_speed, 0)
            elif key == KEY_BASE_BACKWARD:
                self.move_base(-self.base_linear_speed, 0)
            elif key == KEY_BASE_TURNLEFT:
                self.move_base(0, self.base_angular_speed)
            elif key == KEY_BASE_TURNRIGHT:
                self.move_base(0, -self.base_angular_speed)
            elif key == self.cfg['KEY_RESET']:
                self.reset()
            else:
                continue
                if key.isdigit():
                    key_int = int(copy.deepcopy(key))
                else:
                    print('Pressed invalid key: {}'.format(key))
            self.cmd_prev = copy.deepcopy(key)
            self.cmd = None

    def exit(self):
        rospy.loginfo('Exiting...')
        sys.exit(0)

    def signal_handler(self, sig, frame):
        self.exit()


def getch():
    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)
    try:
        tty.setraw(sys.stdin.fileno())
        ch = sys.stdin.read(1)
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
    return ch


class KeyboardTeleoperationClient():
    def __init__(self, cfg_file='teleop.yaml'):
        rospy.init_node('keyboard_teleoperation_client', anonymous=True)

        dir_path = os.path.dirname(os.path.realpath(__file__))
        root_path = os.path.dirname(os.path.dirname(dir_path))
        cfg_path = os.path.join(root_path, 'config', cfg_file)
        with open(cfg_path, 'r') as f:
            self.cfg = yaml.load(f)

        self.pub = rospy.Publisher(self.cfg['ROSTOPIC_TELEOP'], String, queue_size=1)

    def run(self):
        print(HELP_MSG)
        while True:
            cmd = getch()
            if cmd == '\x1b':
                cmd_suffix1 = getch()
                if cmd_suffix1 == '[':
                    cmd_suffix2 = getch()
                    cmd = cmd + cmd_suffix1 + cmd_suffix2

            if cmd == KEY_CTRL_C:
                self.exit()
            else:
                self.pub.publish(cmd)

    def exit(self):
        rospy.loginfo('Exiting...')
        sys.exit(0)
