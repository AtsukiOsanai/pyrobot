# Copyright (c) Facebook, Inc. and its affiliates.

# This source code is licensed under the MIT license found in the
# LICENSE file in the root directory of this source tree.

from yacs.config import CfgNode as CN

from pyrobot.cfg.config import get_cfg_defaults

_C = get_cfg_defaults()

# whether the robot has an arm or not
_C.HAS_ARM = False
# whether the robot has a mobile base or not
_C.HAS_BASE = True
# whether the robot has a camera or not
_C.HAS_CAMERA = True
# whether the robot has a gripper or not
_C.HAS_GRIPPER = False
# whether the robot has a common shared class among all components
_C.HAS_COMMON = True

_CAMERAC = _C.CAMERA
# CAMERA class name
_CAMERAC.CLASS = 'LoCoBotCamera'
# topic name of the camera info
_CAMERAC.ROSTOPIC_CAMERA_INFO_STREAM = '/camera/rgb/camera_info'
# topic name of the RGB images
_CAMERAC.ROSTOPIC_CAMERA_RGB_STREAM = '/camera/rgb/image_raw'
# topic name of the depth images
_CAMERAC.ROSTOPIC_CAMERA_DEPTH_STREAM = '/camera/depth/image_raw'

_BASEC = _C.BASE
# BASE class name
_BASEC.CLASS = 'TurtleBot2Base'
# Type of base being used 'kobuki' or 'create'
_BASEC.BASE_TYPE = 'kobuki'
# Type of contrller being used for postion control
# 'ilqr' or 'proportional' or 'movebase'
_BASEC.BASE_CONTROLLER = 'ilqr'
# Type of planner being used for slam base path planning 'movebase'
_BASEC.BASE_PLANNER = 'movebase'
# Rostopic on which the velocity commands to be published
_BASEC.ROSTOPIC_BASE_COMMAND = '/navigation_velocity_smoother/raw_cmd_vel'
# Rostopic on which the wheel-encoder-based odommetry is available
_BASEC.ROSTOPIC_ODOMETRY = '/odom'
# Rostopic on which base bumper sensor informations is available
_BASEC.ROSTOPIC_BUMPER = '/mobile_base/events/bumper'
# Rosotopic on which base cliff sensor information is available
_BASEC.ROSTOPIC_CLIFF = '/mobile_base/events/cliff'
# Rostopic on whihc the base wheeldrop sensor info is available
_BASEC.ROSTOPIC_WHEELDROP = '/mobile_base/events/wheel_drop'

# Rostopic on which movebase goal to be pusblished
_BASEC.ROSTOPIC_MOVE_BASE_GOAL = '/move_base_simple/goal'
# Rostopic on which the movebase execution status is available
_BASEC.ROSTOPIC_MOVE_BASE_STATUS = '/move_base/status'
# Rostopic on which the command to cancel the goal sent to movebase should be
_BASEC.ROSTOPIC_GOAL_CANCEL = '/move_base/cancel'
# world frame name
_BASEC.MAP_FRAME = '/map'
# Rosaction topic for movebase
_BASEC.ROSTOPIC_BASE_ACTION_COMMAND = 'move_base'
# Rate of control for ILQR
_BASEC.BASE_CONTROL_RATE = 10
# Maximum linear for velocity control and ILQR
_BASEC.MAX_ABS_FWD_SPEED = 0.2
# Maximum rotational velocity for velocity control and ILQR
_BASEC.MAX_ABS_TURN_SPEED = 0.5
# ROSTOPIC to send movebase (x,ym theta) planner request
_BASEC.PLAN_TOPIC = '/move_base/GlobalPlanner/make_plan'
# Index of the point to be tracked on the plan.
# (used by Proportional and ILQR trajectory tracking)
_BASEC.TRACKED_POINT = 20
# Linear treshold used by trajectory tracking with proportional and ILQR
_BASEC.TRESHOLD_LIN = 0.15
# Tolearance to be used by movebase planner while generating plans
_BASEC.PLAN_TOL = 0.1
# z minimum cut-off height for slam-based costmap computation
_BASEC.Z_MIN_TRESHOLD_OCC_MAP = 0.1
# z maximum cut-off height for slam-based costmap computation
_BASEC.Z_MAX_TRESHOLD_OCC_MAP = 0.8
# proportional control specific max linear velocity
_BASEC.MAX_ABS_FWD_SPEED_P_CONTROLLER = 0.5
# proportional control specific max angular velocity
_BASEC.MAX_ABS_TURN_SPEED_P_CONTROLLER = 1
# proportional control specific ignore translation treshold
_BASEC.TRANSLATION_TRESHOLD = 0.01

_BASEC.VSLAM = CN()
# topic name of the camera pose
_BASEC.VSLAM.ROSTOPIC_CAMERA_POSE = '/orb_slam2_rgbd/slam/camera_pose'
# topic name of the camera trajectory
_BASEC.VSLAM.ROSTOPIC_CAMERA_TRAJ = '/orb_slam2_rgbd/slam/camera_traj'
# reference link name of the visual SLAM system, the pose of this frame
# in the first time step will be used to define
# the origin/orientation of the world frame
_BASEC.VSLAM.VSLAM_BASE_FRAME = '/base_link'
# RGB camera center frame name
_BASEC.VSLAM.RGB_CAMERA_CENTER_FRAME = '/camera_color_optical_frame'
# minimum depth values to be considered as valid
_BASEC.VSLAM.DEPTH_MIN = 0.2
# maximum depth values to be considered as valid
_BASEC.VSLAM.DEPTH_MAX = 1.5
# configuration file name for ORB-SLAM2
_BASEC.VSLAM.CFG_FILENAME = 'realsense_d435.yaml'
# sample every n pixels in depth images during
# reconstruction (to save computation time)
_BASEC.VSLAM.SUBSAMPLE_PIXS = 1
# publishing frequence of the occupancy map
_BASEC.VSLAM.OCCUPANCY_MAP_RATE = 0.5


def get_cfg(base_type='kobuki'):
    global _C
    if base_type not in ['kobuki']:
        raise ValueError('Unsupported base type: {:s}'.format(base_type))
    return _C.clone()
