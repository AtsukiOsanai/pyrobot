/*******************************************************************************
# Copyright (c) Facebook, Inc. and its affiliates.
# This source code is licensed under the MIT license found in the
# LICENSE file in the root directory of this source tree.
*******************************************************************************/
#ifndef LOCOBOTBASE_GAZEBO_H
#define LOCOBOTBASE_GAZEBO_H

#include <ros/ros.h>
//#include "message_header.h"

#include <std_msgs/Empty.h>
#include <std_msgs/Float64.h>


class GazeboInteface {

private:

	ros::NodeHandle node_handle_;

	ros::Publisher head_pan_pub;
	ros::Publisher head_tilt_pub;
	ros::Publisher pub_arr[2];

public:

	GazeboInteface();
};

#endif
