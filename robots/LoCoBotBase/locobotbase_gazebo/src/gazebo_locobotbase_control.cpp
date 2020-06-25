/*******************************************************************************
# Copyright (c) Facebook, Inc. and its affiliates.

# This source code is licensed under the MIT license found in the
# LICENSE file in the root directory of this source tree.
*******************************************************************************/

#include<locobotbase_gazebo/gazebo_locobotbase_control.h>

bool isClose(const double a, const double b)
{
    if (abs(a-b) > 0.005) return false;
    return true;
}

GazeboInteface::GazeboInteface(): node_handle_("") {
    head_pan_pub = node_handle_.advertise < std_msgs::Float64 > ("/pan/command", 10);
    head_tilt_pub = node_handle_.advertise < std_msgs::Float64 > ("/tilt/command", 10);

    pub_arr[0] = head_pan_pub;
    pub_arr[1] = head_tilt_pub;

    //set intial positions to 0 for all the joints
    ros::Duration(5).sleep();

    for (int index = 0; index < 2; index++) {
        std_msgs::Float64 msg;
        msg.data = 0;
        pub_arr[index].publish(msg);
    }
}


int main(int argc, char ** argv) {
    // Init ROS node
    ros::init(argc, argv, "gazebo_locobotbase_control");
    GazeboInteface gzi;
    ros::spin();

    return 0;
}
