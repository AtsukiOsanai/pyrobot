/*******************************************************************************
* Copyright (c) Facebook, Inc. and its affiliates.
* This source code is licensed under the MIT license found in the
* LICENSE file in the root directory of this source tree.

* Copyright 2018 ROBOTIS CO., LTD.
*
* Licensed under the Apache License, Version 2.0 (the "License");
* you may not use this file except in compliance with the License.
* You may obtain a copy of the License at
*
*     http://www.apache.org/licenses/LICENSE-2.0
*
* Unless required by applicable law or agreed to in writing, software
* distributed under the License is distributed on an "AS IS" BASIS,
* WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
* See the License for the specific language governing permissions and
* limitations under the License.
*******************************************************************************/

/*
 * Modified from dynamixel_workbench_controllers/dynamixel_workbench_controllers.cpp
 * Authors: Adithya Murali, Tao Chen, Dhiraj Gandhi
 */
#ifndef LOCOBOTBASE_CONTROLLERS_H
#define LOCOBOTBASE_CONTROLLERS_H

#include <ros/ros.h>

#include <yaml-cpp/yaml.h>

#include <sensor_msgs/JointState.h>
#include <geometry_msgs/Twist.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <trajectory_msgs/JointTrajectoryPoint.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Empty.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Int8.h>

// This is for SimpleActionClient
#include <control_msgs/FollowJointTrajectoryAction.h>
#include <time.h>
#include <mutex>

#include <dynamixel_workbench_toolbox/dynamixel_workbench.h>
#include <dynamixel_workbench_msgs/DynamixelStateList.h>
#include <dynamixel_workbench_msgs/DynamixelCommand.h>

// SYNC_WRITE_HANDLER
#define SYNC_WRITE_HANDLER_FOR_GOAL_POSITION 0
#define SYNC_WRITE_HANDLER_FOR_GOAL_VELOCITY 1

// SYNC_READ_HANDLER(Only for Protocol 2.0)
#define SYNC_READ_HANDLER_FOR_PRESENT_POSITION_VELOCITY_CURRENT 0
#define SYNC_READ_HANDLER_FOR_TORQUE_ENABLE 1
#define SYNC_READ_HANDLER_FOR_SHUTDOWN 2
// #define DEBUG

#define HARDWARE_ERROR_STATUS_OVERLOAD 5
#define HARDWARE_ERROR_STATUS_ELECTRICAL_SHOCK 4
#define HARDWARE_ERROR_STATUS_ENCODER 3
#define HARDWARE_ERROR_STATUS_OVERHEATING 2
#define HARDWARE_ERROR_STATUS_INPUT_VOLTAGE 0

typedef struct {
  std::string item_name;
  int32_t value;
} ItemValue;

class LoCoBotBaseController {
 private:
  // ROS NodeHandle
  ros::NodeHandle node_handle_;
  ros::NodeHandle priv_node_handle_;

  // ROS Parameters

  // ROS Topic Publisher
  ros::Publisher joint_states_pub_;
  ros::Publisher tilt_state_pub_;
  ros::Publisher pan_state_pub_;
  ros::Publisher status_pub_;

  // ROS Topic Subscriber
  ros::Subscriber set_tilt_sub_;
  ros::Subscriber set_pan_sub_;

  // ROS Service Server

  // ROS Service Client

  // ROS Actionlib

  // Dynamixel Workbench Parameters
  DynamixelWorkbench *dxl_wb_;

  // For MoveIt
  std::mutex dynamixel_mutex_;
  std::mutex hardware_mutex_;

  std::map<std::string, std::vector<std::string>> group_motors_;
  std::map<std::string, bool> use_group_;
  std::map<std::string, uint32_t> dynamixel_name_2ids_;
  std::vector<std::pair < std::string, ItemValue>> dynamixel_info_;
  dynamixel_workbench_msgs::DynamixelStateList dynamixel_state_list_;
  std::map<int, int> motor_id_2state_list_id_;
  std::map<int, int> state_list_id_2motor_id_;
  std::map<std::string, const ControlItem *> control_items_;

  sensor_msgs::JointState joint_state_msg_;

  int num_motors_;

  bool is_joint_state_topic_;
  bool is_cmd_vel_topic_;
  //bool use_moveit_;
  bool is_hardware_ok_;
  bool is_moving_;
  bool is_initialized_;

  double wheel_separation_;
  double wheel_radius_;

  trajectory_msgs::JointTrajectory *jnt_tra_msg_;

  double read_period_;
  double write_period_;
  double pub_period_;

  void setTiltCallback(const std_msgs::Float64::ConstPtr &msg);
  void setPanCallback(const std_msgs::Float64::ConstPtr &msg);
  void cameraStatePublish(void);
  int getBit(int n, int k);

 public:
  LoCoBotBaseController();
  ~LoCoBotBaseController();

  /**
   * \brief Callback for the joint_velocity_controller
   *
   * @param goal action goal
   */
  bool hardwareOK();
  bool hardwareStatusPrint();
  bool controlLoop(void);
  bool initWorkbench(const std::string port_name, const uint32_t baud_rate);
  bool getDynamixelsInfo(const std::string yaml_file);
  bool loadDynamixels(void);
  bool initDynamixels(void);
  bool stopDynamixels(void);
  bool initControlItems(void);
  bool initSDKHandlers(void);
  bool getPresentPosition(std::vector<std::string> dxl_name);

  double getReadPeriod() { return read_period_; }
  double getWritePeriod() { return write_period_; }
  double getPublishPeriod() { return pub_period_; }

  void initActionlib(void);
  void initPublisher(void);
  void initSubscriber(void);

  void initServer();

  void readCallback(const ros::TimerEvent &);
  void publishCallback(const ros::TimerEvent &);
  void hardwareStatusPublish(const ros::TimerEvent &);

  bool dynamixelCommandMsgCallback(dynamixel_workbench_msgs::DynamixelCommand::Request &req,
                                   dynamixel_workbench_msgs::DynamixelCommand::Response &res);
};

#endif //LOCOBOTBASE_CONTROLLERS_H
