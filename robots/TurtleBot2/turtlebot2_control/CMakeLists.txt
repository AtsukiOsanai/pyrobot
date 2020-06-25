################################################################################
# Set minimum required version of cmake, project name and compile options
################################################################################
cmake_minimum_required(VERSION 2.8.3)
project(turtlebot2_control)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

################################################################################
# Find catkin packages and libraries for catkin and system dependencies
################################################################################
find_package(catkin REQUIRED COMPONENTS
  roscpp
  sensor_msgs
  geometry_msgs
)

################################################################################
# Setup for python modules and scripts
################################################################################
catkin_python_setup()

################################################################################
# Declare ROS messages, services and actions
################################################################################

#add_service_files(
#  FILES
#  JointCommand.srv
#)

#generate_messages(
#  DEPENDENCIES
#  std_msgs
#)`

################################################################################
## Declare ROS dynamic reconfigure parameters
################################################################################

################################################################################
# Declare catkin specific configuration to be passed to dependent projects
################################################################################
catkin_package(
 # INCLUDE_DIRS include
 CATKIN_DEPENDS roscpp sensor_msgs geometry_msgs 
)
################################################################################

# Build
################################################################################
include_directories(
  ${catkin_INCLUDE_DIRS}
)

# add_executable(turtlebot2_control src/)
# add_dependencies(turtlebot2_control ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
# target_link_libraries(turtlebot2_control ${catkin_LIBRARIES})

#################################################################################
## Install
#################################################################################
# install(TARGETS locobotbase_control
#   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

#install(TARGETS turtlebot2_controller
#  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
#)

#install(DIRECTORY include/${PROJECT_NAME}/
#  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
#)

install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(PROGRAMS nodes/robot_teleop_server.py DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
install(PROGRAMS nodes/keyboard_teleop_client.py DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

################################################################################
# Test
################################################################################
