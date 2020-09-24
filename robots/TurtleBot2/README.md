# TurtleBot2 with more stacked plate Navigation

## Requirements
- ROS version
  - melodic (recommended)
  - kinetic  
  Some packages can not work.
- CUDA 10.2 (recommended)
  - If you don't use ZED camera, you can neglect this dependency.

## Install
1. Copy the 'turtlebot2_install_all.sh' from TurtleBot2/install.
2. Make the workspace for your development (e.g. pyrobot_ws).
3. Run the install script with args.  

        $ ./turtlebot2_install_all.sh -t full -p 2 -d path/to/pyrobot_ws -l interbotix -z false 
    options:
    - t : Install option. Set "full" for sim and actual robot, and "sim_only" for simulation user.
    - p : Python version. (2 or 3)
    - d : Path to the install directory.
    - l : LoCoBot version. Please Set the "interbotix"
    - z : Use ZED camera or not.  
          We have two robots, i.e. myrobot and myrobot2.  
          Basically, we use myrobot2 which does not have ZED camera, so set this option to "false".

4. Load the setup to launch.

        $ cd path/to/pyrobot_ws/low_cost_ws 
        $ source devel/setup.bash

5. Activate virtual env for pyrobot Python API.

        $ source path/to/pyrobot_ws/pyenv_pyrobot_python2/bin/activate

## Navigation with hdl_graph_slam and hdl_localization on Gazebo
### Create a point cloud map
1. Launch main.launch without a prior map.

        $ roslaunch turtlebot2_control main.launch use_sim:=true use_2dmap:=false robot:=myrobot2 world_name:=sample use_lidar_slam:=true mapping_mode:=true

    options:
    - robot (str): robot name (turtlebot, myrobot, myrobot2, default: myrobot2)
    - world_name (str): gazebo world (default: sample)
    - use_sim (bool): If True, running with Gazebo simulation mode. (default: false)
    - use_lidar_slam (bool): Employing lidar slam. (default: true)
    - use_visual_slam (bool): Employing rtabmap visual slam. (default: false)
    - mapping_mode (bool): Running the mode to create the map for localization will run.  
                           False means localization mode and map --> odom /tf will be published by   localization node. (default: false)

    You can select playground, actor, willowgarage, car_demo, sample, sample_dynamic, and citysim.
    If your gazebo does not run up, please try the following command to set the gazebo path.

        $ source /usr/local/share/citysim/setup.sh

2. Launch the Rviz.

        $ roslaunch turtlebot2_rviz view_navigation.launch

3. Launch the teleop or Using 2D Nav Goal in Rviz for moving the robot.

        $ roslaunch turtlebot2_control keyboard_teleop.launch

   If you would like to use a joy, let's launch by following command.

        $ roslaunch turtlebot2_control joystick_teleop.launch

4. Launch hdl_graph_slam to create a global 3d map for localization.

        $ roslaunch turtlebot2_navigation hdl_graph_slam_imu.launch

5. Save the global 3d map.

        $ rosservice call /hdl_graph_slam/dump "destination 'path/to/3dmap'"
        $ rosservice call /hdl_graph_slam/save_map "utm: false resolution: 0.05 destination 'path/to/3dmap.pcd'"

   Please choose an appropriate resolution.

6. (Optional) Refine the 3d map by using the interactive slam package.

        $ roscore
        , and in another terminal, 
        $ rosrun interactive_slam interactive_slam

   Regarding the usage, please see the author's [github page](https://github.com/koide3/hdl_localization).

### Navigation with localization
1. Launch main.launch without a prior map.

        $ roslaunch turtlebot2_control main.launch use_sim:=true use_2dmap:=false robot:=myrobot2 world_name:=sample use_lidar_slam:=true mapping_mode:=false

    options:
    - robot (str): robot name (turtlebot, myrobot, myrobot2, default: myrobot2)
    - world_name (str): gazebo world (default: sample)
    - use_sim (bool): If True, running with Gazebo simulation mode. (default: false)
    - use_lidar_slam (bool): Employing lidar slam. (default: true)
    - use_visual_slam (bool): Employing rtabmap visual slam. (default: false)
    - mapping_mode (bool): If false, running the mode for localization mode.  
                           "map --> odom" /tf will be published by localization node. (default: false)
    - use_2dmap (bool): Applying 2d occupancy grid map created from mapping pkg. (default: false)  
                        NOTE: We have not implemented the script to make a 2d OGM from 3d point cloud map.
    - map_file (str): Path to the map yaml file. (default: $(find turtlebot2_gazebo)/maps/playground.yaml)  
                      NOTE: We have not implemented the script to make a 2d OGM from 3d point cloud map.

    You can select playground, actor, willowgarage, car_demo, sample, sample_dynamic, and citysim.

2. Launch the Rviz.

        $ roslaunch turtlebot2_rviz view_navigation.launch

3. Launch the teleop or Using 2D Nav Goal in Rviz for moving the robot.

4. Launch hdl_localization.

        $ roslaunch turtlebot2_navigation hdl_localization_imu.launch 3dmap:=/path/to/3dmap.pcd

    options:
    - 3dmap (str): Path to the 3d map pcd (default: $(find car_demo)/3dmap/sample_3dmap.pcd)


## Navigation with rtabmap_ros (Not recommended, we don't check the followings yet)
### Create a rtabmap database and occupancy grid map.
1. Launch main.launch without a prior map.

        $ roslaunch turtlebot2_control main.launch use_sim:=true use_2dmap:=false robot:=myrobot world_name:=playground

    options:
    - robot (str): robot name (turtlebot or myrobot, default: turtlebot)
    - world_name (str): gazebo world (default: playground)  
    You can select playground, actor, willowgarage, car_demo, and citysim.

2. Rviz

        $ roslaunch turtlebot2_rviz view_navigation.launch

3. Teleop

        $ roslaunch turtlebot2_control keyboard_teleop.launch

4. Rtabmap

        $ roslaunch turtlebot2_navigation rtabmap_mapping_demo.launch simulation:=true database_path:=~/rtabmap/playground.db

    options:
    - simulation (bool): Switch the topic names.
    - localization (bool): If false, rtabmap creates new database file. (default: false)
    - database_path (str): Path to rtabmap database. (default: ~/.ros/rtabmap.db)
    - rgb_topic (str): Topic name of rgb image.
    - depth_topic (str): Topic name of depth image. Depth image must be transformed into rgb frame.
    - cloud_topic (str): Topic name of point cloud.
    - camera_info_topic (str): Topic name of rgb image.

5. Create a map with teleoperation.  
6. Save the occupancy grid map.
This is for move_base prior map.

        $ rosrun map_server map_saver -f path/to/playground map:=/rtabmap/map

    You can see playground.yaml and playground.pgm under /path/to/.

### Navigation with localization based on created prior map.
1. Launch main.launch **with** the prior map.

        $ roslaunch turtlebot2_control main.launch use_sim:=true use_map:=true use_vslam:=true robot:=myrobot world_name:=playground map_file:=/path/to/playground.yaml

    options:
    - use_map (bool): If true, move_base loads the prior map specified by map_file option.
    - map_file (str): Map yaml file created through rtabmap.
    - use_vslam (bool): Enabling planning with pyrobot API. If false, the robot moves with relying on controller only.

2. Rviz
Please change config to myrobot_navigation.rviz.

        $ roslaunch turtlebot2_rviz view_navigation.launch

3. Rtabmap

        $ roslaunch turtlebot2_navigation rtabmap_mapping_demo.launch simulation:=true database_path:=~/rtabmap/playground.db localization:=true 

    options:
    - localization (bool): If true, rtabmap loads an existing database file. (default: false)
    - database_path (str): Path to rtabmap database. (default: ~/.ros/rtabmap.db)

4. Looking aroung by teleop

        $ roslaunch turtlebot2_control keyboard_teleop.launch

    Press j or l button to localize. After localization is converged, stop the teleop launch.

5. Navigate with 2D Nav Goal in Rviz
Set the goal 6d pose by 2D Nav Goal in Rviz.
6. Navigate with PyRobot API
- We exemplify iPython script to navigate the robot.
- For more details, see the [pyrobot navigation document](https://www.pyrobot.org/docs/navigation).

    ```bash
    from pyrobot import Robot

    robot = Robot('turtlebot2')

    # give a relative goal
    x_rel, y_rel, z_rel = 3.0, 1.0, 0.0
    robot.base.go_to_relative([x_rel, y_rel, z_rel], close_loop=True, smooth=True, use_map=True)

    # give an absolute goal
    x_abs, y_abs, z_abs = 10.0, -4.0, 0.0
    robot.base.go_to_absolute([x_abs, y_abs, z_abs], close_loop=True, smooth=True, use_map=True)
    ```
