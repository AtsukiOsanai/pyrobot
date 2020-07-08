# TurtleBot2 with more stacked plate Navigation

## Install
1. Copy the 'turtlebot2_install_all.sh' from TurtleBot2/install.
2. Make the workspace for your development (e.g. pyrobot_ws).
3. Run the install script with args.  

        $ ./turtlebot2_install_all.sh -t full -p 2 -d path/to/pyrobot_ws -l interbotix

4. Load the setup to launch.

        $ cd path/to/pyrobot_ws/low_cost_ws
        $ source devel/setup.bash

5. Activate virtual env for pyrobot Python API.

        $ source path/to/pyrobot_ws/pyenv_pyrobot_python2/bin/activate

## Navigation with rtabmap_ros
### Create a rtabmap database and occupancy grid map.
1. Launch main.launch without a prior map.

        $ roslaunch turtlebot2_control main.launch use_sim:=true use_map:=false robot:=myrobot world_name:=playground

    options:
    - robot (str): robot name (turtlebot or myrobot, default: turtlebot)
    - world_name (str): gazebo world (default: playground)  
    You can select playground, actor, willowgarage, car_demo, and citysim.

2. Rviz
Please change config to myrobot_navigation.rviz.

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
