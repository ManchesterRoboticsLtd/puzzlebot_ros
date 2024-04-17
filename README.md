#     Puzzlebot Ros2 package

This package contains puzzlebot demo pyhon files, gazebo simulation files and launch files.


##   Dependencies


### Ros2
Install Ros2 [Humble](https://docs.ros.org/en/humble/) or [Foxy](https://docs.ros.org/en/foxy/) according to the ubuntu version.
  
  
### Gazebo Sim
Install Gazebo sim [Garden](https://gazebosim.org/docs/garden/install_ubuntu) or [Fortress](https://gazebosim.org/docs/fortress/install_ubuntu) acording to the Ros2 version
  

### Gazebo ros bridge
Install [Gazebo Ros bridge](https://github.com/gazebosim/ros_gz) either from binary or source. Check table for compatibility. 
  
For Ros2 `Humble` and Gazebo `Garden` only from source (clone repository into your ros workspace):

        git clone https://github.com/gazebosim/ros_gz.git


### Aruco detection
Install [Aruco opencv ros](https://github.com/fictionlab/ros_aruco_opencv) for Ros2.

        sudo apt install ros-humble-aruco-opencv



##   Puzzlebot Ros2 package usage

### Setup

Clone repository into your ros workspace source directory:

        git clone https://github.com/ManchesterRoboticsLtd/puzzlebot_ros.git

To build it open a terminal window in your Ros2 workspace and run:

        colcon build --packages-select puzzlebot_ros --symlink-install

Set the Gazebo sim resource and plugin paths environment variables (can be added in .bashrc for convenince):

        export GZ_SIM_RESOURCE_PATH=<gazebo_resource_path>
        export GZ_SIM_SYSTEM_PLUGIN_PATH=<gazebo_resource_path>/plugins

The `gazebo_resource_path` should be the gazebo folder in the puzzlebot_ros directory (can be moved anywhere on your hard-drive)



## Run test apps

To run in simulation mode launch the gazebo sim first:

        ros2 launch puzzlebot_ros aruco_gazebo.launch.py

The following gazebo sim launch files can be used:

* `aruco_gazebo.launch.py`: launches gazebo with an aruco world and a camera puzzlebot;
* `gazebo_empty_world.launch.py`: launches gazebo with an empty walled world and a camera puzzlebot;
* `gazebo_box_world.launch.py`: launches gazebo with a puzzlebot robot suspended on a box.

In another terminal you can run the puzzlebot test apps:

        ros2 launch puzzlebot_ros goto_kalman.launch.py

Or run python executable:

        ros2 run puzzlebot_ros velocity_control
        

To run tests on the real robot you need to launch the camera with aruco node first (only for kalman filter):

        ros2 launch puzzlebot_ros aruco_jetson.launch.py

The following launch files and apps are available:

* `goto_kalman.launch.py`: launches goto_point with kalman filter;
* `goto_deadreckoning.launch.py`: launches goto_point with deadreckoning odometry (does not require camera);
* `pwm_control`: sends pwm signals to the wheels and plots the results;
* `velocity_control`: implements a PID velocity controller for the wheels;
* `distance_control`: cascade control with laser sensor for keeping a constant distance to an obstacle.


The following topics are used for commuication with the robot:
* `ControlR`: right wheel pwm signal (when control_input=3 on the robot);
* `ControlL`: left wheel pwm signal (when control_input=3 on the robot);
* `VelocitySetR`: right wheel angular velocity setpoint (when control_input=2 on the robot);
* `VelocitySetL`: left wheel angular velocity setpoint (when control_input=2 on the robot);
* `cmd_vel`: linear and angular velocity setpoints (when control_input=1 on the robot);
* `VelocityEncR`: right wheel angular velocity;
* `VelocityEncL`: left wheel angular velocity;
* `robot_vel`: linear and angular velocities of the robot;


