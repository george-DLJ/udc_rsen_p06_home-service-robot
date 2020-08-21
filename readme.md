# Home Service Robot
Simulate a home service robot that will autonomously map an environment and navigate to pickup and deliver objects.

## Table of contents
* [General info](#general-info)
* [Technologies](#technologies)
* [Ros Packges used](#ros-packages-used)
* [Setup](#setup)

## General info
This is the project of the 6th Part of the Udacity Robotic Software Engineer Nanodegree.
Tasks in this project:
* Simulation Setup
* SLAM Testing
* Wall Follower Node
* Navigation Testing
* Waypoint Node
* Virtual Objects
* Put all together

## Technologies
* ROS
* Gazebo
* Ubuntu
* C++

## Ros Packges used
In this project I've imported and installed following ROS packages (kinetic version):

1. **gmapping package**
This package  file performs SLAM(Simultaneous Localization and Mapping) and builds a map of the environment using a robot equipped with lidar sensors or RGB-D cameras.
NOTICE: this package is not employed on the final version of this project but it was very helpful on the development and test phase.
Package name: *turtlebot_gazebo*
Launch file: *gmapping_demo.launch*
See [ROS gmapping](http://wiki.ros.org/gmapping)

2. **turtlebot_teleop package** 
This package allows manual control of the robot. Use keyboard_teleop.launch file to control the robot using joystic or keyboard commands.
NOTICE: this package is not employed on the final version of this project but it was very helpful on the development and test phase. There is an example using this package on  **test_slam.sh** on the *scripts* folder
Package Name: *turtlebot_teleop*
Launch file: *keyboard_teleop.launch*
See [ROS teleop](http://wiki.ros.org/turtlebot_teleop)

3. **turtlebot_rviz_launchers**
This package allows you to load preconfigured rviz workspaces. With the *view_navigation.launch* file, you can load a preconfigured rviz workspace. Tis file will automatically load the robot model, trajectories, and map.
Package Name: *turtlebot_rviz_launchers*
Launch file: *view_navigation.launch*
See [ROS rviz launchers](http://wiki.ros.org/turtlebot_rviz_launchers)

4. **turtlebot_gazebo** 
This package contains an already built turtlebot with its sensors that can be deployed in a gazebo environment by linking a world file.
The *amcl_demo.lauch* allows the robot to localize itself and recognize poses.
Package Name: *turtlebot_gazebo*
Launch files: 
   - *turtlebot_world.launch*
   - *amcl_demo.launch*
See [ROS gmapping](http://wiki.ros.org/turtlebot_gazebo)

5. **Custom nodes**
NOTICE: remember that nodes require the creation of a package with dependencies.
    **pick_objects:** this node commands your robot to drive to a pickup zone and afterwards to a drop off zone.
    **add_markers:** this node adds a marker on the rviz workspace. It simulates the pick up and drop off of an object.

How to use them:
- To run packages lauch files use following command: 
```
   $roslaunch <package_name> <launch_file>
```
- To start nodes use following command: 
```
   $rosrun <package> <node_cpp_file>
```
for further details see *shell files* on */script* folder.

## Setup
NOTICE: this project has been tested only on Udacity Workspace environment. It may be necessary to install additional packages if you want to build on your own system. 
Steps to setup this project: 
1. update and upgrade working environment:
```
$ sudo apt-get update && apt-get upgrade
```
2. clone this repository

After each workspace new start:
3. install rospkg: 
```
$ cd /home/workspace/udc_rsen_p06_home-service-robot/catkin_ws
$ pip install rospkg
```

4. then set up the ROS envirnoment.

```
$ cd /home/workspace/udc_rsen_p06_home-service-robot/catkin_ws
$ catkin_make
```
or if the catkin_ws folder is not directly on the /home/workspace/ (you have a subfolder)
```
$ catkin_make --source . 
$ ... (TODO: add commands to start roslaunch...)
```

5. source devel/setup.bash
```
$ cd /home/workspace/udc_rsen_p06_home-service-robot/catkin_ws
$ source devel/setup.bash
```

6. run home-service-robot shell:
```
$ cd /home/workspace/udc_rsen_p06_home-service-robot/catkin_ws
$ src/scripts/home_service.sh
```



