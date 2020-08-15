# Home Service Robot
Simulate a home service robot that will autonomously map an environment and navigate to pickup and deliver objects.

## Table of contents
* [General info](#general-info)
* [Technologies](#technologies)
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

	
## Setup
To run this project, update and upgrade working environment:
```
$ sudo apt-get update && apt-get upgrade
```
clone this repository


then set up the ROS envirnoment.

```
$ cd ../catkin_ws
$ catkin_make
```
or if the catkin_ws folder is not directly on the /home/workspace/ (you have a subfolder)
```
$ catkin_make --source . 
$ ... (TODO: add commands to start roslaunch...)
```


