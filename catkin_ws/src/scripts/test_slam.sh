#!/bin/sh
# Project 6, SLAM Testing shell script
# 1. launch turtlebot_world.launch to deploy a turlebot in the environment:
xterm -hold -e  " roslaunch turtlebot_gazebo turtlebot_world.launch" &
sleep 5
# 2. launch gmapping_demo.launch or run slam_gmapping to perform SLAM:
xterm -hold -e  " roslaunch turtlebot_gazebo gmapping_demo.launch" & 
sleep 5
# 3. launch view_navigation.launch to observe the map in rviz:
xterm -hold -e  " roslaunch turtlebot_rviz_launchers view_navigation.launch" &
sleep 5
# 4. lauch keyboard_teleop.launch to manually control the robot with keyboard commands:
xterm -hold -e  " roslaunch turtlebot_teleop keyboard_teleop.launch" 


