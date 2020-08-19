#!/bin/sh
# Project 6.9, Your Home Service Robot shell script
# 1. launch turtlebot (turtle_world.launch) to deploy a turlebot in the environment:
xterm -hold -e  " roslaunch turtlebot_gazebo turtlebot_world.launch" &
sleep 5
# 2. launch amcl(amcl_demo.launch) to localize your turtlebot:
xterm -hold -e  " roslaunch turtlebot_gazebo amcl_demo.launch" &
sleep 5
# 3. launch rviz config file (view_navigation.launch) to observe the map in rviz with markers:
xterm -hold -e  " roslaunch turtlebot_rviz_launchers view_navigation.launch" &
# 4. launch pick_objects node:
xterm -hold -e  " rosrun pick_objects pick_objects" &
# 5. launch add_markers node:
xterm -hold -e  " rosrun add_markers add_markers"
