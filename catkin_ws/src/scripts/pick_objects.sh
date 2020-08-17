#!/bin/sh
# Project 6, Navigation Goal (pick objects) shell script
# 1. Add turtlebot_world.launch to deploy a turlebot in the environment:
xterm -hold -e  " roslaunch turtlebot_gazebo turtlebot_world.launch" &
sleep 5
# 2. Add amcl_demo.launch to localize your turtlebot:
xterm -hold -e  " roslaunch turtlebot_gazebo amcl_demo.launch" &
sleep 5
# 3. Add view_navigation.launch to observe the map in rviz:
xterm -hold -e  " roslaunch turtlebot_rviz_launchers view_navigation.launch" &
# 4. Add (your) pick_objects node:
xterm -hold -e  " rosrun pick_objects pick_objects"
