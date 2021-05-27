#!/bin/sh
xterm -e "roslaunch turtlebot_gazebo turtlebot_world.launch world_file:=$(rospack find home-service-robot-world)/worlds/myworld.world" &
sleep 10
xterm -e "roslaunch turtlebot_gazebo amcl_demo.launch map_file:=$(rospack find home-service-robot-world)/maps/myworld_slam_map2.yaml" &
sleep 5
xterm -e "roslaunch turtlebot_rviz_launchers view_navigation.launch" &
sleep 5
xterm -e "rosrun pick_objects pick_objects"