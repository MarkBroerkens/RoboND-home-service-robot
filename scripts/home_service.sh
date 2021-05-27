#!/bin/sh
xterm -e "roslaunch turtlebot_gazebo turtlebot_world.launch world_file:=$(rospack find home-service-robot-world)/worlds/myworld.world" &
sleep 10
xterm -e "roslaunch turtlebot_gazebo amcl_demo.launch map_file:=$(rospack find home-service-robot-world)/maps/myworld_slam_map2.yaml" &
sleep 5
xterm -e "rosrun rviz rviz -d $(rospack find home-service-robot-world)/rvizConfig/navigation.rviz" &
sleep 5
xterm -e "rosrun add_markers add_markers" &
sleep 5
xterm -e "rosrun pick_objects pick_objects" 


