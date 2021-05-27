#!/bin/sh
xterm -e "roslaunch turtlebot_gazebo turtlebot_world.launch world_file:=$CATKIN_WS/src/RoboND-home-service-robot/worlds/myworld.world" &
sleep 10
xterm -e "roslaunch turtlebot_gazebo amcl_demo.launch map_file:=$CATKIN_WS/src/RoboND-home-service-robot/maps/myworld_slam_map2.yaml" &
sleep 5
xterm -e "rosrun rviz rviz -d /$CATKIN_WS/src/RoboND-home-service-robot/rvizConfig/navigation.rviz" &
sleep 5
xterm -e "rosrun add_markers add_markers" &
sleep 5
xterm -e "rosrun pick_objects pick_objects" 


