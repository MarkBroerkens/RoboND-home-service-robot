#!/bin/sh
xterm -e "roslaunch turtlebot_gazebo turtlebot_world.launch world_file:=$(rospack find home-service-robot-world)/worlds/myworld.world" &
sleep 10
xterm -e "roslaunch turtlebot_gazebo amcl_demo.launch map_file:=$(rospack find home-service-robot-world)/maps/myworld_slam_map2.yaml" &
sleep 5
xterm -e "rosrun rviz rviz -d $(rospack find home-service-robot-world)/rvizConfig/navigation.rviz" &
sleep 5
xterm -e "rosrun add_markers add_markers" &
sleep 5


# Publish the marker at the pickup zone
for x in 0.5 1 1.5 2.0
do
    echo "new load to be transported ($x, 0.5) -> (-$x, 0.5)"
    echo "pickup at ($x, 0.5)"
    rostopic pub -1 /move_base_simple/goal geometry_msgs/PoseStamped "{header: {stamp: now, frame_id: 'map'}, pose: {position: {x: $x, y: 0.5, z: 0.0}, orientation: {w: 1.0}}}" &
    sleep 15
    rostopic pub -1 /show_marker std_msgs/Bool true &
    # Pause 5 seconds
    sleep 5
    # Hide the marker
    rostopic pub -1 /show_marker std_msgs/Bool false &
    # Pause 5 seconds
    sleep 5
    # Publish the marker at the drop off zone
    echo "dropoff at (-$x, 0.5)"
    rostopic pub -1 /move_base_simple/goal geometry_msgs/PoseStamped "{header: {stamp: now, frame_id: 'map'}, pose: {position: {x: -$x, y: 0.5, z: 0.0}, orientation: {w: 1.0}}}" &
    sleep 15
    rostopic pub -1 /show_marker std_msgs/Bool true &
    
done