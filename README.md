[![Udacity - Robotics NanoDegree Program](https://s3-us-west-1.amazonaws.com/udacity-robotics/Extra+Images/RoboND_flag.png)](https://www.udacity.com/robotics) 
![ROS CI](https://github.com/MarkBroerkens/RoboND-home-service-robot/workflows/ROS%20CI/badge.svg)

# Home Service Robot
Simulation of 2-wheeled robot with *differential drive* that applies gmapping for SLAM and demonstrates the following features: 

* Build a map of the environment using gmapping and teleop.
* Use Adaptive Monte Carlo Localisation to detect the robot position within the known map.
* Use the ROS move_base library to plot a path to a target pose and navigate to it.
* Write a custom node to publish goal poses for the robot.
* Write a custom node to visualize virtual objects in rviz that are transported by the robot.

![Robot](https://github.com/MarkBroerkens/RoboND-home-service-robot/blob/main/images/turtlebot.gif)



# Directory Structure of catkin workspace src folder 
```
├── RoboND-home-service-robot
│   ├── add_markers                        # PACKAGE add_markers
│   │   ├── CMakeLists.txt
│   │   ├── package.xml
│   │   └── src
│   │       └── add_markers.cpp
│   ├── home-service-robot.rosinstall
|   ├── home-service-robot-world           # PACKAGE home-service-robot-world
│   │   ├── CMakeLists.txt
│   │   ├── package.xml
│   │   ├── maps                           # maps
│   │   │   ├── myworld_slam_map2.pgm      # maps created using gmapping slam
│   │   │   └── myworld_slam_map2.yaml
│   │   │   ├── myworld_map.pgm            # maps created using pgm_map_creator
│   │   │   └── myworld_map.yaml
│   │   ├── rvizConfig                     # rviz config with marker
│   │   │   └── navigation.rviz
│   │   └── worlds                         # gazebo worlds
│   │       └── myworld.world
│   ├── pick_objects                       # PACKAGE pick_objects
│   │   ├── CMakeLists.txt
│   │   ├── package.xml
│   │   └── src
│   │       └── pick_objects.cpp
│   ├── LICENSE
│   ├── README.md
│   ├── scripts                            # shell scripts
│   │   ├── add_markers.sh
│   │   ├── home_service.sh
│   │   ├── pick_objects.sh
│   │   ├── test_navigation.sh
│   │   └── test_slam.sh
├── slam_gmapping
│   │  ..
├── turtlebot
│   │  ..
├── turtlebot_interactions
│   │  ..
└── turtlebot_simulator
    │  ..

```


# How to Install
## Prerequisites
* Ubuntu 16.04 with ROS Kinetic. I used the Udacity RoboND Virtual Machine whcih you can download [here](https://s3-us-west-1.amazonaws.com/udacity-robotics/Virtual+Machines/Lubuntu_071917/RoboVM_V2.1.0.zip)
* I configured 4GB memory for graphics acceleration in VMWare Fusion

## Step 1: Update and upgrade the Workspace image
```sh
# update the distribution including the mesa graphical drivers. Required in order to avoid crashes in rviz
sudo add-apt-repository ppa:ubuntu-x-swat/updates
sudo apt-get update
sudo apt-get dist-upgrade -y
sudo apt-get upgrade -y
```

## Step 2 define environment variables 
Replace the following data to the ~/.bashrc so that the ROS_IP gets a single value:
```sh
export ROS_IP=`echo $(/sbin/ifconfig -a | awk '/(cast)/ { print $2 }' | cut -d':' -f2 | head -1)`
```

Add one of the following lines to the end of ~/.bashrc.
```sh
export CATKIN_WS=$HOME/catkin_ws
```

or

```sh
export CATKIN_WS=/home/workspace/catkin_ws
```

```sh
source ~/.bashrc
```



## Step 3 Create the catkin workspace
```sh
mkdir -p $CATKIN_WS/src
cd $CATKIN_WS/src
catkin_init_workspace
```


## step 4 Install dependencies in workspace
```sh
cd $CATKIN_WS
sudo apt-get install python-rosinstall 
sudo apt-get install xterm
rosinstall . https://raw.githubusercontent.com/MarkBroerkens/RoboND-home-service-robot/main/home-service-robot.rosinstall
rosdep install --from-paths src --ignore-src -r -y
```


## Step 5 Compile the code
```sh
cd $CATKIN_WS
catkin_make
source devel/setup.bash
```


# Run the code
## Test Slam
The ```test_slam.sh``` script launches Gazebo with the home environment, places a turtlebot in it and launches the slam_gmapping nodes. The user can move the robot in the environment using keyboard as in turtlebot_teleop. The map is saved by invoking ```rosrun map_server map_saver -f $CATKIN_WS/src/RoboND-home-service-robot/maps/myworld_slam_map2```

![Test Slam](https://github.com/MarkBroerkens/RoboND-home-service-robot/blob/main/images/test_slam.gif)


## Test Navigation
The ```test_navigation.sh``` script uses the creted map and enables the robot to reach a goal pose designated by the user in RViz through the amcl package.

![Test Navigation](https://github.com/MarkBroerkens/RoboND-home-service-robot/blob/main/images/test_navigation.gif)


## Pick Objects
The ```pick_objects.sh``` script assigns two goal poses to the robot consecutively as the robot traverses through the environment to simulate picking up objects from a location and dropping them off to another location.

![Pick Objects](https://github.com/MarkBroerkens/RoboND-home-service-robot/blob/main/images/pick_objects.gif)


## Add Markers
The ```add_markers.sh``` script adds markers to the goal positions at constant times programmatically as the robot moves through the environment.

![Add Markers](https://github.com/MarkBroerkens/RoboND-home-service-robot/blob/main/images/add_markers.gif)


## Home Service
The script ```home_service.sh``` simulates the transport of a virtual object by the robot. 
* The ```pick_objects``` is responsible for the provisioning of the transport of the  virtual vehicles: It specifies the pickup goal position and enables the visualization of a marker at the pickup goal position . The actual visualization of the marker is handled by the ```add_markers``` node. It shows a marker at the navigation goal position if ```show_markers=true```.
* The robot then navigates to the pickup goal position. 
* The ```pick_objects``` node waits for the robot to navigate to the pickup position, waits 5 seconds and disables the visibility of the marker by sending ```show_markers=false``` to the ```add_markers``` node. This simulates the loading of the virtual object.
* The ```pick_objects``` node then sets the dropoff goal position and waits for the robot to navigate to that dropoff goal.
* On arrival of the robot at the dropoff goal position the ```pick_objects``` node enables the marker by sending ```show_markers=true``` to the ```add_markers``` node.

![Home Service](https://github.com/MarkBroerkens/RoboND-home-service-robot/blob/main/images/home_service.gif)



# Additional workflows
## Alternative method for creating map from gazebo world
```sh
sudo apt-get install libignition-math2-dev protobuf-compiler
cd $CATKIN_WS/src
git clone https://github.com/udacity/pgm_map_creator.git
cd $CATKIN_WS
catkin_make
cp $CATKIN_WS/src/RoboND-home-service-robot/world/myworld.world src/pgm_map_creator/world/myworld.world
```
at end of world file fill in (yust before the </world> tag)
```xml
<plugin filename="libcollision_map_creator.so" name="collision_map_creator"/>
```
```sh
gzserver src/pgm_map_creator/world/myworld.world
```

```sh
cd $CATKIN_WS
source devel/setup.bash
roslaunch pgm_map_creator request_publisher.launch
```

Note: you might need to tweak the ```initial_pose_a``` parameter in the launch config in order to help the robot to find its initial position.


# License
MIT license


# Further Reading
* [Tutorial. Markers Basic Shapes](http://wiki.ros.org/rviz/Tutorials/Markers%3A%20Basic%20Shapes)
* [Tutorial. Sending Goals to the Navigation Stack](http://wiki.ros.org/navigation/Tutorials/SendingSimpleGoals)
* [Tutorials. Navigation](http://wiki.ros.org/navigation/Tutorials)

