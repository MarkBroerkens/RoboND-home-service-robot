[![Udacity - Robotics NanoDegree Program](https://s3-us-west-1.amazonaws.com/udacity-robotics/Extra+Images/RoboND_flag.png)](https://www.udacity.com/robotics) 
![ROS CI](https://github.com/MarkBroerkens/RoboND-home-service-robot/workflows/ROS%20CI/badge.svg)

# Home Service Robot
Simulation of 2-wheeled robot with *differential drive* that applies the gmapping for SLAM and . 




# Impressions
## The robot
![Robot](https://github.com/MarkBroerkens/RoboND-slam/blob/main/my_robot/images/mybot.png)

## The environment


## 3D Occupancy Grid Map


## RTABMap Database Viewer



### Directory Structure
```
.
├── LICENSE
├── my_robot
│   ├── CMakeLists.txt
│   ├── images                         # images for documentation
│   │   └── mybot.png
│   ├── launch                         # launch files
│   │   ├── maping.launch              # launch mapping
│   │   ├── localization.launch        # launch localization
│   │   ├── teleop.launch              # launch teleop
│   │   ├── robot_description.launch   # launch robot
│   │   └── world.launch               # launch world
│   ├── meshes                         
│   │   └── hokuyo.dae                 # mesh of lidar sensor
│   ├── package.xml                    # package info
│   ├── rviz                           # rviz configuration
│   │   └── default.rviz
│   ├── urdf                           # robot description files
│   │   ├── my_robot.gazebo
│   │   └── my_robot.xacro
└── README.md                          # this README.md file

```


### Steps to launch the simulation

#### Step 1 Update and upgrade the Workspace image
```sh
$ sudo apt-get update
$ sudo apt-get upgrade -y
```

#### Step 1a define the workspace location
replace the following data to the ~/.bashrc:
```sh
$ export ROS_IP=`echo $(/sbin/ifconfig -a | awk '/(cast)/ { print $2 }' | cut -d':' -f2 | head -1)`
```

add one of the following lines to the end of ~/.bashrc
```sh
$ export CATKIN_WS=$HOME/catkin_ws
```

or

```sh
$ export CATKIN_WS=/home/workspace/catkin_ws
```

```sh
source ~/.bashrc
```

#### Step 2 Create the catkin workspace
```sh
$ mkdir -p $CATKIN_WS/src
$ cd $CATKIN_WS/src
$ catkin_init_workspace
```


#### step 3 Install dependencies of packages in workspace
```sh
$ cd $CATKIN_WS
$ sudo apt-get install python-rosinstall 
$ sudo apt-get install xterm
$ rosinstall . https://raw.githubusercontent.com/MarkBroerkens/RoboND-home-service-robot/main/home-service-robot.rosinstall
$ rosdep install --from-paths src --ignore-src -r -y
```


#### Step 4 Compile the code
```sh
$ cd $CATKIN_WS
$ catkin_make
$ source devel/setup.bash
```


#### Step 5 Run the Simulation 
##### in Terminal 1
```sh
$ source $CATKIN_WS/devel/setup.bash
$ roslaunch my_robot world.launch gazebo_gui:=true

```
This will open Rviz and Gazebo. Omit the "gazebo_gui:=true" if you do not need the gazebo gui.

##### in Terminal 2
```sh
$ source $CATKIN_WS/devel/setup.bash
$ roslaunch my_robot teleop.launch

```
This will run the teleoperation mode.

##### in Terminal 3

```sh
$ source $CATKIN_WS/devel/setup.bash
$ roslaunch my_robot mapping.launch
```
This will run the RTAB mapping.




# License
MIT license

# Thanks to
* ros teleop for [https://github.com/ros-teleop/teleop_twist_keyboard](https://github.com/ros-teleop/teleop_twist_keyboard)
* Amazon AWS for its [Gazebo - Robomaker Small House World](https://github.com/aws-robotics/aws-robomaker-small-house-world)

# Further Reading
* [RTAB-Map Parameter Tutorial](http://wiki.ros.org/rtabmap_ros/Tutorials/Advanced%20Parameter%20Tuning)
* [List of RTAB-Map Parameters](https://github.com/introlab/rtabmap/blob/master/corelib/include/rtabmap/core/Parameters.h)
