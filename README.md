# EECE5550 Mobile Robotics Final Project [Team Rocksteady] @ NEU
## Background
* #### Turtlebot3 SLAM with Cartographer
* #### Autonomous exploration using explore_lite.
* #### Apriltag detection using apriltag_ros.

## Introduction
#### This project involves the use of autonomous mobile robots to perform search and rescue operations in a simulated disaster area. The autonomous robot used for this project is Turtlebot3 burger and it is placed in an unknown area which is the simulated disaster environment populated with AprilTags that serve as the victim. The robot is expected to produce a complete grid of the environment in addition to providing the IDs and locations of the Apriltags. In order for the autonomous robot to accomplish this objective it is equipped with a 360o LiDAR scanner, necessary for localization and mapping, and a Raspberry Pi camera (RPi) to detect the AprilTags.

## Installation

To setup turtlebot3 please refer to the [ROBOTIS e-Manual for TurtleBot 3](https://emanual.robotis.com/docs/en/platform/turtlebot3/quick-start/)

Turtlebot3 [github](https://github.com/ROBOTIS-GIT/turtlebot3)

Turtlebot3 [Overview](https://emanual.robotis.com/docs/en/platform/turtlebot3/overview/)

### Installation of Dependencies
* Apriltag ROS package -- [apriltag_ros](http://wiki.ros.org/apriltag_ros)
```bash
sudo apt install ros-[distro]-aprtiltag-ros
```
* Explore Lite ROS package -- [explore_lite](http://wiki.ros.org/explore_lite)
```bash
sudo apt install ros-[distro]-explore-lite
```

* Cartographer ROS --> To install Cartographer refer to the official installation guide of [cartographer](https://google-cartographer-ros.readthedocs.io/en/latest/compilation.html)

### Workspace Installation
* Clone the rocksteady_apriltag workspace
```bash
cd [$Your_Destination_Folder]
git clone https://github.com/Abinav2695/rocksteady_apriltag.git
```
* Build workspace
```bash
cd rocksteady_apriltag/rocksteady_artags_slam
catkin_build
```

## Usage

Connect to turtlebot3 Raspberry Pi using ssh
```bash
ssh ubuntu@[IP_ADDRESS_OF_RASPI_ON_ROBOT]
```
Update the .bashrc with the IP addresses of both devices if not performed in the robot setup stage. 

On the robot, the following two
lines should be in the .bashrc:
```bash
export ROS_MASTER_URI=http://IP_ADDRESS_OF_REMOTE_PC:11311
export ROS_HOSTNAME=IP_ADDRESS_OF_RASPI_ON_ROBOT
```

The following two lines should be written on the remote PC.
```bash
export ROS_MASTER_URI=http://IP_ADDRESS_OF_REMOTE_PC:11311
export ROS_HOSTNAME=IP_ADDRESS_OF_REMOTE_PC
```

Run the camera video capture node on raspi via SSH.
```bash
roslaunch raspicam_node camerav2_1280x960_10fps.launch enable_raw:=true
```

Run the SLAM node on remote PC using the following command. This should start the autonomous exploration, apriltag detection and cartographer slam node.
```bash
cd <Workspace Path>/rocksteady_apriltag/rocksteady_artags_slam
source devel/setup.bash
roslaunch rocksteady_launch rocksteady_slam.launch
```

If you are using a gazebo simulation instead of the robot run the following command
```bash
cd <Workspace Path>/rocksteady_apriltag/rocksteady_artags_slam
source devel/setup.bash
roslaunch rocksteady_launch rocksteady_slam.launch configuration_basename:=rpcksteady_lds_2d_gazebo.lua
```
Once the turtlebot completes its task, you can save the map by running
```bash
rosrun map_server map_saver <FILEPATH>
```