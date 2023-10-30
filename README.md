# ros2_pm_robot_control
## 1. Overview
This repository contains multiple ROS2 packages that can be used to interact with a robot (enviroment) in ROS2. Specifically, the packages have been designed to be used for precision assembly tasks.
The repository contains the following packages:
* `object_spawner_manager`: This package can be used to spawn objects (parts for assembly) in the ROS environment and to interact with them. A more detailed description can be found in the package.readme.
* `spawn_object_interfaces`: Contains the message and service definitions used by the object_spawner_manager.
* `pm_moveit_server`: This package can be used to interact with the move_groups (see Moveit2 doc for more information) that are launched from the pm_robot_bringup package (see match_pm_robot repository). The pm_moveit_server provides multiple services to control the pm_robot using the inverse kinematics. A more detailed description can be found in the package.readme.
* `pm_moveit_interfaces`: Contains the message and service definitions used by the pm_moveit_server.

## 2. Installation 
To run all the packages in this repository, the installation of ROS2 (tested on Humble) is mandatory! 
Before you attempt the installation for this repository, the full installation of the "match_pm_robot" repository is recommended.
* Make sure you have the following packages installed (only mandatory to run the pm_moveit_server)!
```
sudo apt install ros-humble-moveit
```
```
sudo apt install ros-humble-moveit-visual-tools
```
```
sudo apt install ros-humble-graph-msgs
```
* Open a terminal and change directory to your workspace source!
* Clone the repository
```
git clone https://github.com/match-PM/ros2_pm_robot_control.git
```
* Change to your workspace folder. Build your workspace:
```
colcon build 
```
