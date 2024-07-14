# Robot_Arm_ROS2
A ROS2 package designed for planning and executing motion trajectories of a robot arm in simulation, utilizing the Moveit plugin for applying kinematics.


These packages were tested under ROS2 Humble and Ubuntu 22.04 .

## Dependencies
make sure you installed all these packages:
```
sudo apt-get update && sudo apt-get install -y \
     ros-humble-joint-state-publisher-gui \
     ros-humble-gazebo-ros \
     ros-humble-xacro \
     ros-humble-ros2-control \
     ros-humble-moveit \
     ros-humble-ros2-controller \
     ros-humble-gazebo-ros2-control 
```
## Installation

1- Clone the repo:
```
https://github.com/Mjd0001/Robot_Arm_ROS2.git
```
2- Build the ROS 2 workspace
```
cd ~/your_ws
```
```
colcon build
```
3- Source the project
```
. install/setup.bash
```
## Usage
### Controlling the robot arm by joint_state_publisher
```
$ ros2 launch arduinobot_description display.launch.xml
```
![image](https://github.com/user-attachments/assets/683164ab-a99e-47db-96a4-0590e61944f3)

### Controlling the robot arm by Moveit and kinematics
run the following commands, each command in a separate terminal window:
```
ros2 launch arduinobot_description gazebo.launch.py
```
```
ros2 launch arduinobot_controller controller.launch.py
```
```
ros2 launch arduinobot_moveit moveit.launch.py
```
![image](https://github.com/user-attachments/assets/65746bab-c6e4-4580-9295-0283a7764b16)

