# Robot_Arm_ROS2
A ROS2 package designed for planning and executing motion trajectories of a robot arm in simulation and real-life, utilizing the Moveit plugin for applying kinematics.


These packages were tested under ROS2 Humble and Ubuntu 22.04 .

## Dependencies
make sure you installed all these packages:
```
sudo apt-get update && sudo apt-get install -y \
     ros-humble-joint-state-publisher-gui \
     ros-humble-gazebo-ros \
     ros-humble-xacro \
     ros-humble-ros2-control \
     ros-humble-ros2-controllers \
     ros-humble-joint-state-broadcaster \
     ros-humble-joint-trajectory-controller \
     ros-humble-controller-manager
     ros-humble-moveit \
     ros-humble-gazebo-ros2-control
```
## Installation

1- Clone the repo in src folder of your workspace:
```
cd ~/ros2_ws/src
```
```
https://github.com/smart-methods/Robot_Arm_ROS2.git
```
2- Install all dependencies, then build
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
ros2 launch arduinobot_description display.launch.xml
```
![image](https://github.com/user-attachments/assets/683164ab-a99e-47db-96a4-0590e61944f3)

You can also connect with hardware by running on another terminal:
```
ros2 run arduinobot_description joint_trajectory.py --ros-args -p port:=/dev/ttyUSB0
```
Make sure the port is enabled 
```
sudo chmod 777 /dev/ttyUSB0
```
#### Simulation
```
ros2 launch arduinobot_description display.launch.xml
```
```
ros2 launch arduinobot_description simulation.launch.py 
```
```
ros2 run arduinobot_description joint_trajectory.py
```
### Controlling the robot arm by Moveit and kinematics
```
ros2 launch  arduinobot_mc demo.launch.py 
```
You can also connect with hardware by running on another terminal:
```
ros2 run arduinobot_description joint_trajectory.py --ros-args -p port:=/dev/ttyUSB0
```
Make sure the port is enabled 
```
sudo chmod 777 /dev/ttyUSB0
```
#### Simulation
Run the following commands, each command in a separate terminal window:
```
ros2 launch arduinobot_description simulation.launch.py 
```
```
ros2 launch  arduinobot_mc demo.launch.py
```
![image](https://github.com/user-attachments/assets/65746bab-c6e4-4580-9295-0283a7764b16)

