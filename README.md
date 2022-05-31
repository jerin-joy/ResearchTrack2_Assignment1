# Research Track II - Assignment 1
## Assignment Description

The package contains the nodes and the simulation environment for controlling a mobile robot in the Gazebo simulation environment. 

In particular, the ROS2 package described here can communicate with the ROS1 nodes in order to control a non holonomic robot in a Gazebo environment. The user is requested to press '1' to start the movement of the robot and robot will move to a random position. Using the implementation of an action server, the user is able to stop the robot by pressing '0'. Unless the robot is stopped by user, the robot continues to move in a loop to random positions within a given range by generating new targets.

## Package Description

```bash

├── launch
│   └── sim.py                      - nodes launch file
├── src
│   ├── position_service.cpp        - returns random position
│   └── state_machine.cpp           - manages finite state machine logic
├── srv
│   ├── Command.srv                 - user interface service
│   ├── Position.srv                - position service
│   └── RandomPosition.srv          - random position service
├── my_mapping.yaml                 - mapping file used to link ROS and ROS2
├── package.xml                     - package manifest
├── README.md                       - ReadMe file
└── CMakeLists.txt                  - CMakeLists file

3 directories, 10 files

```

## Node Structure

![Package Tree](action_server.png)

This package consists of mainly two nodes:

- **positionService**: It implements a random position service which generates a random pose as a response to a request. 
  
- **stateMachine**: The node which implements an action server to start or stop the robot, and calls the other two services to drive the robot. 

## Building and Running

1. Create three bash script files ros.sh, ros2.sh and ros12.sh in the root folder for sourcing ROS1 workspace, ROS2 workspace and ROS 1 and 2 workspace together respectively.

2. Open a new terminal window and source the ROS1 workspace by entering:

```
source ros.sh
```
3. In another terminal window, source the ROS2 workspace:

```
source ros2.sh
```
4. In another terminal window, source the ROS12 workspace:

```
source ros12.sh
```
5. If the ROS1 bridge package is not present in the ROS2 workspace, add the bridge package by cloning:

```
git clone https://github.com/ros2/ros1_bridge.git
```
6. After cloning the ROS1 bridge package, in the ROS2 sourced terminal, build the ROS2 workspace by:

```
colcon build --symlink-install --packages-skip ros1_bridge
```
7. For compiling the ROS1 bridge package, open the ROS12 sourced terminal, build the workspace by:

```
colcon build --packages-select ros1_bridge --cmake-force-configure
```

8. For launching the file, open the ROS1 sourced terminal and enter:

```
roslaunch rt2_assignment1 ros12_bridge.launch
```
9. Open the ROS2 sourced terminal and run:

```
ros2 run ros1_bridge dynamic_bridge
```
10. Open the ROS12 sourced terminal and run:

```
ros2 launch rt2_assignment1 sim_container.py
```
