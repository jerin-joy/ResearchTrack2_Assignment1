# Research Track II - Assignment 1
## Assignment Description

The package contains the nodes and the simulation environment for controlling a mobile robot in the Gazebo simulation environment. 

The user is requested to press '1' to start the movement of the robot and robot will move to a random position. Using the implementation of an action server, the user is able to stop the robot by pressing '0'. Unless the robot is stopped by user, the robot continues to move in a loop to random positions within a given range by generating new targets.

## Package Description

```bash
├── launch 
│   ├── sim.launch              - simulation launch file   
│   ├── ros12_bridge.launch     - ros-ros2 bridge launch file
│   └── vrep.launch             - vrep launch file         
├── scripts
│   ├── go_to_point.py          - python script to control the robot
│   └── user_interface.py       - cmd line interface
├── src
│   ├── position_service.cpp    - random position generation
│   └── state_machine.cpp       - FSM CPP code
├── srv
│   ├── Command.srv             - user interface service
│   ├── Position.srv            - position service
│   └── RandomPosition.srv      - random position service 
├── urdf
|    └── my_robot.urdf          - mobile robot description
├── pioneerp3dx.ttt             - pioneer P3DX vrep scene file
├── CMakeLists.txt              - CMakeLists file
└── package.xml                 - package manifest

5 directories, 14 files
```

## Node Structure

The package consists of four nodes:
- **goToPoint**: The service server managing the robot speed control depending on the goal received.

- **userInterface**: It implements a simple command interface, which asks the user to start or stop the robot, and calls the service implemented in the FSM.
  
- **positionService**: It implements a random position service which generates a random pose as a response to a request. 
  
- **stateMachine**: The node which implements an action server to start or stop the robot, and calls the other two services to drive the robot. 

## Building and Running

To compile the workspace, navigate to the workspace and enter:
```
catkin_make
```
To launch the nodes which will communicate with the Coppeliasim simulation, please run:
```
roslaunch rt2_assignment1 vrep.launch
```
Before starting coppeliasim, an instance of ros master must be running. Coppeliasim must be started separately by navigating to the coppeliasim directory and then running:
```
./Coppeliasim.sh
```
