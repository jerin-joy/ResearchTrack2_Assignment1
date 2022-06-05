# Research Track II - Assignment 1

Software documentation can be found [here](https://jerin-joy.github.io/ResearchTrack2_Assignment1/)
## Assignment Description

The package contains the nodes and the simulation environment for controlling a mobile robot in the Gazebo simulation environment. 

The task of this assignment is to replace the User Interface of the [first assignment](https://github.com/jerin-joy/ResearchTrack2_Assignment1) in the Research Track 2 course by a jupyter notebook interface. It involves the control of a holonomic robot by using Jupyter Notebook widgets and monitors the data obtained by using various graphical tools in matplotlib.

More specifically, the first part of this Jupyter Notebook interface is able of:
- Starting or stopping the robot "random position" behaviour by using two buttons - Start and Stop.
- Setting the linear and angular velocities of the robot by using two sliders.
- Directly controlling the robot movements by using 5 buttons (Forward, Turn right, Backward, Turn Left and Stop)

Also the second part of this jupyter notebook interface displays:
- A line plot for visualizing cmd_vel vs. actual velocity (for linear and angular velocity)
- A bar plot displaying the number of reached targets and cancelled targets.
- A hist plot showing the time required to reach targets.
- An xy graph showing the robot’s position and the orientation.
## Package Description

```
├── action
│   └── Move.action             - action file
├── launch 
│   └── sim.launch              - simulation launch file             
├── scripts
│   ├── go_to_point.py          - python script to control the robot
│   └── user_interface.py       - command line interface
├── src
│   ├── position_service.cpp    - random position generation
│   └── state_machine.cpp       - cpp code for FSM
├── srv
│   ├── Command.srv             - user interface service
│   ├── Position.srv            - position service
│   └── RandomPosition.srv      - random position service 
├── urdf
|    └── my_robot.urdf          - mobile robot description
├── CMakeLists.txt              - CMakeLists file
└── package.xml                 - package manifest

6 directories, 12 files
```

## Node Structure

![Package Tree](action_server.png)

The package consists of four nodes:
- **goToPoint**: The node implements an action server to
drive a robot toward a point in the environment.

- **userInterface**: It implements a simple command interface, which asks the user to start or stop the robot, and calls the service implemented in the FSM.
  
- **positionService**: It implements a random position service which generates a random pose as a response to a request. 
  
- **stateMachine**: The node which implements an action server to start or stop the robot, and calls the other two services to drive the robot. 

## Notebook
The implementation of the interface of the assignment in Jupyter Notebook can be found in the home page as the name [Robot Control.ipynb](Robot%20Control.ipynb) 

## Building and Running

To compile the workspace, navigate to the workspace and enter:
```
catkin_make
```

To launch the node, please run:
```
roslaunch rt2_assignment1 sim.launch
```

