# Research Track 1 - Final assignment
The purpose of this project is to develop a software architecture for the control of the robot in a 3D simulation.
This control consists in locating the robot and planning the movement towards a specific target.
For both localization and planning we rely on the packages 'move_base' and 'gmapping'.
## Compatibility
The project is developed using ROS Noetic on Ubuntu 20.04.2.
For this reason, the project is compatible with ROS Noetic.
On top, it has never been tested on ROS Kinetic, and probably it doesn't work there.
## Expected behaviour
As said previously, this project aims to control a mobile robot into a 3D environment.
The robot is controlled by command line interface by the user.
In other words, the user can remotely control the robot.
According to this aspect, the user can choose which action, among the implemented ones, the robot should perform.
The actions that can be given to the robot are:
1. move to a pseudo-random target, i.e. moving towards one of following target positions [(-4,-3);(-4,2);(-4,7);(5,-7);(5,-3);(5,1)]
2. reach a specific target entered by the user, after checking that it is one of the above six targets
3. follow the wall within the environment
4. stop in the last position reached by the robot
5. (optional) chage the planning algorithm in use (default one is the Dijkstra's shortest path algorithm, while the Bug0 algorithm can be implemented afterwards)
# Description of the package
This package is composed by four nodes and two servers.

The nodes are identifiable through the files inside the src folder, and so we have:
* [commands.cpp](https://github.com/RiccardoZuppetti/final_assignment/blob/main/src/commands.cpp)
* [random_number.cpp](https://github.com/RiccardoZuppetti/final_assignment/blob/main/src/random_number.cpp)
* [robot_interface.cpp](https://github.com/RiccardoZuppetti/final_assignment/blob/main/src/robot_interface.cpp)
* [wall_follower.cpp](https://github.com/RiccardoZuppetti/final_assignment/blob/main/src/wall_follower.cpp)

Instead the servers are:
* [random.srv](https://github.com/RiccardoZuppetti/final_assignment/blob/main/srv/random.srv)
* [wall_follow.srv](https://github.com/RiccardoZuppetti/final_assignment/blob/main/srv/wall_follow.srv)

The node [random_number.cpp](https://github.com/RiccardoZuppetti/final_assignment/blob/main/src/random_number.cpp) implements the service for the `random` server, while the node [wall_follower.cpp](https://github.com/RiccardoZuppetti/final_assignment/blob/main/src/wall_follower.cpp) implements the service for the `wall_follow` server.
The node [robot_interface.cpp](https://github.com/RiccardoZuppetti/final_assignment/blob/main/src/robot_interface.cpp) act, instead, as a client for both servers.
The node [commands.cpp](https://github.com/RiccardoZuppetti/final_assignment/blob/main/src/commands.cpp) implements a command line interface, useful for making the user give an action to the robot.
For this reason, this node sets the parameter `/state`.
The node [robot_interface.cpp](https://github.com/RiccardoZuppetti/final_assignment/blob/main/src/robot_interface.cpp) reads the current position of the robot and depending on the states implements the different behaviors of the robot.
Each state corresponds to an action, in particular the above implemented actions.
Every state has the following structure: it checks if it should do some initialization operations and in that case completes them, or it completes other operations that must be done every iteration.

The first state corresponds to reach a random target, chosen between a set of valid ones.
Here the initialization consists in recalling the `random` server and publishing the goal (i.e. the random position that the robot have to reach) on the topic `/move_base/goal`.
On top, it updates the value of the variable 'target_position' of type `nav_msgs/Odometry`, that is used to calculate the distance at each iteration of the robot from the target.
So, this state has to compute, at each iteration, the distance between the robot and the target to reach.
If and only if this distance is less or equal to 1, the goal position is considered to have been reached and the state changes from 'reach a random target' to 'stop in the last position'.
For this reason, reached the goal, the robot stops in that position.

The second state, that corresponds to reach a specific target entered by the user, is very similar to the previous state.
In fact, it slightly differ from the first state for what concerns the initialization, where it asks to the user the position to reach and checks if this one is valid.
The rest is basically the same.

The third state corresponds to the action of following the wall.
Here, the initialization consists of recalling the `wall_follow` server and passing 1 as request, in order to start to follow the wall.
In this state, the user is able to change the robot behaviour at any time.
For this reason, if the user sends a new action (at any time), different from the action to follow the wall, it recalls the `wall_follow` server, passing 0 now as request, in order to stop to follow the wall.
After that, the state changes, i.e. the robot stops in the last position, and so now the state is put equal to 4.

The fourth and last state sets the target_position as the current one and publishes it on the topic `/move_base/goal`.
Here, on top, it's possible to switch to any of the above states, in order to perform the action that the user has given.

The [random_number.cpp](https://github.com/RiccardoZuppetti/final_assignment/blob/main/src/random_number.cpp) service calls the function 'randMToN' that returns an integer that is the index of two given vectors with elements the valid x and y coordinates that are returned as responses of the server.
The server `random` has as request two integer, the minimum and maximum random number requested, and as response the two coordinates.

The [wall_follower.cpp](https://github.com/RiccardoZuppetti/final_assignment/blob/main/src/wall_follower.cpp) service when called sets the 'init' variable equal to the request (1 to follow the wall, 0 to not follow the wall).
The state of the laser is updated each time data are published on the topic `/scan` and a new state is decided.
In the main a while loop is implemented that checks if the 'init' variable is equal to 1.
In that case, depending on the state, it calls a function that sets the velocity on the topic `/cmd_vel`.
If, instead, 'init' is equal to 0, the function stop is called and all velocities are set to zero.
# How to launch
1. Open a shell and install the required packages
```
sudo apt-get install ros-noetic-openslam-gmapping
```
```
sudo apt-get install ros-noetic-navigation
```
```
sudo apt-get install ros-noetic-move-base
```
2. Move to your catkin_ws/src folder and install the 'slam-gmapping' package
```
git clone https://github.com/CarmineD8/slam_gmapping.git
```
3. Move to your catkin_ws/src folder and run
```
git clone https://github.com/RiccardoZuppetti/final_assignment.git
```
4. Go to the root of the workspace, and from there launch `catkin_make`

5. To launch the 3D simulation environment run the command
```
roslaunch final_assignment move_base.launch
```
6. Open a second shell and type
```
rosrun final_assignment commands
```
7. Open a third shell and type
```
rosrun final_assignment robot_interface
```
# Robot behaviour
Initially, at the start of the simulation, the robot is in the fourth state, i.e. it stays stopped in the initial position.
For this reason, it waits for the next action (or command) that the user will send.
Given the first command, the robot will start to serve the requested action.
If the command given is 1 (reach a random target), the robot will start to move in the direction of the first random target.
Reached that position it will ask for a new random target, if and only if the user don't send another request.
Otherwise it will serve the new request.
If the command given is 2 (reach a specific target), the [robot_interface.cpp](https://github.com/RiccardoZuppetti/final_assignment/blob/main/src/robot_interface.cpp) asks to the user to insert a position, and it will wait for a valid one.
Once a valid target has been entered, and once this target has been reached, the robot goes into the fourth state, i.e. it stops in the position reached.
If the user don't send another action (or command), the robot will ask to the user for another position to be reached.
Otherwise, it will serve the new requested action.
If the command given is 3 (start to follow the wall), the robot will immediately start to follow the wall, and it will check at any moment if the user sends a new action (or command), in order to change its behaviour at any time.
Unlike state 3, the states 1 and 2 allow the robot to change its behaviour only when the target has been reached.
In the event that, in states 1 and 2, multiple actions to be performed are sent to the robot, the robot will carry out only the last of these sent actions.
If the robot doesn't receive any request while moving towards a target in state 1 or 2, when the target is reached the robot will immediately start another cycle by asking to the server for a random goal, or asking to the user for a new target to be reached.
# Software architecture and architectural choices
For the development of this project it is used as programming language C++, in order to re-use partially what has been used for the first assignment.
Since the robot interface can be assimilated to an FSM, the codes have many if else statements, in order to verify every possible condition and establish for each of these the right behaviour that the robot must undertake.
Referring to the above package's description, four nodes have been created, in order to correctly diversify the various tasks necessary for the correct functioning of the simulation:
* the [commands.cpp](https://github.com/RiccardoZuppetti/final_assignment/blob/main/src/commands.cpp) node has to ask and retrieve the 'state' information
* the [random_number.cpp](https://github.com/RiccardoZuppetti/final_assignment/blob/main/src/random_number.cpp) node has to provide a random target
* the [robot_interface.cpp](https://github.com/RiccardoZuppetti/final_assignment/blob/main/src/robot_interface.cpp) node has to communicate will all of the other nodes, in order to establish the action to do and retrieve the information to do this action
* the [wall_follower.cpp](https://github.com/RiccardoZuppetti/final_assignment/blob/main/src/wall_follower.cpp) node has to implement the homonym behaviour
# System's limitations and possible improvements
The user send commands to the robot through two different shells, in particular when it asks for a specific target position to reach.
Maybe this aspect could be solved in a further implementation, that allows to the user to send commands through only one shell.
On top, sometimes the robot slows down dramatically, and for sure this aspect has to be solved in a further implementation of the project.