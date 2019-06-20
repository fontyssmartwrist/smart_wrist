# smart_wrist
Fontys Delta Smart Wrist Project

# Fontys Smart Wrist

Is a project at fontys university funded by Delta electronics, special thanks to all our teahcers at Fontys for their help and support, as well as Delta company for making all of this possible.

# Smart Wrist Packages

The sw_v3 folder contains several packages that simulate the first concept of the smart wrist implemtned in Fontys 2019.

# Installation

1. Download and extract the complete file into your catkin workspace
2. in your terminal: cd catkin/ && catkin_make
3. in your terminal: source /opt/ros/kinetic/setup.bash
4. in your terminal: source ~/catkin_ws/devel/setup.bash
5. Now you should be able to use all of the packages

#sw_v3_description

This package contains the urdf/xacro model of the SW(smart wrist).

It has a launch a file to visualize the module in RVIZ, can be run with the following command:
    roslaunch sw_v3_description sw_v3_rviz.launch

#sw_v3_control

This package is the controller for the smart wrist joints, the parameters can be tuned in the cinfiguration file.

It can be launched with: roslaunch sw_v3_control sw_v3_control.launch 

#kinematic_model

This packages runs a node that listens to two topics:
1. /joy : from a joystick package
2. /arm_point : a topic where you can publish an x and y position in the SW workspace

When something is available on any of those topics this node translates x and y positions of the SW workspace into rotaion values of every joint.

Then it publishes them to the controller for the simulation to move.

It can be run with: rosrun kinematic_model kinematic_model_node

#sw_v3_gazebo

This package is for the simulation of the model in gazebo, it has two launch files:

1. first only spawns the module in gazebo: roslaunch sw_v3_gazebo sw_v3_world.launch
2. second one spawns the module, starts the controller and starts the "joy" and "kinematic module nodes":
    roslaunch sw_v3_gazebo arm_simulation.launch


**NOTE** 
1. The imported CAD models are pretty heavy inertially, thus the controller will glitch at some points due to limited effort of the motors
2. If the gazebo or RVIZ model crashes just close the terminal and start again, I never really figured out why that happens.
