# ROS-Projects
# Assignment 1

In this project we consider a ROS ecosystem, which consists of a robot with a camera mounted on it as well as an object. We defined 4 coordinate frames: 
1. A base coordinate frame called 'base_frame'
2. A robot coordinate frame  called 'robot_frame'
3. A camera coordinate frame called 'camera_frame'
4. An object coordinate frame 'object_frame'

The task was to fill in the code for a node called solution.py that published following transforms:
1. The transform from the 'base_frame' coordinate frame to the 'object_frame' coordinate frame 
2. The transform from the 'base_frame' coordinate frame to the 'robot_frame' coordinate frame 
3. The transform from the 'robot_frame' coordinate frame to the 'camera_frame' coordinate frame

The objective of the project was to publish the correct transform, which would make the arrow point out of the cube directly at the cylinder and the tip of the arrow should be just barely touching the cylinder. 

# Assignment 2
The goal of this assignment is to write a Kinematics script in a file called forward_kinematics.py. The script must work for both kuka and ur5 robots.

Every time a new set of joint values are received, the node uses this information, along with the URDF, and computes the transform from the robot "root" (first link) to each link in the chain. All of these transforms are then published to TF.

# Assignment 3
In this project demonstartes a node capable of performing Cartesian Control and Numerical IK of kuka and ur5 robots.
This project is an implementation of a complete algorithm for Cartesian end-effector translation control with a secondary objective.
The file ccik.py contains primarily three functions get_cartesian_command, get_ik_command, get_jacobian.


