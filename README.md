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


