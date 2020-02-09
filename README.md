# ROS-Projects
# Assignment 1

In this project we consider a ROS ecosystem, which consists of a robot with a camera mounted on it as well as an object. To describe the poses of all these items, we define the following coordinate frames:

A base coordinate frame called 'base_frame'
A robot coordinate frame  called 'robot_frame'
A camera coordinate frame called 'camera_frame'
An object coordinate frame 'object_frame'

The following relationships are true:

The transform from the 'base_frame' coordinate frame to the 'object_frame' coordinate frame consists of a rotation expressed as (roll, pitch, yaw) of (0.64, 0.64, 0.0) followed by a translation of 1.5m along the resulting x-axis and 0.8m along the resulting y-axis. 
The transform from the 'base_frame' coordinate frame to the 'robot_frame' coordinate frame consists of a rotation around the y-axis by 1.5 radians followed by a translation along the resulting z-axis of -2.0m. 
The transform from the 'robot_frame' coordinate frame to the 'camera_frame' coordinate frame must be defined as follows:
The translation component of this transform is (0.3, 0.0, 0.3)
The rotation component of this transform must be set such that the camera is pointing directly at the object. In other words, the x-axis of the 'camera_frame' coordinate frame must be pointing directly at the origin of the 'object_frame' coordinate frame.
