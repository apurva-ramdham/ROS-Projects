#!/usr/bin/env python

import numpy
import geometry_msgs.msg
import rospy
from sensor_msgs.msg import JointState
import tf
import tf.msg
from urdf_parser_py.urdf import URDF

"""This function will transform a 4x4 transformation matrix T into a ros message 
which can be published. In addition to the transform itself, the message
also specifies who is related by this transform, the parent and the child.
It is an optional function which you may use as you see fit."""
def convert_to_message(T, child, parent):
    t = geometry_msgs.msg.TransformStamped()
    t.header.frame_id = parent
    t.header.stamp = rospy.Time.now()
    t.child_frame_id = child
    translation = tf.transformations.translation_from_matrix(T)
    rotation = tf.transformations.quaternion_from_matrix(T)
    t.transform.translation.x = translation[0]
    t.transform.translation.y = translation[1]
    t.transform.translation.z = translation[2]
    t.transform.rotation.x = rotation[0]
    t.transform.rotation.y = rotation[1]
    t.transform.rotation.z = rotation[2]
    t.transform.rotation.w = rotation[3]        
    return t
    
#Our main class for computing Forward Kinematics
class ForwardKinematics(object):

    #Initialization
    def __init__(self):
        """Announces that it will publish forward kinematics results, in the form of tfMessages.
        "tf" stands for "transform library", it's ROS's way of communicating around information
        about where things are in the world"""
        self.pub_tf = rospy.Publisher("/tf", tf.msg.tfMessage, queue_size=1)

        #Loads the robot model, which contains the robot's kinematics information
        self.robot = URDF.from_parameter_server()

        #Subscribes to information about what the current joint values are.
        rospy.Subscriber("joint_states", JointState, self.callback)


    """This function is called every time the robot publishes its joint values. You must use
    the information you get to compute forward kinematics.

    The callback you write should iterate through the entire robot chain, and publish 
    the transform for each link you find.
    """
    def callback(self, joint_values):
	
	I=numpy.eye(4)
        link = self.robot.get_root()
        t1=[]
        while True:
	    if link not in self.robot.child_map:
		break
	    else:
	    	(next_joint_name, next_link) = self.robot.child_map[link][0] 
          	next_joint=self.robot.joint_map[next_joint_name]
                joint_values_position=joint_values.position
          	
	  	if next_joint.axis == None:
		   
		    R1=numpy.eye(4)
	  	else:
		    i=joint_values.name.index(next_joint_name)
		    position=joint_values.position[i]
		    R1=tf.transformations.quaternion_matrix(tf.transformations.quaternion_about_axis(joint_values.position[i], next_joint.axis))
	  
		xyz=next_joint.origin.xyz
		t=tf.transformations.translation_matrix(xyz)
		rpy=next_joint.origin.rpy
		R2=tf.transformations.quaternion_matrix(tf.transformations.quaternion_from_euler(rpy[0], rpy[1], rpy[2]))
		R3=numpy.dot(R1,R2)
		
		T=numpy.dot(t,R3)
		
		I=numpy.dot(I,T)
		t1.append(convert_to_message(I, next_link, self.robot.get_root()))
		link= next_link
          
	self.pub_tf.publish(t1)
       
if __name__ == '__main__':
    rospy.init_node('fwk', anonymous=True)
    fwk = ForwardKinematics()
    rospy.spin()

