#!/usr/bin/env python
import rospy

import numpy

import tf
import tf2_ros
import geometry_msgs.msg



def publish_transforms():
   
    Rt1=tf.transformations.quaternion_matrix(tf.transformations.quaternion_from_euler(0.64, 0.64, 0.00)) 
    Tr1=tf.transformations.translation_matrix((1.5, 0.8, 0.0))  
    T1 = tf.transformations.concatenate_matrices(Rt1, Tr1)

    tr1 = geometry_msgs.msg.TransformStamped()
    tr1.header.stamp = rospy.Time.now()
    tr1.header.frame_id = "base_frame"
    tr1.child_frame_id = "object_frame"
    trfm1 = geometry_msgs.msg.Transform()
    r1 = tf.transformations.quaternion_from_matrix(T1)
    tr1.transform.rotation.x = r1[0]
    tr1.transform.rotation.y = r1[1]
    tr1.transform.rotation.z = r1[2]
    tr1.transform.rotation.w = r1[3]

    t1 = tf.transformations.translation_from_matrix(T1)
    tr1.transform.translation.x = t1[0]
    tr1.transform.translation.y = t1[1]
    tr1.transform.translation.z = t1[2] 
   
   
    br.sendTransform(tr1)




    Rt2=tf.transformations.quaternion_matrix(tf.transformations.quaternion_about_axis(1.5,(0.00, 1.00, 0.00))) 
    Tr2=tf.transformations.translation_matrix((0.0, 0.0, -2.0))  
    T2 = tf.transformations.concatenate_matrices(Rt2, Tr2)
    print Rt2,Tr2
    tr2 = geometry_msgs.msg.TransformStamped()
    tr2.header.stamp = rospy.Time.now()
    tr2.header.frame_id = "base_frame"
    tr2.child_frame_id = "robot_frame"
    
    r2 = tf.transformations.quaternion_from_matrix(T2)
    tr2.transform.rotation.x = r2[0]
    tr2.transform.rotation.y = r2[1]
    tr2.transform.rotation.z = r2[2]
    tr2.transform.rotation.w = r2[3]

    t2 = tf.transformations.translation_from_matrix(T2)
    tr2.transform.translation.x = t2[0]
    tr2.transform.translation.y = t2[1]
    tr2.transform.translation.z = t2[2] 
   
    br.sendTransform(tr2)

    
    



    Tr3 = tf.transformations.translation_matrix([0.3, 0.0, 0.3])
    Rt3 = tf.transformations.quaternion_matrix(tf.transformations.quaternion_from_euler(0,0,0))
    
    T3 = tf.transformations.concatenate_matrices(Tr3, Rt3)
    
    T=[0,0,0,1]
    
    
   
    base2camera_transform = numpy.dot(T2,T3)
   
    camera2base_transform = tf.transformations.inverse_matrix(base2camera_transform)
   
    camera2object_transform= numpy.dot(camera2base_transform,T1)
   
    objectposition=numpy.dot(T1,T)
    cameraposition=numpy.dot(camera2object_transform,T)
     
   
    camera2object_translation=numpy.dot(camera2object_transform,T)
    cameraposition=cameraposition[:(len(cameraposition)-1)]
    xaxis=[1,0,0]
    xaxis= xaxis / numpy.linalg.norm(xaxis)
    cameraposition= cameraposition / numpy.linalg.norm(cameraposition)
   
    angle=numpy.arccos(numpy.clip(numpy.dot(xaxis, cameraposition),-1.0,1.0))
    
    normal=numpy.cross(xaxis,cameraposition)
    R3= tf.transformations.quaternion_matrix(tf.transformations.quaternion_about_axis(angle,normal))
    T4 = tf.transformations.concatenate_matrices(Tr3, R3)

    tr4 = geometry_msgs.msg.TransformStamped()
    tr4.header.stamp = rospy.Time.now()
    tr4.header.frame_id = "robot_frame"
    tr4.child_frame_id = "camera_frame"
    
    r4 = tf.transformations.quaternion_from_matrix(T4)
    tr4.transform.rotation.x = r4[0]
    tr4.transform.rotation.y = r4[1]
    tr4.transform.rotation.z = r4[2]
    tr4.transform.rotation.w = r4[3]
 
    t4 = tf.transformations.translation_from_matrix(T4)
    tr4.transform.translation.x = t4[0]
    tr4.transform.translation.y = t4[1]
    tr4.transform.translation.z = t4[2] 

 
    br.sendTransform(tr4)
   


    
if __name__ == '__main__':

    rospy.init_node('solution')

    br = tf2_ros.TransformBroadcaster()
    rospy.sleep(0.1)

    while not rospy.is_shutdown():
        publish_transforms()
rospy.sleep(0.1)
