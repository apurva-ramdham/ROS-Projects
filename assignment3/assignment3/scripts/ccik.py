#!/usr/bin/env python

import math
import numpy
import rospy
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Transform
from cartesian_control.msg import CartesianCommand
from urdf_parser_py.urdf import URDF
import random
import tf
from threading import Thread, Lock

'''This is a class which will perform both cartesian control and inverse
   kinematics'''
class CCIK(object):
    def __init__(self): 
	#Load robot from parameter server
        self.robot = URDF.from_parameter_server()

	#Subscribe to current joint state of the robot
        rospy.Subscriber('/joint_states', JointState, self.get_joint_state)

	#This will load information about the joints of the robot
        self.num_joints = 0
        self.joint_names = []
        self.q_current = []
        self.joint_axes = []
        self.get_joint_info()

	#This is a mutex
        self.mutex = Lock()

	#Subscribers and publishers for for cartesian control
        rospy.Subscriber('/cartesian_command', CartesianCommand, self.get_cartesian_command)
        self.velocity_pub = rospy.Publisher('/joint_velocities', JointState, queue_size=10)
        self.joint_velocity_msg = JointState()

        #Subscribers and publishers for numerical IK
        rospy.Subscriber('/ik_command', Transform, self.get_ik_command)
        self.joint_command_pub = rospy.Publisher('/joint_command', JointState, queue_size=10)
        self.joint_command_msg = JointState()

    '''This is a function which will collect information about the robot which
       has been loaded from the parameter server. It will populate the variables
       self.num_joints (the number of joints), self.joint_names and
       self.joint_axes (the axes around which the joints rotate)'''
    def get_joint_info(self):
        link = self.robot.get_root()
        while True: 
            if link not in self.robot.child_map: break
            (joint_name, next_link) = self.robot.child_map[link][0]
            current_joint = self.robot.joint_map[joint_name]
            if current_joint.type != 'fixed':
                self.num_joints = self.num_joints + 1
                self.joint_names.append(current_joint.name)
                self.joint_axes.append(current_joint.axis)
            link = next_link

    '''This is the callback which will be executed when the cartesian control
       recieves a new command. The command will contain information about the
       secondary objective and the target q0. At the end of this callback, 
       you should publish to the /joint_velocities topic.'''
    def get_cartesian_command(self, command):
        self.mutex.acquire()
        #--------------------------------------------------------------------------
        #FILL IN YOUR PART OF THE CODE FOR CARTESIAN CONTROL HERE
        a=command.x_target
        
        [joint_transforms,b_T_ee_current] = self.forward_kinematics(self.q_current)
        
        b_tr_ee_desired=tf.transformations.translation_matrix((a.translation.x,a.translation.y,a.translation.z))
        
        b_R_ee_desired=tf.transformations.quaternion_matrix((a.rotation.x,a.rotation.y,a.rotation.z,a.rotation.w))
      
        b_T_ee_desired=numpy.dot(b_tr_ee_desired,b_R_ee_desired)
        
        ee_current_T_b=tf.transformations.inverse_matrix(b_T_ee_current)
        
        ee_current_T_ee_desired=numpy.dot(ee_current_T_b,b_T_ee_desired)
        
        ee_tr_ee=tf.transformations.translation_from_matrix(ee_current_T_ee_desired)
        
        [angle,axis]=self.rotation_from_matrix(ee_current_T_ee_desired)
        
        rot=numpy.dot(axis,angle)
           
        dx=numpy.concatenate((ee_tr_ee,rot),axis=0)
        
        prop_gain=1.2
        vee=dx*prop_gain
        
        if numpy.linalg.norm(vee[:3])>0.1:
         for i in range (0,3):
             vee[i]=0.1*vee[i]/numpy.linalg.norm(vee[:3]) #divide by norm to get the condition
        if numpy.linalg.norm(vee[3:6])>1:
         for i in range (3,6) :
             vee[i]=vee[i]/numpy.linalg.norm(vee[3:6])
        J=self.get_jacobian(b_T_ee_current, joint_transforms)
        
        Jp=numpy.linalg.pinv(J,1.0e-2) #pseudo
        
        q_desired=numpy.dot(Jp,vee)
        if numpy.linalg.norm(vee[3:6])>1:
         for i in range (3,6) :
             q_desired[i]=q_desired[i]/numpy.linalg.norm(q_desired[3:6]) 
        
        #SECONDARY OBJECTIVE 
        if command.secondary_objective== True:
          r0=command.q0_target
          q0=self.q_current[0]
          p=3
          q_sec=numpy.zeros(self.num_joints)
          q_sec[0]=p*(r0-q0)
          I=numpy.identity(self.num_joints)
          Jp=numpy.linalg.pinv(J,0)
          
          q_null=numpy.dot(I-numpy.dot(Jp,J),q_sec)
          q_desired=q_desired+q_null
          
        self.joint_velocity_msg.name=self.joint_names
        self.joint_velocity_msg.velocity=q_desired
        self.velocity_pub.publish(self.joint_velocity_msg)
        #--------------------------------------------------------------------------
     
        
        self.mutex.release()

    '''This is a function which will assemble the jacobian of the robot using the
       current joint transforms and the transform from the base to the end
       effector (b_T_ee). Both the cartesian control callback and the
       inverse kinematics callback will make use of this function.
       Usage: J = self.get_jacobian(b_T_ee, joint_transforms)'''
    def get_jacobian(self, b_T_ee, joint_transforms):
        J = numpy.zeros((6,self.num_joints))
        #--------------------------------------------------------------------------
        #FILL IN YOUR PART OF THE CODE FOR ASSEMBLING THE CURRENT JACOBIAN HERE
        
        for j in range(self.num_joints):
            b_T_j= joint_transforms[j]
            ee_T_b=tf.transformations.inverse_matrix(b_T_ee)
            ee_T_j= numpy.dot(ee_T_b,b_T_j)
            ee_R_j=ee_T_j[:3,:3]
            j_T_ee=tf.transformations.inverse_matrix(ee_T_j)
            j_tr_ee=j_T_ee[:3,3]
            S=numpy.identity(3) #skew symmetric matrix
            S[0][0]=0
            S[0][1]=-j_tr_ee[2]
            S[0][2]=j_tr_ee[1]
            S[1][0]=j_tr_ee[2]
            S[1][1]=0
            S[1][2]=-j_tr_ee[0]
            S[2][0]=-j_tr_ee[1]
            S[2][1]=j_tr_ee[0]
            S[2][2]=0
            productRS=-numpy.dot(ee_R_j,S)
            Zero=numpy.zeros((3,3))
            AB=numpy.concatenate((ee_R_j,Zero),axis=0) #along vertical
            CD=numpy.concatenate((productRS,ee_R_j),axis=0)
            Vj=numpy.concatenate((AB,CD),axis=1) #along horizontal
            axis=self.joint_axes[j]
            axist=numpy.transpose(axis) #to get column matrix
            #print axist
            V=numpy.dot(Vj[:,3:6],axist) 
            J[:,j]=V
          
        #--------------------------------------------------------------------------
        return J

    '''This is the callback which will be executed when the inverse kinematics
       recieve a new command. The command will contain information about desired
       end effector pose relative to the root of your robot. At the end of this
       callback, you should publish to the /joint_command topic. This should not
       search for a solution indefinitely - there should be a time limit. When
       searching for two matrices which are the same, we expect numerical
       precision of 10e-3.'''
    def get_ik_command(self, command):
        self.mutex.acquire()
	num=0
        #--------------------------------------------------------------------------
        #FILL IN YOUR PART OF THE CODE FOR INVERSE KINEMATICS HERE 
        
        a=command
        b_tr_ee_desired=tf.transformations.translation_matrix((a.translation.x,a.translation.y,a.translation.z))
        b_R_ee_desired=tf.transformations.quaternion_matrix((a.rotation.x,a.rotation.y,a.rotation.z,a.rotation.w))
        b_T_ee_desired=numpy.dot(b_tr_ee_desired,b_R_ee_desired)

        
        
	qc=numpy.random.uniform(low=0.0,high=2*math.pi,size=self.num_joints)
	
        while True:
            num=0
            [joint_transforms,b_T_ee]=self.forward_kinematics(qc)
            b_T_ee_current_inv=tf.transformations.inverse_matrix(b_T_ee)	
	    current_T_desired=numpy.dot(b_T_ee_current_inv,b_T_ee_desired)
	    
            ee_tr_ee=tf.transformations.translation_from_matrix(current_T_desired)
            
            [angle,axis]=self.rotation_from_matrix(current_T_desired)
            
            rot=numpy.dot(axis,angle)
	       
            dx=numpy.concatenate((ee_tr_ee,rot),axis=0) #6*1 matrix
	    
            J=self.get_jacobian(b_T_ee, joint_transforms)
            Jp=numpy.linalg.pinv(J)
            
	    dq=numpy.dot(Jp,dx)
	    
	    qc = qc+dq
            
	    for i in range(0,self.num_joints):            
		if abs(dq[i])<0.001:
                    num=num+1
		else:
		    break
	    print num
            if num==self.num_joints:
		break
	    else:
		continue
		
        self.joint_command_msg.name=self.joint_names
        self.joint_command_msg.position=qc
        self.joint_command_pub.publish(self.joint_command_msg)
        
         


        #--------------------------------------------------------------------------
        self.mutex.release()

    '''This function will return the angle-axis representation of the rotation
       contained in the input matrix. Use like this: 
       angle, axis = rotation_from_matrix(R)'''
    def rotation_from_matrix(self, matrix):
        R = numpy.array(matrix, dtype=numpy.float64, copy=False)
        R33 = R[:3, :3]
        # axis: unit eigenvector of R33 corresponding to eigenvalue of 1
        l, W = numpy.linalg.eig(R33.T)
        i = numpy.where(abs(numpy.real(l) - 1.0) < 1e-8)[0]
        if not len(i):
            raise ValueError("no unit eigenvector corresponding to eigenvalue 1")
        axis = numpy.real(W[:, i[-1]]).squeeze()
        # point: unit eigenvector of R33 corresponding to eigenvalue of 1
        l, Q = numpy.linalg.eig(R)
        i = numpy.where(abs(numpy.real(l) - 1.0) < 1e-8)[0]
        if not len(i):
            raise ValueError("no unit eigenvector corresponding to eigenvalue 1")
        # rotation angle depending on axis
        cosa = (numpy.trace(R33) - 1.0) / 2.0
        if abs(axis[2]) > 1e-8:
            sina = (R[1, 0] + (cosa-1.0)*axis[0]*axis[1]) / axis[2]
        elif abs(axis[1]) > 1e-8:
            sina = (R[0, 2] + (cosa-1.0)*axis[0]*axis[2]) / axis[1]
        else:
            sina = (R[2, 1] + (cosa-1.0)*axis[1]*axis[2]) / axis[0]
        angle = math.atan2(sina, cosa)
        return angle, axis

    '''This is the function which will perform forward kinematics for your 
       cartesian control and inverse kinematics functions. It takes as input
       joint values for the robot and will return an array of 4x4 transforms
       from the base to each joint of the robot, as well as the transform from
       the base to the end effector.
       Usage: joint_transforms, b_T_ee = self.forward_kinematics(joint_values)'''
    def forward_kinematics(self, joint_values):
        joint_transforms = []

        link = self.robot.get_root()
        T = tf.transformations.identity_matrix()

        while True:
            if link not in self.robot.child_map:
                break

            (joint_name, next_link) = self.robot.child_map[link][0]
            joint = self.robot.joint_map[joint_name]

            T_l = numpy.dot(tf.transformations.translation_matrix(joint.origin.xyz), tf.transformations.euler_matrix(joint.origin.rpy[0], joint.origin.rpy[1], joint.origin.rpy[2]))
            T = numpy.dot(T, T_l)

            if joint.type != "fixed":
                joint_transforms.append(T)
                q_index = self.joint_names.index(joint_name)
                T_j = tf.transformations.rotation_matrix(joint_values[q_index], numpy.asarray(joint.axis))
                T = numpy.dot(T, T_j)

            link = next_link
        return joint_transforms, T #where T = b_T_ee

    '''This is the callback which will recieve and store the current robot
       joint states.'''
    def get_joint_state(self, msg):
        self.mutex.acquire()
        self.q_current = []
        for name in self.joint_names:
            self.q_current.append(msg.position[msg.name.index(name)])
        self.mutex.release()


if __name__ == '__main__':
    rospy.init_node('cartesian_control_and_IK', anonymous=True)
    CCIK()
    rospy.spin()
