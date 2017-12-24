#!/usr/bin/env python

# Copyright (C) 2017 Electric Movement Inc.
#
# This file is part of Robotic Arm: Pick and Place project for Udacity
# Robotics nano-degree program
#
# All Rights Reserved.

# Author: Harsh Pandya, Jean Gonzalez

# import modules
import rospy
import tf
import numpy as np
from kuka_arm.srv import *
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from geometry_msgs.msg import Pose
from mpmath import *
from sympy import *
from sympy.matrices import Matrix


'''
Modified DH Transform Matrix
alpha: Twist angle
a: Link length
d: Link offset
q: Joint variable; Theta
'''
def dh_transform(alpha, a, d, q):
    t = Matrix([[cos(q), -sin(q), 0, a],
		[sin(q) * cos(alpha), cos(q) * cos(alpha), -sin(alpha), -sin(alpha) * d],
		[sin(q) * sin(alpha), cos(q) * sin(alpha), cos(alpha), cos(alpha) * d],
		[0, 0, 0, 1]])

    return t


# Rotation matrix along the Y-axis.
def R_y(q):
    rot_y = Matrix([[cos(q), 	0, 	sin(q), 	0],
		    [0,		1,	     0, 	0],
		    [-sin(q),	0,	cos(q), 	0],
		    [0,		0,	     0,		1]])

    return rot_y


# Rotation matrix along the Z-axis.
def R_z(q):
    rot_z = Matrix([[cos(q), 	  -sin(q), 		0, 	0],
		    [sin(q),	   cos(q),		0, 	0],
		    [0,			0,		1, 	0],
		    [0,			0,		0,	1]])

    return rot_z


# Roll transform
def T_roll(): 
    roll = Matrix([[1, 	     	    0, 		       0],
		   [0,	cos(roll_sym),	  -sin(roll_sym)],
		   [0,	sin(roll_sym),    cos(roll_sym)]])

    return roll


# Pitch transform
def T_pitch():
    pitch = Matrix([[cos(pitch_sym), 	0, sin(pitch_sym)],
		    [0,			1,		0],
		    [-sin(pitch_sym),	0, cos(pitch_sym)]])

    return pitch


# Yaw transform
def T_yaw():
    yaw = Matrix([[cos(yaw_sym), 	-sin(yaw_sym), 		0],
		  [sin(yaw_sym),	 cos(yaw_sym),		0],
		  [0,				    0,		1]])

    return yaw


def handle_calculate_IK(req):
    global yaw_sym, pitch_sym, roll_sym

    rospy.loginfo("Received %s eef-poses from the plan" % len(req.poses))
    if len(req.poses) < 1:
        print "No valid poses received"
        return -1

    else:
        # Initialize service response
        joint_trajectory_list = []


        # Define DH param symbols
        q1, q2, q3, q4, q5, q6, q7 = symbols('q1:8')
        d1, d2, d3, d4, d5, d6, d7 = symbols('d1:8')
        a0, a1, a2, a3, a4, a5, a6 = symbols('a0:7')
        alpha0, alpha1, alpha2, alpha3, alpha4, alpha5, alpha6 = symbols('alpha0:7')
 
        # End-effector orientation symbols
        roll_sym, pitch_sym, yaw_sym = symbols("roll pitch yaw")

        # Variables
        theta1, theta2, theta3, theta4, theta5 = 0, 0, 0, 0, 0

        # Modified DH params
        s = {alpha0:       0, a0:      0, d1:  0.75,
             alpha1: -pi / 2, a1:   0.35, d2:     0, q2: q2 - pi / 2,
             alpha2:       0, a2:   1.25, d3:     0,
             alpha3: -pi / 2, a3: -0.054, d4:   1.5,
             alpha4:  pi / 2, a4:      0, d5:     0,
             alpha5: -pi / 2, a5:      0, d6:     0,
             alpha6: 0, a6: 0, d7: 0.303, q7: 0}

            
        '''
        Define Modified DH Transformation matrix
        '''
            
        # base_link to link1
        T0_1 = dh_transform(alpha0, a0, d1, q1)
        T0_1 = T0_1.subs(s)

        # link1 to link2
        T1_2 = dh_transform(alpha1, a1, d2, q2)
        T1_2 = T1_2.subs(s)

        # link2 to link3
        T2_3 = dh_transform(alpha2, a2, d3, q3)
        T2_3 = T2_3.subs(s)

        # link3 to link4
        T3_4 = dh_transform(alpha3, a3, d4, q4)
        T3_4 = T3_4.subs(s)

        # link4 to link5
        T4_5 = dh_transform(alpha4, a4, d5, q5)
        T4_5 = T4_5.subs(s)

        # link5 to link6
        T5_6 = dh_transform(alpha5, a5, d6, q6)
        T5_6 = T5_6.subs(s)

        # link6 to end effector
        T6_7 = dh_transform(alpha6, a6, d7, q7)
        T6_7 = T6_7.subs(s)


        # Create individual transformation matrices
        T0_2 = simplify(T0_1 * T1_2) # base_link to link2
        T0_3 = simplify(T0_2 * T2_3) # base_link to link3
        T0_4 = simplify(T0_3 * T3_4) # base_link to link4
        T0_5 = simplify(T0_4 * T4_5) # base link to link5
        T0_6 = simplify(T0_5 * T5_6) # base link to link6
        T0_G = simplify(T0_6 * T6_7) # base link to gripper
            
        '''
        Correction to account for the orientation difference between the 
        defition of gripper link in URDF versus DH convention.
        '''
        R_corr = R_z(pi) * R_y(-pi/2)

        '''
        Total homogeneous transform between base link and gripper link
        with orientation correction applied
        '''
        T_total = simplify(T0_G * R_corr)


        for x in xrange(0, len(req.poses)):
            # IK code starts here

            joint_trajectory_point = JointTrajectoryPoint()

      
            '''      
            Extract end-effector position and orientation from request
	    px,py,pz = end-effector position
	    roll, pitch, yaw = end-effector orientation
            '''
            px = req.poses[x].position.x
            py = req.poses[x].position.y
            pz = req.poses[x].position.z
            

            (roll, pitch, yaw) = tf.transformations.euler_from_quaternion(
                [req.poses[x].orientation.x, req.poses[x].orientation.y,
                    req.poses[x].orientation.z, req.poses[x].orientation.w])
     
            # Calculate joint angles using Geometric IK method
            R_ee = (T_yaw() * T_pitch() * T_roll()).evalf(subs={roll_sym:roll, pitch_sym:pitch, yaw_sym:yaw})
           
            '''
            Wrist Center (WC) position
            px, py, pz = end-effector positions
            wx, wy, wz = wrist positions
            '''  
            wx = (px - (d6 + d7) * R_ee[0,0]).subs(s)
            wy = (py - (d6 + d7) * R_ee[1,0]).subs(s)
            wz = (pz - (d6 + d7) * R_ee[2,0]).subs(s)		

            # Theta1
            theta1 = atan2(wy, wx)

            '''
            Fix wrist positions to account for errors in link length of joint 4.
            Kudos to Richie Muniak for finding this. 
            '''
            j4_fix = atan2(wz - 1.94645, wx)
            wx = wx - 0.054 * sin(j4_fix)
            wz = wz + 0.054 * cos(j4_fix)

            # Distance from origin to fixed wrist center
            wx_dist = sqrt(wy * wy + wx * wx)

            # Link length for theta 2 and theta 3 as per the modified DH table
            l1 = s[a2]
            l2 = s[d4]

            # Return joint to origin
            wx_dist = wx_dist - s[a1]
            wz_dist = wz - s[d1]


            '''
            Formula to derive theta 3 as provided by Mark W. Spong in 
            Robot Dynamics and Control.
            '''
            D = (wx_dist * wx_dist + wz_dist * wz_dist - l1 * l1 - l2 * l2) / (2 * l1 * l2)

            # Correct D to not go above 1
            if (D > 1):
                D = 1


            # Theta3
            theta3 = atan2(-sqrt(1 - D * D), D)


            # Mark Spong formula to derive theta2 and theta3
            S1 = ((l1 + l2 * cos(theta3)) * wz_dist - l2 * sin(theta3) * wx_dist) / (wx_dist * wx_dist + wz_dist * wz_dist)

            C1 = ((l1 + l2 * cos(theta3)) * wx_dist + l2 * sin(theta3) * wz_dist) / (wx_dist * wx_dist + wz_dist * wz_dist)

            theta2 = atan2(S1, C1)


            # Theta3 translated by 90 degrees
            theta3 = -1 * (theta3 + pi/2)

            # Theta2 translated by 90 degrees
            theta2 = pi/2 - theta2


            # Rotation matrix up to WC
            R0_3 = (T0_1 * T1_2 * T2_3).evalf(subs={q1:theta1, q2:theta2, q3:theta3})[0:3, 0:3]

            # Rotation matrix from WC to EE
            R3_6 = R0_3.transpose()[0:3, 0:3] * R_ee[0:3, 0:3]


            # Rotation adjustments to determine remaining thetas
            R3_6 = (R3_6 * T_yaw()).evalf(subs={yaw_sym: -pi/2})[0:3, 0:3] * T_pitch().evalf(subs={pitch_sym: -pi/2})[0:3, 0:3]

            theta4, theta5, theta6 = tf.transformations.euler_from_matrix(np.array(R3_6).astype(np.float64), "ryzy")

            # Clip theta5 values to avoid collisions
            theta5 = np.clip(theta5, -2,2)         


            # Response for the IK request

	    joint_trajectory_point.positions = [theta1, theta2, theta3, theta4, theta5, theta6]
	    joint_trajectory_list.append(joint_trajectory_point)


        rospy.loginfo("length of Joint Trajectory List: %s" % len(joint_trajectory_list))
        return CalculateIKResponse(joint_trajectory_list)


def IK_server():
    # Initialize node and declare calculate_ik service
    rospy.init_node('IK_server')
    s = rospy.Service('calculate_ik', CalculateIK, handle_calculate_IK)
    print "Ready to receive an IK request"
    rospy.spin()

if __name__ == "__main__":
    IK_server()
