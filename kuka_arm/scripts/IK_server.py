#!/usr/bin/env python

# Copyright (C) 2017 Electric Movement Inc.
#
# This file is part of Robotic Arm: Pick and Place project for Udacity
# Robotics nano-degree program
#
# All Rights Reserved.

# Author: Harsh Pandya

# import modules
import numpy as np
import rospy
import tf
from kuka_arm.srv import *
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from geometry_msgs.msg import Pose
from mpmath import *
from sympy import *

class IK():
    # Define Modified DH Transformation matrix
    def TF_Matrix(self, alpha, a, d, q):
        TF = Matrix([[           cos(q),           -sin(q),           0,             a],
                     [sin(q)*cos(alpha), cos(q)*cos(alpha), -sin(alpha), -sin(alpha)*d],
                     [sin(q)*sin(alpha), cos(q)*sin(alpha),  cos(alpha),  cos(alpha)*d],
                     [                0,                 0,           0,            1]])
        return TF

    def __init__(self):
        ### Your FK code here
        # Create symbols
        self.q1, self.q2, self.q3, self.q4, self.q5, self.q6, q7 = symbols('q1:8') # joint angle
        d1, d2, d3, d4, d5, d6, d7 = symbols('d1:8') # link offset
        a0, a1, a2, a3, a4, a5, a6 = symbols('a0:7') # link length
        alpha0,alpha1,alpha2,alpha3,alpha4,alpha5,alpha6 = symbols('alpha0:7')# twist angle

        # Create Modified DH parameters
        dh = {alpha0:     0, a0:      0, d1:     0.75,   self.q1:         self.q1,
              alpha1: -pi/2, a1:   0.35, d2:        0,   self.q2:    self.q2-pi/2,
              alpha2:     0, a2:   1.25, d3:        0,   self.q3:         self.q3,
              alpha3: -pi/2, a3: -0.054, d4:      1.5,   self.q4:         self.q4,
              alpha4:  pi/2, a4:      0, d5:        0,   self.q5:         self.q5,
              alpha5: -pi/2, a5:      0, d6:        0,   self.q6:         self.q6,
              alpha6:     0, a6:      0, d7:    0.303,   q7:                    0}


	# Create individual transformation matrices
        T0_1 = self.TF_Matrix(alpha0, a0, d1, self.q1).subs(dh)
        T1_2 = self.TF_Matrix(alpha1, a1, d2, self.q2).subs(dh)
        T2_3 = self.TF_Matrix(alpha2, a2, d3, self.q3).subs(dh)
        T3_4 = self.TF_Matrix(alpha3, a3, d4, self.q4).subs(dh)
        T4_5 = self.TF_Matrix(alpha4, a4, d5, self.q5).subs(dh)
        T5_6 = self.TF_Matrix(alpha5, a5, d6, self.q6).subs(dh)
        T6_ee = self.TF_Matrix(alpha6, a6, d7, q7).subs(dh)

        T0_ee = T0_1 * T1_2 * T2_3 * T3_4 * T4_5 * T5_6 * T6_ee

	# Extract rotation matrices from the transformation matrices
        self.R0_3 = (T0_1 * T1_2 * T2_3)[:3,:3]

        self.r, self.p, self.y = symbols('r p y')
        R_x = Matrix([[1,          0,      0],
                      [0,     cos(self.r),-sin(self.r)],
                      [0,     sin(self.r), cos(self.r)]]) # Roll
        R_y = Matrix([[  cos(self.p),  0,  sin(self.p)],
                      [       0,  1,       0],
                      [ -sin(self.p),  0,  cos(self.p)]]) # pitch
        R_z = Matrix([[cos(self.y), -sin(self.y),   0],
                      [sin(self.y),  cos(self.y),   0],
                      [     0,       0,   1]]) # yaw

        # Correction needed to account for orientation difference 
        # between definition of griffer link in URDF versus DH convention
        Ree_rviz_corr = R_z.subs(self.y, pi) * R_y.subs(self.p, -pi/2)
        R0_rviz_rpy = R_z * R_y * R_x

        self.R0_ee = R0_rviz_rpy * Ree_rviz_corr.transpose()
        ###



    def handle_calculate_IK(self, req):
        rospy.loginfo("Received %s eef-poses from the plan" % len(req.poses))
        if len(req.poses) < 1:
            print "No valid poses received"
            return -1
        else:

            # Initialize service response
            joint_trajectory_list = []
            for i in xrange(0, len(req.poses)):
                # IK code starts here
                joint_trajectory_point = JointTrajectoryPoint()

                # Extract end-effector position and orientation from request
                # px,py,pz = end-effector position
                # roll, pitch, yaw = end-effector orientation
                px = req.poses[i].position.x
                py = req.poses[i].position.y
                pz = req.poses[i].position.z

                (roll, pitch, yaw) = tf.transformations.euler_from_quaternion(
                    [req.poses[i].orientation.x, req.poses[i].orientation.y,
                        req.poses[i].orientation.z, req.poses[i].orientation.w])

                ### Your IK code here 
                # Compensate for rotation discrepancy between DH parameters and Gazebo
                R0_ee = self.R0_ee.subs({self.r: roll, self.p: pitch, self.y: yaw})

                ## Calculate joint angles using Geometric IK method
                # 1. find the location of the WC relative to the base frame.
                EE = Matrix([[px], [py], [pz]])
                WC = (EE - 0.303 * R0_ee[:,2])

                # 2. find theta1, theta2, theta3 from r_WC
                theta1 = atan2(WC[1], WC[0])

                side_a = 1.501 #sqrt(1.5*1.5 + 0.054*0.054)
                side_b = sqrt((sqrt(WC[0]**2 + WC[1]**2) - 0.35)**2 + (WC[2] - 0.75)**2)
                side_c = 1.25 #a2

                angle_a = acos((side_b**2 + side_c**2 - side_a**2)/(2.*side_b*side_c))
                angle_b = acos((side_c**2 + side_a**2 - side_b**2)/(2.*side_c*side_a))
                angle_c = acos((side_a**2 + side_b**2 - side_c**2)/(2.*side_a*side_b))

                theta2 =  pi/2. - angle_a - atan2(WC[2] - 0.75, sqrt(WC[0]**2 + WC[1]**2) - 0.35)
                theta3 =  pi/2. - (angle_b + 0.036) #0.036 = atan2(abs(a3), d4)

                # 3. find R0_3 via application of homogeneous transforms up to the WC.
                R0_3 = self.R0_3.subs({self.q1:theta1, self.q2:theta2, self.q3:theta3})
                R3_6 = R0_3.transpose() * R0_ee

                # 4. find the final set of Euler angles corresponding to the rotation matrix
                # R3_6 = Matrix([
                # [-sin(q4)*sin(q6) + cos(q4)*cos(q5)*cos(q6), -sin(q4)*cos(q6) - sin(q6)*cos(q4)*cos(q5), -sin(q5)*cos(q4)], 
                # [sin(q5)*cos(q6), -sin(q5)*sin(q6), cos(q5)], 
                # [-sin(q4)*cos(q5)*cos(q6) - sin(q6)*cos(q4), sin(q4)*sin(q6)*cos(q5) - cos(q4)*cos(q6), sin(q4)*sin(q5)]])


                theta5 = atan2(sqrt(R3_6[0,2]**2 + R3_6[2,2]**2), R3_6[1,2])
                theta4 = atan2(R3_6[2,2], -R3_6[0,2])
                theta6 = atan2(-R3_6[1,1], R3_6[1,0])


                # Populate response for the IK request
                # In the next line replace theta1,theta2...,theta6 by your joint angle variables
                joint_trajectory_point.positions = [theta1, theta2, theta3, theta4, theta5, theta6]
                joint_trajectory_list.append(joint_trajectory_point)

            rospy.loginfo("length of Joint Trajectory List: %s" % len(joint_trajectory_list))
            return CalculateIKResponse(joint_trajectory_list)


def IK_server():
    # initialize node and declare calculate_ik service
    rospy.init_node('IK_server')
    ik = IK()
    s = rospy.Service('calculate_ik', CalculateIK, ik.handle_calculate_IK)
    print "Ready to receive an IK request"
    rospy.spin()

if __name__ == "__main__":
    IK_server()
