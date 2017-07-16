#!/usr/bin/env python

# Copyright (C) 2017 Electric Movement Inc.
#
# This file is part of Robotic Arm: Pick and Place project for Udacity
# Robotics nano-degree program
#
# All Rights Reserved.

# Author: Harsh Pandya

# import modules
from __future__ import print_function
import rospy
import tf
from kuka_arm.srv import *
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from geometry_msgs.msg import Pose
from std_msgs.msg import String
from IK_variables import *
from mpmath import *
from sympy import *


print("Loaded IK variables")

global cycles
global pub
cycles = 1


def handle_calculate_IK(req):
    global cycles
    global pub
    rospy.loginfo("Received %s eef-poses from the plan" % len(req.poses))
    if len(req.poses) < 1:
        print("No valid poses received")
        return -1
    else:
        # Initialize service response
        joint_trajectory_list = []
        for x in xrange(0, len(req.poses)):
            # IK code starts here
            joint_trajectory_point = JointTrajectoryPoint()

            # Extract end-effector position and orientation from request
            # px,py,pz = end-effector position
            # roll, pitch, yaw = end-effector orientation
            px = req.poses[x].position.x
            py = req.poses[x].position.y
            pz = req.poses[x].position.z

            (roll, pitch, yaw) = tf.transformations.euler_from_quaternion(
                [req.poses[x].orientation.x, req.poses[x].orientation.y,
                    req.poses[x].orientation.z, req.poses[x].orientation.w])

            # Calculate joint angles using Geometric IK method
            P_EE = Matrix([[px], [py], [pz]])
            R0_6 = R0_6_sym.evalf(subs={r: roll, p: pitch, y: yaw})
            R0_6 = R0_6[0:3, 0:3] * R_corr

            # Calculate wrist center
            P_WC = simplify(P_EE - (0.303) * R0_6 * Matrix([[0], [0], [1]]))

            # Calculate theta1
            theta1 = N(atan2(P_WC[1], P_WC[0]))

            # Calculate theta3
            P0_2 = P_2_sym.evalf(subs={q1: theta1})
            x_c, y_c, z_c = P_WC - P0_2[0:3, :]

            # link lengths
            l_23 = a2
            l_35 = sqrt(a3**2 + d4**2)
            l_25 = sqrt(x_c**2 + y_c**2 + z_c**2)

            # Intermediate angles for theta3
            theta_31 = atan2(a3, d4)
            D1 = (l_25**2 - l_23**2 - l_35**2) / (2 * l_23 * l_35)
            theta_32 = atan2(sqrt(1 - D1**2), D1)
            theta3 = N((theta_32 + theta_31 - (pi / 2)).subs(s))

            # Calculate theta2
            D2 = (l_23**2 - l_35**2 + l_25**2) / (2 * l_23 * l_25)
            theta_21 = atan2(sqrt(1 - D2**2), D2)
            theta_22 = atan2(z_c, sqrt(x_c**2 + y_c**2))
            theta2 = N(((pi / 2) - theta_21 - theta_22).subs(s))

            R0_3_inv = R0_3_inv_sym.evalf(subs={q1: theta1, q2: theta2, q3: theta3})
            R3_6 = np.array(R0_3_inv * R0_6).astype(np.float64)

            # Individual Rotation Matrix elements for theta4 theta5 and theta6 calculations
            r12, r13 = R3_6[0, 1], R3_6[0, 2]
            r21, r22, r23 = R3_6[1, 0], R3_6[1, 1], R3_6[1, 2]
            r32, r33 = R3_6[2, 1], R3_6[2, 2]

            if np.abs(r23) is not 1:
                theta5 = atan2(sqrt(r13**2 + r33**2), r23)
                # workaround for smooth movement
                if sin(theta5) < 0:
                    theta4 = atan2(-r33, r13)
                    theta6 = atan2(r22, -r21)
                else:
                    theta4 = atan2(r33, -r13)
                    theta6 = atan2(-r22, r21)
            else:
                if r23 == 1:
                    theta5 = 0
                    theta4 = -theta6 + atan2(-r12, -r32)
                else:
                    theta5 = 0
                    theta4 = theta6 - atan2(r12, -r32)

            theta1, theta2, theta3 = np.float64(theta1), np.float64(theta2), np.float64(theta3)
            theta4, theta5, theta6 = np.float64(theta4), np.float64(theta5), np.float64(theta6)

            print("(px, py, pz): ({:.4f}, {:.4f}, {:.4f})".format(px, py, pz))
            print("(roll, pitch, yaw): ({:.4f}, {:.4f}, {:.4f})".format(roll, pitch, yaw))
            print(u'\u03B8'"1 = {:.4f}, " u'\u03B8'"2 = {:.4f}, " u'\u03B8'"3 = {:.4f}\n" u'\u03B8'"4 = {:.4f}, " u'\u03B8'"5 = {:.4f}, " u'\u03B8'"6 = {:.4f}\n".format(theta1, theta2, theta3, theta4, theta5, theta6))

        # Populate response for the IK request
        # In the next line replace theta1,theta2...,theta6 by your joint angle variables
        joint_trajectory_point.positions = [np.float64(theta1), np.float64(theta2), np.float64(theta3), np.float64(theta4), np.float64(theta5), np.float64(theta6)]
        joint_trajectory_list.append(joint_trajectory_point)
        # print("Calculated all thetas")
        msg = String()
        msg.data = str(int(floor(cycles / 2)))
        rospy.loginfo("Total # of cylinders picked up: " + str(int(floor(cycles / 2))))
        if (int(cycles) % 2) == 0:
            pub.publish(msg)
        cycles += 1
        rospy.loginfo("length of Joint Trajectory List: %s" % len(joint_trajectory_list))
        return CalculateIKResponse(joint_trajectory_list)


def IK_server():
    global pub
    # initialize node and declare calculate_ik service
    rospy.init_node('IK_server')
    s = rospy.Service('calculate_ik', CalculateIK, handle_calculate_IK)
    pub = rospy.Publisher('cycles', String, queue_size=1)
    print("Ready to receive an IK request")
    rospy.spin()


if __name__ == "__main__":
    IK_server()
