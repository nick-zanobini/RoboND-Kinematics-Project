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
from IK_variables import *
from mpmath import *
from sympy import *


print("Loaded IK variables")


def handle_calculate_IK(req):
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

            # Calculate wrist center
            P_WC = simplify(P_EE - (0.303) * R0_6[0:3, 0:3] * R_corr * Matrix([[0], [0], [1]]))

            # Calculate theta1, theta2 and theta3
            theta1, theta2, theta3 = calculateTheta1_2_3(s, P_WC)

            R0_3_inv = R0_3.evalf(subs={q1: theta1, q2: theta2, q3: theta3})
            R3_6 = np.array(R0_3_inv * R0_6 * R_corr).astype(np.float64)

            # Calculate theta4, theta5 and theta6
            theta4, theta5, theta6 = rotationMatrixToEulerAngles(R3_6)
            print("(px, py, pz): ({:.4f}, {:.4f}, {:.4f})".format(px, py, pz))
            print("(roll, pitch, yaw): ({:.4f}, {:.4f}, {:.4f})".format(roll, pitch, yaw))
            print(u'\u03B8'"1 = {:.4f}, " u'\u03B8'"2 = {:.4f}, " u'\u03B8'"3 = {:.4f}\n" u'\u03B8'"4 = {:.4f}, " u'\u03B8'"5 = {:.4f}, " u'\u03B8'"6 = {:.4f}\n".format(theta1, theta2, theta3, theta4, theta5, theta6))

        # Populate response for the IK request
        # In the next line replace theta1,theta2...,theta6 by your joint angle variables
        joint_trajectory_point.positions = [theta1, theta2, theta3, theta4, theta5, theta6]
        joint_trajectory_list.append(joint_trajectory_point)
        # print("Calculated all thetas")

        rospy.loginfo("length of Joint Trajectory List: %s" % len(joint_trajectory_list))
        return CalculateIKResponse(joint_trajectory_list)


def IK_server():
    # initialize node and declare calculate_ik service
    rospy.init_node('IK_server')
    s = rospy.Service('calculate_ik', CalculateIK, handle_calculate_IK)
    print("Ready to receive an IK request")
    rospy.spin()


if __name__ == "__main__":
    IK_server()
