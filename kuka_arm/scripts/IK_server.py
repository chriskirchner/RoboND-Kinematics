#!/usr/bin/env python

# Copyright (C) 2017 Udacity Inc.
#
# This file is part of Robotic Arm: Pick and Place project for Udacity
# Robotics nano-degree program
#
# All Rights Reserved.

# Author: Harsh Pandya

# import modules
import rospy
import tf
from kuka_arm.srv import *
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from geometry_msgs.msg import Pose
from mpmath import *
from sympy import *


def rot_x(q):
    R_x = Matrix([[1, 0, 0], [0, cos(q), -sin(q)], [0, sin(q), cos(q)]])

    return R_x


def rot_y(q):
    R_y = Matrix([[cos(q), 0, sin(q)], [0, 1, 0], [-sin(q), 0, cos(q)]])

    return R_y

def rot_z(q):
    R_z = Matrix([[cos(q), -sin(q), 0], [sin(q), cos(q), 0], [0, 0, 1]])

    return R_z

def handle_calculate_IK(req):
    rospy.loginfo("Received %s eef-poses from the plan" % len(req.poses))
    if len(req.poses) < 1:
        print "No valid poses received"
        return -1
    else:

        ### Your FK code here
        # Create symbols
        q, q1, q2, q3, q4, q5, q6, q7 = symbols('q q1:8')
        d1, d2, d3, d4, d5, d6, d7 = symbols('d1:8')
        a0, a1, a2, a3, a4, a5, a6 = symbols('a0:7')
        alpha0, alpha1, alpha2, alpha3, alpha4, alpha5, alpha6 = symbols('alpha0:7')

        # Create Modified DH parameters
        s = {alpha0: 0, a0: 0, d1: 0.75,
             alpha1: -pi / 2, a1: 0.35, d2: 0, q2: q2 - pi / 2,
             alpha2: 0, a2: 1.25, d3: 0,
             alpha3: -pi / 2, a3: -0.054, d4: 1.50,
             alpha4: pi / 2, a4: 0, d5: 0,
             alpha5: -pi / 2, a5: 0, d6: 0,
             alpha6: 0, a6: 0, d7: 0.303, q7: 0}

        # Define Modified DH Transformation matrix
        def df_transform_matrix(alpha, a, d, q):
            return  Matrix([[cos(q), -sin(q), 0, a],
                    [sin(q) * cos(alpha), cos(q) * cos(alpha), -sin(alpha), -sin(alpha) * d],
                    [sin(q) * sin(alpha), cos(q) * sin(alpha), cos(alpha), cos(alpha) * d],
                    [0, 0, 0, 1]])

        # Create individual transformation matrices
        T0_1 = df_transform_matrix(s[alpha0], s[a0], s[d1], q1)
        T1_2 = df_transform_matrix(s[alpha1], s[a1], s[d2], s[q2])
        T2_3 = df_transform_matrix(s[alpha2], s[a2], s[d3], q3)


        # Extract rotation matrices from the transformation matrices
        R0_1 = T0_1[0:3, 0:3]
        R1_2 = T1_2[0:3, 0:3]
        R2_3 = T2_3[0:3, 0:3]


        R_z = Matrix([
            [cos(pi), -sin(pi), 0, 0],
            [sin(pi), cos(pi), 0, 0],
            [0, 0, 1, 0],
            [0, 0, 0, 1]
        ])

        R_y = Matrix([
            [cos(-pi / 2), 0, sin(-pi / 2), 0],
            [0, 1, 0, 0],
            [-sin(-pi / 2), 0, cos(-pi / 2), 0],
            [0, 0, 0, 1]
        ])

        R_corr = R_z * R_y

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

            ### Your IK code here
	        # Compensate for rotation discrepancy between DH parameters and Gazebo
            Rrpy = rot_z(yaw) * rot_y(pitch) * rot_x(roll) * R_corr[:3, :3]

            # Calculate joint angles using Geometric IK method
            # calculate wrist center position from ros adjusted orientation
            nx, ny, nz = Rrpy[:, 2][:]
            d = s[d6]
            l = s[d7]
            wx = px - (d + l) * nx
            wy = py - (d + l) * ny
            wz = pz - (d + l) * nz

            # calculate parameters needed for a and b angles
            a_side = sqrt(s[d4] ** 2 + s[a3] ** 2)
            c_side = s[a2]
            r = sqrt(wx ** 2 + wy ** 2)
            tx = r - s[a1]
            tz = wz - s[d1]
            b_side = sqrt(tx ** 2 + tz ** 2)

            # calculate a and b angles
            a_angle = acos((b_side ** 2 + c_side ** 2 - a_side ** 2) / (2 * b_side * c_side))
            b_angle = acos((a_side ** 2 + c_side ** 2 - b_side ** 2) / (2 * a_side * c_side))

            # calculate thetas 1 through 3
            theta1 = (atan2(wy, wx)).evalf()
            theta2 = (pi / 2 - a_angle - atan2(tz, tx)).evalf()
            theta3 = (pi / 2 - b_angle + atan2(s[a3], s[d4])).evalf()

            # calculate rotation matrix for first three thetas
            R01 = R0_1.evalf(subs={q1: theta1})
            R12 = R1_2.evalf(subs={q2: theta2})
            R23 = R2_3.evalf(subs={q3: theta3})
            R0_3 = R01 * R12 * R23

            # remove affect of first three angles from rotation of end effector to get rotation of last three thetas
            R3_6 = R0_3.inv() * Rrpy

            # derive last three thetas after wrist center
            theta5 = atan2(sqrt(R3_6[0, 2] ** 2 + R3_6[2, 2] ** 2), R3_6[1, 2])
            # # got conditionals from https://udacity-robotics.slack.com/archives/C5HUQ0HB9/p1517885796000276
            # if sin(theta5) < 0:
            #     theta6 = atan2(R3_6[1, 1], -R3_6[1, 0])
            #     theta4 = atan2(-R3_6[2, 2], R3_6[0, 2])
            # else:
            #     theta6 = atan2(-R3_6[1, 1], R3_6[1, 0])
            #     theta4 = atan2(R3_6[2, 2], -R3_6[0, 2])
            theta6 = atan2(-R3_6[1, 1], R3_6[1, 0])
            theta4 = atan2(R3_6[2, 2], -R3_6[0, 2])

            # Populate response for the IK request
            # In the next line replace theta1,theta2...,theta6 by your joint angle variables
            joint_trajectory_point.positions = [theta1, theta2, theta3, theta4, theta5, theta6]
            joint_trajectory_list.append(joint_trajectory_point)

        rospy.loginfo("length of Joint Trajectory List: %s" % len(joint_trajectory_list))
        return CalculateIKResponse(joint_trajectory_list)


def IK_server():
    # initialize node and declare calculate_ik service
    rospy.init_node('IK_server')
    s = rospy.Service('calculate_ik', CalculateIK, handle_calculate_IK)
    print "Ready to receive an IK request"
    rospy.spin()

if __name__ == "__main__":
    IK_server()
