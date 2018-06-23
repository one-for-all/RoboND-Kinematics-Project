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


dtr = pi/180.


def dh_transform(alpha, a, d, theta):
    # Takes angles in radians
    return Matrix([[           cos(theta),           -sin(theta),           0,             a],
                   [sin(theta)*cos(alpha), cos(theta)*cos(alpha), -sin(alpha), -sin(alpha)*d],
                   [sin(theta)*sin(alpha), cos(theta)*sin(alpha),  cos(alpha),  cos(alpha)*d],
                   [                    0,                     0,           0,             1]])


def rot_x(q):
    R_x = Matrix([[1, 0, 0],
                  [0, cos(q), -sin(q)],
                  [0, sin(q), cos(q)]])

    return R_x


def rot_y(q):
    R_y = Matrix([[cos(q), 0, sin(q)],
                  [0, 1, 0],
                  [-sin(q), 0, cos(q)]])

    return R_y


def rot_z(q):
    R_z = Matrix([[cos(q), -sin(q), 0],
                  [sin(q), cos(q), 0],
                  [0, 0, 1]])

    return R_z


def init_constants():
    d1 = 0.33 + 0.42
    a1 = 0.35
    a2 = 1.25
    a3 = -0.054
    d4 = 0.96 + 0.54
    dg = 0.193 + 0.11

    return d1, a1, a2, a3, d4, dg


def forward_dh_transform(q1, q2, q3, q4, q5, q6):
    d1, a1, a2, a3, d4, dg = init_constants()

    # Define Modified DH Transformation matrix
    T0_1 = dh_transform(0, 0, d1, q1)
    T1_2 = dh_transform(-pi / 2, a1, 0, -pi / 2 + q2)
    T2_3 = dh_transform(0, a2, 0, q3)
    T3_4 = dh_transform(-pi / 2, a3, d4, q4)
    T4_5 = dh_transform(pi / 2, 0, 0, q5)
    T5_6 = dh_transform(-pi / 2, 0, 0, q6)
    T6_g = dh_transform(0, 0, dg, 0)

    return (T0_1, T1_2, T2_3, T3_4, T4_5, T5_6, T6_g), (d1, a1, a2, a3, d4, dg)


def compute_IK(req, calculate_fk=False):
    ### Your FK code here
    #  Create symbols
    q1, q2, q3, q4, q5, q6 = symbols('q1:7')

    # Create Modified DH parameters

    dh_transforms, constants = forward_dh_transform(q1, q2, q3, q4, q5, q6)
    T0_1, T1_2, T2_3, T3_4, T4_5, T5_6, T6_g = dh_transforms
    d1, a1, a2, a3, d4, dg = constants

    R_corr = rot_z(pi) * rot_y(-pi / 2)

    # Create individual transformation matrices
    T0_4 = T0_1 * T1_2 * T2_3 * T3_4
    T_corr = R_corr.row_join(Matrix([[0], [0], [0]])).col_join(Matrix([[0, 0, 0, 1]]))
    T0_g = T0_4 * T4_5 * T5_6 * T6_g * T_corr
    # Extract rotation matrices from the transformation matrices
    R0_1 = T0_1[0:3, 0:3]
    R1_2 = T1_2[0:3, 0:3]
    R2_3 = T2_3[0:3, 0:3]
    R3_4 = T3_4[0:3, 0:3]

    R0_3 = R0_1 * R1_2 * R2_3
    ###

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

        # Calculate joint angles using Geometric IK method
        Rrpy = rot_z(yaw) * rot_y(pitch) * rot_x(roll) * R_corr.transpose()
        pee_target = Matrix([[px], [py], [pz]])
        pwc_target = pee_target - dg * Rrpy[:, 2]

        theta1 = atan2(pwc_target[1], pwc_target[0])

        h = pwc_target[2] - d1
        l = sqrt(pwc_target[1] * pwc_target[1] + pwc_target[0] * pwc_target[0]) - a1
        phi_interval = atan2(h, l)
        A = sqrt(d4 * d4 + a3 * a3)
        B = a2
        C = sqrt(h * h + l * l)
        a = acos((B * B + C * C - A * A) / (2 * B * C))
        theta2 = pi / 2 - a - phi_interval

        rho = atan2(a3, d4) + pi/2
        c = acos((B * B + A * A - C * C) / (2 * B * A))
        theta3 = pi - c - rho

        # Compute last 3 q
        R0_3_inv = R0_3.evalf(subs={q1: theta1, q2: theta2, q3: theta3}).transpose()
        R3_6 = R0_3_inv * Rrpy

        r13 = R3_6[0, 2]
        r33 = R3_6[2, 2]
        r23 = R3_6[1, 2]
        r21 = R3_6[1, 0]
        r22 = R3_6[1, 1]
        p = sqrt(r13 * r13 + r33 * r33)
        theta5 = atan2(p, r23)
        theta4 = atan2(r33 / p, -r13 / p)
        theta6 = atan2(-r22 / p, r21 / p)
        ###

        # Populate response for the IK request
        # In the next line replace theta1,theta2...,theta6 by your joint angle variables
        joint_trajectory_point.positions = [theta1, theta2, theta3, theta4, theta5, theta6]

        pwc, pee, euler_angles = [None] * 3
        if calculate_fk:
            params = {q1: theta1,
                      q2: theta2,
                      q3: theta3,
                      q4: theta4,
                      q5: theta5,
                      q6: theta6}

            T0_4_eval = T0_4.evalf(subs=params)
            pwc = T0_4_eval[0:3, 3]

            T0_g_eval = T0_g.evalf(subs=params)
            pee = T0_g_eval[0:3, 3]
            R_XYZ = T0_g_eval[0:3, 0:3]
            r21 = R_XYZ[1, 0]
            r11 = R_XYZ[0, 0]
            r31 = R_XYZ[2, 0]
            r32 = R_XYZ[2, 1]
            r33 = R_XYZ[2, 2]

            alpha = atan2(r21, r11)  # rotation about Z-axis
            beta = atan2(-r31, sqrt(r11 * r11 + r21 * r21))  # rotation about Y-axis
            gamma = atan2(r32, r33)  # rotation about X-axis
            euler_angles = Matrix([gamma, beta, alpha])
        
        cartesian_positions = (pwc, pee, euler_angles)
        joint_trajectory_list.append((joint_trajectory_point, cartesian_positions))

    return joint_trajectory_list


def handle_calculate_IK(req):
    rospy.loginfo("Received %s eef-poses from the plan" % len(req.poses))
    if len(req.poses) < 1:
        print "No valid poses received"
        return -1
    else:
        joint_trajectory_list = compute_IK(req)
        response_list = [item[0] for item in joint_trajectory_list]
        rospy.loginfo("length of Joint Trajectory List: %s" % len(response_list))
        return CalculateIKResponse(response_list)


def IK_server():
    # initialize node and declare calculate_ik service
    rospy.init_node('IK_server')
    s = rospy.Service('calculate_ik', CalculateIK, handle_calculate_IK)
    print "Ready to receive an IK request"
    rospy.spin()

if __name__ == "__main__":
    IK_server()
