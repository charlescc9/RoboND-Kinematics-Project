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


def handle_calculate_IK(req):
    rospy.loginfo("Received %s eef-poses from the plan" % len(req.poses))
    if len(req.poses) < 1:
        print "No valid poses received"
        return -1
    else:

        # Symbols
        alpha0, alpha1, alpha2, alpha3, alpha4, alpha5, alpha6 = symbols('p0:7')
        a0, a1, a2, a3, a4, a5, a6 = symbols('a0:7')
        d1, d2, d3, d4, d5, d6, d7 = symbols('d1:8')
        q1, q2, q3, q4, q5, q6, q7 = symbols('q1:8')
        r, p, y = symbols('r, p, y')
        p_x, p_y, p_z = symbols('p_x, p_y, p_z')

        # DH parameters
        dh_params = {alpha0: 0, a0: 0, d1: 0.75,
                     alpha1: -pi / 2, a1: 0.35, d2: 0, q2: q2 - pi / 2,
                     alpha2: 0, a2: 1.25, d3: 0,
                     alpha3: -pi / 2, a3: -0.054, d4: 1.5,
                     alpha4: pi / 2, a4: 0, d5: 0,
                     alpha5: -pi / 2, a5: 0, d6: 0,
                     alpha6: 0, a6: 0, d7: 0.303, q7: 0}

        # Transformation matrices
        T0_1 = Matrix([[cos(q1), -sin(q1), 0, a0],
                       [sin(q1) * cos(alpha0), cos(q1) * cos(alpha0), -sin(alpha0), -sin(alpha0) * d1],
                       [sin(q1) * sin(alpha0), cos(q1) * sin(alpha0), cos(alpha0), cos(alpha0) * d1],
                       [0, 0, 0, 1]]).subs(dh_params)
        T1_2 = Matrix([[cos(q2), -sin(q2), 0, a1],
                       [sin(q2) * cos(alpha1), cos(q2) * cos(alpha1), -sin(alpha1), -sin(alpha1) * d2],
                       [sin(q2) * sin(alpha1), cos(q2) * sin(alpha1), cos(alpha1), cos(alpha1) * d2],
                       [0, 0, 0, 1]]).subs(dh_params)
        T2_3 = Matrix([[cos(q3), -sin(q3), 0, a2],
                       [sin(q3) * cos(alpha2), cos(q3) * cos(alpha2), -sin(alpha2), -sin(alpha2) * d3],
                       [sin(q3) * sin(alpha2), cos(q3) * sin(alpha2), cos(alpha2), cos(alpha2) * d3],
                       [0, 0, 0, 1]]).subs(dh_params)
        T3_4 = Matrix([[cos(q4), -sin(q4), 0, a3],
                       [sin(q4) * cos(alpha3), cos(q4) * cos(alpha3), -sin(alpha3), -sin(alpha3) * d4],
                       [sin(q4) * sin(alpha3), cos(q4) * sin(alpha3), cos(alpha3), cos(alpha3) * d4],
                       [0, 0, 0, 1]]).subs(dh_params)
        T4_5 = Matrix([[cos(q5), -sin(q5), 0, a4],
                       [sin(q5) * cos(alpha4), cos(q5) * cos(alpha4), -sin(alpha4), -sin(alpha4) * d5],
                       [sin(q5) * sin(alpha4), cos(q5) * sin(alpha4), cos(alpha4), cos(alpha4) * d5],
                       [0, 0, 0, 1]]).subs(dh_params)
        T5_6 = Matrix([[cos(q6), -sin(q6), 0, a5],
                       [sin(q6) * cos(alpha5), cos(q6) * cos(alpha5), -sin(alpha5), -sin(alpha5) * d6],
                       [sin(q6) * sin(alpha5), cos(q6) * sin(alpha5), cos(alpha5), cos(alpha5) * d6],
                       [0, 0, 0, 1]]).subs(dh_params)
        T6_G = Matrix([[cos(q7), -sin(q7), 0, a6],
                       [sin(q7) * cos(alpha6), cos(q7) * cos(alpha6), -sin(alpha6), -sin(alpha6) * d7],
                       [sin(q7) * sin(alpha6), cos(q7) * sin(alpha6), cos(alpha6), cos(alpha6) * d7],
                       [0, 0, 0, 1]]).subs(dh_params)
        T0_G = T0_1 * T1_2 * T2_3 * T3_4 * T4_5 * T5_6 * T6_G

        # Rotation matrices
        R_x = Matrix([[1, 0, 0],
                      [0, cos(r), -sin(r)],
                      [0, sin(r), cos(r)]])
        R_y = Matrix([[cos(p), 0, sin(p)],
                      [0, 1, 0],
                      [-sin(p), 0, cos(p)]])
        R_z = Matrix([[cos(y), -sin(y), 0],
                      [sin(y), cos(y), 0],
                      [0, 0, 1]])
        R_corr = R_z.subs(y, pi) * R_y.subs(p, -pi / 2)
        R = R_z * R_y * R_x * R_corr
        t = Matrix([[p_x],
                    [p_y],
                    [p_z]])
        T0_G_homogeneous = R.col_insert(3, t).row_insert(3, Matrix([[0, 0, 0, 1]]))

        # Initialize service response
        joint_trajectory_list = []
        for x in xrange(0, len(req.poses)):
            joint_trajectory_point = JointTrajectoryPoint()

            # Extract end-effector position and orientation from request
            px = req.poses[x].position.x
            py = req.poses[x].position.y
            pz = req.poses[x].position.z
            (roll, pitch, yaw) = tf.transformations.euler_from_quaternion(
                [req.poses[x].orientation.x, req.poses[x].orientation.y,
                 req.poses[x].orientation.z, req.poses[x].orientation.w])

            # Wrist position
            R = R.subs({'r': roll, 'p': pitch, 'y': yaw})
            wx = px - dh_params[d7] * R[0, 2]
            wy = py - dh_params[d7] * R[1, 2]
            wz = pz - dh_params[d7] * R[2, 2]

            # Joint angles 1 - 3
            A = round(sqrt(dh_params[d4] ** 2 + dh_params[a3] ** 2), 3)
            B = sqrt((sqrt(wx ** 2 + wy ** 2) - dh_params[a1]) ** 2 + (wz - dh_params[d1]) ** 2)
            C = dh_params[a2]
            a = acos((B ** 2 + C ** 2 - A ** 2) / (2 * B * C))
            b = acos((A ** 2 + C ** 2 - B ** 2) / (2 * A * C))
            theta1 = atan2(wy, wx)
            theta2 = pi / 2 - a - atan2(wz - dh_params[d1], sqrt(wx ** 2 + wy ** 2) - dh_params[a1])
            theta3 = pi / 2 - b - round(atan2(abs(dh_params[a3]), dh_params[d4]), 3)

            # Join angles 4 - 6
            R0_3 = T0_1[0:3, 0:3] * T1_2[0:3, 0:3] * T2_3[0:3, 0:3]
            R0_3 = R0_3.evalf(subs={q1: theta1, q2: theta2, q3: theta3})
            R3_6 = R0_3.inv('LU') * R
            theta4 = atan2(R3_6[2, 2], -R3_6[0, 2])
            theta5 = atan2(sqrt(R3_6[0, 2] ** 2 + R3_6[2, 2] ** 2), R3_6[1, 2])
            theta6 = atan2(-R3_6[1, 1], R3_6[1, 0])

            # Populate response for the IK request
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
