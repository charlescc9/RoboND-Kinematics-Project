from sympy import *
import math

# Define symbols
alpha0, alpha1, alpha2, alpha3, alpha4, alpha5, alpha6 = symbols('p0:7')
a0, a1, a2, a3, a4, a5, a6 = symbols('a0:7')
d1, d2, d3, d4, d5, d6, d7 = symbols('d1:8')
q1, q2, q3, q4, q5, q6, q7 = symbols('q1:8')

# Define DH parameters
s = {alpha0: 0, a0: 0, d1: 0.75,
     alpha1: -pi / 2, a1: 0.35, d2: 0, q2: q2 - pi / 2,
     alpha2: 0, a2: 1.25, d3: 0,
     alpha3: -pi / 2, a3: -0.054, d4: 1.5,
     alpha4: pi / 2, a4: 0, d5: 0,
     alpha5: -pi / 2, a5: 0, d6: 0,
     alpha6: 0, a6: 0, d7: 0.303, q7: 0}

# Define transformation matrices
T0_1 = Matrix([[cos(q1), -sin(q1), 0, a0],
               [sin(q1) * cos(alpha0), cos(q1) * cos(alpha0), -sin(alpha0), -sin(alpha0) * d1],
               [sin(q1) * sin(alpha0), cos(q1) * sin(alpha0), cos(alpha0), cos(alpha0) * d1],
               [0, 0, 0, 1]])
T0_1 = T0_1.subs(s)

T1_2 = Matrix([[cos(q2), -sin(q2), 0, a1],
               [sin(q2) * cos(alpha1), cos(q2) * cos(alpha1), -sin(alpha1), -sin(alpha1) * d2],
               [sin(q2) * sin(alpha1), cos(q2) * sin(alpha1), cos(alpha1), cos(alpha1) * d2],
               [0, 0, 0, 1]])
T1_2 = T1_2.subs(s)

T2_3 = Matrix([[cos(q3), -sin(q3), 0, a2],
               [sin(q3) * cos(alpha2), cos(q3) * cos(alpha2), -sin(alpha2), -sin(alpha2) * d3],
               [sin(q3) * sin(alpha2), cos(q3) * sin(alpha2), cos(alpha2), cos(alpha2) * d3],
               [0, 0, 0, 1]])
T2_3 = T2_3.subs(s)

T3_4 = Matrix([[cos(q4), -sin(q4), 0, a3],
               [sin(q4) * cos(alpha3), cos(q4) * cos(alpha3), -sin(alpha3), -sin(alpha3) * d4],
               [sin(q4) * sin(alpha3), cos(q4) * sin(alpha3), cos(alpha3), cos(alpha3) * d4],
               [0, 0, 0, 1]])
T3_4 = T3_4.subs(s)

T4_5 = Matrix([[cos(q5), -sin(q5), 0, a4],
               [sin(q5) * cos(alpha4), cos(q5) * cos(alpha4), -sin(alpha4), -sin(alpha4) * d5],
               [sin(q5) * sin(alpha4), cos(q5) * sin(alpha4), cos(alpha4), cos(alpha4) * d5],
               [0, 0, 0, 1]])
T4_5 = T4_5.subs(s)

T5_6 = Matrix([[cos(q6), -sin(q6), 0, a5],
               [sin(q6) * cos(alpha5), cos(q6) * cos(alpha5), -sin(alpha5), -sin(alpha5) * d6],
               [sin(q6) * sin(alpha5), cos(q6) * sin(alpha5), cos(alpha5), cos(alpha5) * d6],
               [0, 0, 0, 1]])
T5_6 = T5_6.subs(s)

T6_G = Matrix([[cos(q7), -sin(q7), 0, a6],
               [sin(q7) * cos(alpha6), cos(q7) * cos(alpha6), -sin(alpha6), -sin(alpha6) * d7],
               [sin(q7) * sin(alpha6), cos(q7) * sin(alpha6), cos(alpha6), cos(alpha6) * d7],
               [0, 0, 0, 1]])
T6_G = T6_G.subs(s)

# Define gripper rotation matrices
R_y = Matrix([[cos(-pi / 2), 0, sin(-pi / 2), 0],
              [0, 1, 0, 0],
              [-sin(-pi / 2), 0, cos(-pi / 2), 0],
              [0, 0, 0, 1]])
R_z = Matrix([[cos(pi), -sin(pi), 0, 0],
              [sin(pi), cos(pi), 0, 0],
              [0, 0, 1, 0],
              [0, 0, 0, 1]])
R_corr = simplify(R_z * R_y)

# Transform from base link to gripper
T = simplify(T0_1 * T1_2 * T2_3 * T3_4 * T4_5 * T5_6 * T6_G * R_corr)

# pprint(T)
result = T.evalf(subs={q1: 0, q2: 0, q3: 0, q4: 0, q5: 0, q6: 0})
# pprint(result)

# cosine_for_pitch = math.sqrt(result[0, 0] ** 2 + result[1, 0] ** 2)
# is_singular = cosine_for_pitch < 10**-6
# if not is_singular:
#     yaw = math.atan2(result[1, 0], result[0, 0])
#     pitch = math.atan2(-result[2, 0], cosine_for_pitch)
#     roll = math.atan2(result[2, 1], result[2, 2])
# else:
#     yaw = math.atan2(-result[1, 2], result[1, 1])
#     pitch = math.atan2(-result[2, 0], cosine_for_pitch)
#     roll = 0
#
# print roll
# print pitch
# print yaw

px = 0.1
py = 0.2
pz = 0.3

roll = 0.4
pitch = 0.5
yaw = 0.6


R_x = Matrix([[1, 0, 0, 0],
              [0, cos(roll), -sin(roll), 0],
              [0, sin(roll), cos(roll), 0],
              [0, 0, 0, 1]])
R_y = Matrix([[cos(pitch), 0, sin(pitch), 0],
              [0, 1, 0, 0],
              [-sin(pitch), 0, cos(pitch), 0],
              [0, 0, 0, 1]])
R_z = Matrix([[cos(yaw), -sin(yaw), 0, 0],
              [sin(yaw), cos(yaw), 0, 0],
              [0, 0, 1, 0],
              [0, 0, 0, 1]])
R_rpy = R_z * R_y * R_x * R_corr

# pprint(R_rpy)

wx = px - s[d7] * R_rpy[0, 2]
wy = py - s[d7] * R_rpy[1, 2]
wz = pz - s[d7] * R_rpy[2, 2]

A = sqrt(s[d4] ** 2 + s[a3] ** 2)
B = sqrt(wx ** 2 + wy ** 2 + (wz - s[d1]) ** 2)
C = s[a2]
b = acos((A ** 2 + C ** 2 - B ** 2) / 2 * A * C)
theta1 = atan2(wy, wx)
theta2 = atan2(sqrt(wx ** 2 + wy ** 2), wz - s[d1])
theta3 = pi / 2 - b - atan2(abs(s[a3]), s[d4])

R0_3 = T0_1 * T1_2 * T2_3
R3_6 = simplify(R0_3.inv(method='LU') * R_rpy)
pprint(R3_6)

theta4 = atan2(R3_6[2, 2], -R3_6[0, 2])
theta5 = atan2(sqrt(R3_6[0, 2] ** 2 + R3_6[2, 2] ** 2), R3_6[1, 2])
theta6 = atan2(-R3_6[1, 1], R3_6[1, 0])

foo = 213
