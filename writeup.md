## Project: Kinematics Pick & Place

[//]: # (Image References)

[image1]: robot_figure.png
[image2]: theta_figure.png
[image3]: pick_and_place1.png
[image4]: pick_and_place2.png

### Kinematic Analysis
#### 1. Run the forward_kinematics demo and evaluate the kr210.urdf.xacro file to perform kinematic analysis of Kuka KR210 robot and derive its DH parameters.

In this section, I derived the DH parameter table for use in the forward
and inverse kinematics calculations. I accomplished this by first
drawing a diagram of the robot KUKA KR210 and labeling all joints and
links. I then defined a reference frame with Z and X axes
for each joint along with the gripper end effector. I chose the Z axes to
point along the joint axes, and the X axes to be normal to the Z axes. I
placed most frame origins at the center of their corresponding joint,
though I placed joints 4 and 6 coincident with joint 5,
to allow for kinematic decoupling of the gripper end effector. I then
calculated the alpha, a, d, and theta values for each frame using John
J. Craig's modified DH convention.

Links | alpha(i-1) | a(i-1) | d(i-1) | theta(i)
--- | --- | --- | --- | ---
0->1 | 0 | 0 | 0.75 | q1
1->2 | -pi/2 | 0.35 | 0 | q2 - pi/2
2->3 | 0 | 1.25 | 0 | q3
3->4 |  -pi / 2 | -0.054 | 1.5 | q4
4->5 | pi / 2 | 0 | 0 | q5
5->6 | -pi / 2 | 0 | 0 | q6
6->EE | 0 | 0 | 0.303 | 0

![alt text][image1]

#### 2. Using the DH parameter table you derived earlier, create individual transformation matrices about each joint. In addition, also generate a generalized homogeneous transform between base_link and gripper_link using only end-effector(gripper) pose.

In this section, I used the DH parameter table to create transform
matrices between adjacent frames and a homogenous transform matrix around from
the base to the gripper end effector. I created these matrices using
Sympy. For the DH transform matrices, I first defined the symbols:
```
alpha0, alpha1, alpha2, alpha3, alpha4, alpha5, alpha6 = symbols('p0:7')
a0, a1, a2, a3, a4, a5, a6 = symbols('a0:7')
d1, d2, d3, d4, d5, d6, d7 = symbols('d1:8')
q1, q2, q3, q4, q5, q6, q7 = symbols('q1:8')
```
I then defined the DH parameter table:
```
dh_params = {alpha0: 0, a0: 0, d1: 0.75,
             alpha1: -pi / 2, a1: 0.35, d2: 0, q2: q2 - pi / 2,
             alpha2: 0, a2: 1.25, d3: 0,
             alpha3: -pi / 2, a3: -0.054, d4: 1.5,
             alpha4: pi / 2, a4: 0, d5: 0,
             alpha5: -pi / 2, a5: 0, d6: 0,
             alpha6: 0, a6: 0, d7: 0.303, q7: 0}
```
I then defined all the transform matrices between adjacent frames, using
John J. Craig's modified DH convention and substituting in
the corresponding values from the DH parameter table:
```
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
```
Finally I created the complete DH transform matrix between the base and
gripper end effector via subsequent matrix multiplications:
```
T0_G = T0_1 * T1_2 * T2_3 * T3_4 * T4_5 * T5_6 * T6_G
```

For the homogenous transform matrix, I first created the necessary
symbols:
```
r, p, y = symbols('r, p, y')
p_x, p_y, p_z = symbols('p_x, p_y, p_z')
```
Then I created rotation matrices around each axis:
```
R_x = Matrix([[1, 0, 0],
              [0, cos(r), -sin(r)],
              [0, sin(r), cos(r)]])
R_y = Matrix([[cos(p), 0, sin(p)],
              [0, 1, 0],
              [-sin(p), 0, cos(p)]])
R_z = Matrix([[cos(y), -sin(y), 0],
              [sin(y), cos(y), 0],
              [0, 0, 1]])
```
I then created a matrix to correct for the difference in orientation of
the gripper end effector as described in the URDF description and the
DH parameter table (a 180 degree rotation around the Z axis and a -90
degree rotation around the Y axis):
```
R_corr = R_z.subs(y, pi) * R_y.subs(p, -pi / 2)
```
I then calculated the complete rotation matrix via subsequent matrix
multiplications:
```
R = R_z * R_y * R_x * R_corr
```
This rotation matrix sufficed for the inverse kinematics calculations, but to obtain a
complete homogeneous transform matrix, I also defined a translation
column vector and added it to the rotation matrix (along with the
bottom row):
```
t = Matrix([[p_x],
            [p_y],
            [p_z]])
T0_G_homogeneous = R.col_insert(3, t).row_insert(3, Matrix([[0, 0, 0, 1]]))
```

#### 3. Decouple Inverse Kinematics problem into Inverse Position Kinematics and inverse Orientation Kinematics; doing so derive the equations to calculate all individual joint angles.

In this part, I used inverse kinematics to calculate the joint angles
(theta 1 - 6) given the gripper end effector position and orientation.
Since I placed joints 4, 5, and 6 in intersecting reference frames, I
was able to kinematic decouple the position and orientation of the
gripper end effector, splitting the problem into inverse position
kinematics and inverse orientation kinematics. I used inverse position
kinematics to calculate theta 1 - 3. To do this, I first calculated the
wrist (reference frames 4, 5, and 6) positions by performing a rotation
and translation from the gripper end effector:
```
R = R.subs({'r': roll, 'p': pitch, 'y': yaw})
wx = px - dh_params[d7] * R[0, 2]
wy = py - dh_params[d7] * R[1, 2]
wz = pz - dh_params[d7] * R[2, 2]
```
This allowed me to solve closed-form trigonometry equations to get the
values for theta 1 - 3:
```
A = round(sqrt(dh_params[d4] ** 2 + dh_params[a3] ** 2), 3)
B = sqrt((sqrt(wx ** 2 + wy ** 2) - dh_params[a1]) ** 2 + (wz - dh_params[d1]) ** 2)
C = dh_params[a2]
a = acos((B ** 2 + C ** 2 - A ** 2) / (2 * B * C))
b = acos((A ** 2 + C ** 2 - B ** 2) / (2 * A * C))
theta1 = atan2(wy, wx)
theta2 = pi / 2 - a - atan2(wz - dh_params[d1], sqrt(wx ** 2 + wy ** 2) - dh_params[a1])
theta3 = pi / 2 - b - round(atan2(abs(dh_params[a3]), dh_params[d4]), 3)
```

![alt text][image2]

Finally, I was able to use inverse orientation kinematics to solve for
theta 4 - 6. I accomplished this by first setting the DH transform
matrix equal to the homogeneous transform matrix and multiplying both by
the DH transform matrix for the first three joint angles and
substituting the theta values found above. This gave the DH transform
matrix for the last three theta values. I was then able to extract the
euler angles for theta 4 - 6:
```
R0_3 = T0_1[0:3, 0:3] * T1_2[0:3, 0:3] * T2_3[0:3, 0:3]
R0_3 = R0_3.evalf(subs={q1: theta1, q2: theta2, q3: theta3})
R3_6 = R0_3.inv('LU') * R
theta4 = atan2(R3_6[2, 2], -R3_6[0, 2])
theta5 = atan2(sqrt(R3_6[0, 2] ** 2 + R3_6[2, 2] ** 2), R3_6[1, 2])
theta6 = atan2(-R3_6[1, 1], R3_6[1, 0])
```

### Project Implementation

#### 1. Fill in the `IK_server.py` file with properly commented python code for calculating Inverse Kinematics based on previously performed Kinematic Analysis. Your code must guide the robot to successfully complete 8/10 pick and place cycles. Briefly discuss the code you implemented and your results.

In this section, I placed my developed code above into the IK_server.py
in order to calculate the KUKA KR210 inverse kinematics for the pick and
place task. I first calculated the DH transform matrix and homogeneous
transform matrix. I then iterated through the poses sent by rviz,
calculated theta 1 - 6 for each pose, and returned the values as a
list of JointTrajectoryPoints. Running this in simulation via rviz and
Gazebo gave good results; the KUKA KR210 arm correctly placed the
blue cylinder into the dropbox 9 out of 10 trials. That being said, I
may have reached better accuracy through a more rigorous analysis of how
to choose theta angles with more than one solution. My code also ran
fairly quickly, roughly 0.5sec per pose, but optimization could have
certainly improved performance.
![alt text][image3]
![alt text][image4]


