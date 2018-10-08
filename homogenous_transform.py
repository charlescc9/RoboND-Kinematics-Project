#!/usr/bin/env python

from sympy import symbols, cos, sin, pi, simplify, pprint
from sympy.matrices import Matrix

# Create symbols for joint variables
# The numbers 1 to 4 correspond to each rotation in the order specified to you.
q1, q2, q3, q4 = symbols('q1:5')
rtd = 180 / pi
dtr = pi / 180


# Define functions for Rotation Matrices about x, y, and z given specific angle.
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


# Define rotations between frames

# Initial Rotation Matrix for Frame A
Ra = Matrix([[1, 0, 0],
             [0, 1, 0],
             [0, 0, 1]])

# Rotations performed on individual Frames for A->B->E
Rb_a = rot_y(-90 * dtr)
Re_b = rot_x(90 * dtr)

# Rotations performed on individual Frames for A->C->D->E
Rc_a = rot_x(0)
Rd_c = rot_x(90 * dtr)
Re_d = rot_z(90 * dtr)

# Define Translations between frames.
ta = Matrix([[0], [0], [0], [1]])
tb_a = Matrix([[-2], [2], [4], [1]])
te_b = Matrix([[0], [2], [0], [1]])
tc_a = Matrix([[4], [4], [0], [1]])
td_c = Matrix([[-3], [3], [2], [1]])
te_d = Matrix([[-3], [2], [3], [1]])

# Define homogenous transformation matrices
# HINT: Check out sympy's documentation for functions row_join and col_join
Ta = Ra.col_join(Matrix([[0, 0, 0]])).row_join(ta)
Tb_a = Rb_a.col_join(Matrix([[0, 0, 0]])).row_join(tb_a)
Te_b = Re_b.col_join(Matrix([[0, 0, 0]])).row_join(te_b)
Tc_a = Rc_a.col_join(Matrix([[0, 0, 0]])).row_join(tc_a)
Td_c = Rd_c.col_join(Matrix([[0, 0, 0]])).row_join(td_c)
Te_d = Re_d.col_join(Matrix([[0, 0, 0]])).row_join(te_d)

# Composition of Transformations
Te_a_1 = simplify(Ta * Tb_a * Te_b)
Te_a_2 = simplify(Ta * Tc_a * Td_c * Te_d)

# Calculate orientation and position for E
E_1 = Te_a_1.evalf(subs={q1: 0, q2: 0}, chop=True)
E_2 = Te_a_2.evalf(subs={q3: 0, q4: 0}, chop=True)

print("Transformation Matrix for A->B->E:")
pprint(E_1)

print("Transformation Matrix for A->C->D->E:")
pprint(E_2)
