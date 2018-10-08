#!usr/bin/env python

from sympy import symbols, cos, sin, pi, simplify, sqrt, atan2, pprint
from sympy.matrices import Matrix

###############################################################
# Problem Statement:
# Let P be a vector expressed in frame {B} with (x,y,z)
# coordinates = (15.0, 0.0, 42.0)
# Rotate P about the Y-axis by angle = 110 degrees.
# Then translate the vector 1 unit
# in the X-axis and 30 units in the Z-axis.
# Print the new (x, y, z) coordinates of P after the transformation.
###############################################################
# Create symbols for joint variables
q1 = symbols('q1')
gamma = symbols('gamma')
rtd = 180 / pi
dtr = pi / 180

# Replace P and T with appropriate expressions and calculate new coordinates of P in {A}.
P = Matrix([[15.0], [0.0], [42.0], [1.0]])  # P should be a 4x1 Matrix
T = Matrix([[cos(110 * dtr), 0.0, sin(110 * dtr), 1.0],
           [0.0, 1.0, 0.0, 0.0],
           [-sin(110 * dtr), 0.0, cos(110 * dtr), 30.0],
           [0.0, 0.0, 0.0, 1.0]])  # T Should be a 4x4 homogeneous Transform

pprint(P)
pprint(T)

P_new = 1
