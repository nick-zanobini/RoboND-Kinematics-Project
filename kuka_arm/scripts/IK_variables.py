#!/usr/bin/env python

# This file is part of Robotic Arm: Pick and Place project for Udacity
# Robotics nano-degree program

# Author: Nick Zanobini

import numpy as np
from sympy import symbols, cos, sin, pi, simplify, sqrt, atan2, N
from sympy.matrices import Matrix

# Define DH param symbols
q1, q2, q3, q4, q5, q6, q7 = symbols('q1:8')
d1, d2, d3, d4, d5, d6, d7 = symbols('d1:8')
a0, a1, a2, a3, a4, a5, a6 = symbols('a0:7')
alpha0, alpha1, alpha2, alpha3, alpha4, alpha5, alpha6 = symbols('alpha0:7')
r, p, y = symbols("r, p, y")
rad2deg = 180. / np.pi

# Define Modified DH Transformation matrix
s = {alpha0: 0., a0: 0., d1: 0.75,
     alpha1: -pi / 2, a1: 0.35, d2: 0., q2: q2 - pi / 2.,
     alpha2: 0., a2: 1.25, d3: 0.,
     alpha3: -pi / 2, a3: -0.054, d4: 1.5,
     alpha4: pi / 2, a4: 0., d5: 0.,
     alpha5: -pi / 2, a5: 0., d6: 0.,
     alpha6: 0., a6: 0., d7: 0.303, q7: 0.}


# Rotation matrix along x axis
def rot_x(q):
    R_x = Matrix([[1, 0, 0],
                  [0, cos(q), -sin(q)],
                  [0, sin(q), cos(q)]])

    return R_x


# Rotation matrix along y axis
def rot_y(q):
    R_y = Matrix([[cos(q), 0, sin(q)],
                  [0, 1, 0],
                  [-sin(q), 0, cos(q)]])

    return R_y


# Rotation matrix along z axis
def rot_z(q):
    R_z = Matrix([[cos(q), -sin(q), 0],
                  [sin(q), cos(q), 0],
                  [0, 0, 1]])

    return R_z


def calculateTheta1_2_3(s, P_WC):
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

    return np.float64(theta1), np.float64(theta2), np.float64(theta3)


# Create individual transformation matrices
# base_link to link1
T0_1 = Matrix([[cos(q1), -sin(q1), 0, a0],
               [sin(q1) * cos(alpha0), cos(q1) * cos(alpha0), -sin(alpha0), -sin(alpha0) * d1],
               [sin(q1) * sin(alpha0), cos(q1) * sin(alpha0), cos(alpha0), cos(alpha0) * d1],
               [0, 0, 0, 1]])

T0_1 = T0_1.subs(s)

# linke1 to link 2
T1_2 = Matrix([[cos(q2), -sin(q2), 0, a1],
               [sin(q2) * cos(alpha1), cos(q2) * cos(alpha1), -sin(alpha1), -sin(alpha1) * d2],
               [sin(q2) * sin(alpha1), cos(q2) * sin(alpha1), cos(alpha1), cos(alpha1) * d2],
               [0, 0, 0, 1]])

T1_2 = T1_2.subs(s)

# link2 to link3
T2_3 = Matrix([[cos(q3), -sin(q3), 0, a2],
               [sin(q3) * cos(alpha2), cos(q3) * cos(alpha2), -sin(alpha2), -sin(alpha2) * d3],
               [sin(q3) * sin(alpha2), cos(q3) * sin(alpha2), cos(alpha2), cos(alpha2) * d3],
               [0, 0, 0, 1]])

T2_3 = T2_3.subs(s)

# Create individual transformation matrices
T0_2 = simplify(T0_1 * T1_2)
T0_3 = simplify(T0_2 * T2_3)

# Correction Rotation Matrix
R_corr = rot_z(pi) * rot_y(-pi / 2)
P_2_sym = T0_2 * Matrix([0, 0, 0, 1])
R0_3 = simplify((T0_3[0:3, 0:3]).inv())
R0_6_sym = simplify(rot_z(y) * rot_y(p) * rot_x(r))
