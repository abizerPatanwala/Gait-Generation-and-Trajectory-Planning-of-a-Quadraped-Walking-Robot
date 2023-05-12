#! /usr/bin/env python

import numpy as np  # imports all function so dont have to use np.function()

# Link lengths
a1 = 24  # length of link a1 in mm
a2 = 55  # length of link a2 in mm
a3 = 75  # length of link a3 in mm


# Angles
theta_1 = 0  # theta 1 angle in degrees
theta_2 = -60  # theta 2 angle in degrees
theta_3 = 100 # theta 3 angle in degrees

theta_1 = np.deg2rad(theta_1)  # theta 1 in radians
theta_2 = np.deg2rad(theta_2)  # theta 2 in radians
theta_3 = np.deg2rad(theta_3)  # theta 3 in radians

# DH Parameter Table for 3 DOF robot leg
PT = [[theta_1, 0, a1, 0],
      [np.pi/2, 0, 0, np.pi/2],
      [theta_2, 0, a2, 0],
      [theta_3, 0, a3, 0]]

# Homogeneous Transformation Matrices
i = 0
H0_1 = [[np.cos(PT[i][0]), -np.sin(PT[i][0]), 0, PT[i][2]*np.cos(PT[i][0])],
        [np.sin(PT[i][0]), np.cos(PT[i][0]), 0, PT[i][2]*np.sin(PT[i][0])],
        [0, 0, 1, 0],
        [0, 0, 0, 1]]

i = 1
H1_2 = [[0, 0, 1, 0],
        [1, 0, 0, 0],
        [0, 1, 0, 0],
        [0, 0, 0, 1]]

i = 2
H2_3 = [[np.cos(PT[i][0]), -np.sin(PT[i][0]), 0, PT[i][2]*np.cos(PT[i][0])],
        [np.sin(PT[i][0]), np.cos(PT[i][0]), 0, PT[i][2]*np.sin(PT[i][0])],
        [0, 0, 1, 0],
        [0, 0, 0, 1]]

i = 3
H3_4 = [[np.cos(PT[i][0]), -np.sin(PT[i][0]), 0, PT[i][2]*np.cos(PT[i][0])],
        [np.sin(PT[i][0]), np.cos(PT[i][0]), 0, PT[i][2]*np.sin(PT[i][0])],
        [0, 0, 1, 0],
        [0, 0, 0, 1]]


print("H1_0 =")
print(np.array(H0_1))
print("H1_2 =")
print(np.array(H1_2))
print("H2_3 =")
print(np.array(H2_3))

H0_2 = np.dot(H0_1, H1_2)
H0_3 = np.dot(H0_2, H2_3)
H0_4 = np.dot(H0_3, H3_4)

endpoint_coordinates = []
x = round(H0_4[0][3], 2)
endpoint_coordinates.append(x)
y = round(H0_4[1][3], 2)
endpoint_coordinates.append(y)
z = round(H0_4[2][3], 2)
endpoint_coordinates.append(z)
print("H0_4 =")
print(np.array(H0_4))
print("Wspolrzedne koncowki manipulatora dla podanych wyzej katow:")
print(endpoint_coordinates)
