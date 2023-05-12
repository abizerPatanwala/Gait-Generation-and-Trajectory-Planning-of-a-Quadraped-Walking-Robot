#! /usr/bin/env python

import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import numpy as np  # imports all function so don't have to use np.function()


def simple_kinematics(a, b, c):
    """" Takes angles as arguments, returns x y z coordinates of the end effector"""

    # lnk lengths

    a1 = 24  # length of link a1 in mm
    a2 = 55  # length of link a2 in mm
    a3 = 65  # length of link a3 in mm

    # Angles

    theta_1 = np.deg2rad(a)  # theta 1 in radians
    theta_2 = np.deg2rad(b)  # theta 2 in radians
    theta_3 = np.deg2rad(c)  # theta 3 in radians

    # DH Parameter Table for 3 DOF robot leg
    pt = [[theta_1, 0, a1, 0],
          [np.pi/2, 0, 0, np.pi/2],
          [theta_2, 0, a2, 0],
          [theta_3, 0, a3, 0]]

    # Homogeneous Transformation Matrices
    n = 0
    H0_1 = [[np.cos(pt[n][0]), -np.sin(pt[n][0]), 0, pt[n][2]*np.cos(pt[n][0])],
            [np.sin(pt[n][0]), np.cos(pt[n][0]), 0, pt[n][2]*np.sin(pt[n][0])],
            [0, 0, 1, 0],
            [0, 0, 0, 1]]

    H1_2 = [[0, 0, 1, 0],
            [1, 0, 0, 0],
            [0, 1, 0, 0],
            [0, 0, 0, 1]]

    n = 2
    H2_3 = [[np.cos(pt[n][0]), -np.sin(pt[n][0]), 0, pt[n][2]*np.cos(pt[n][0])],
            [np.sin(pt[n][0]), np.cos(pt[n][0]), 0, pt[n][2]*np.sin(pt[n][0])],
            [0, 0, 1, 0],
            [0, 0, 0, 1]]

    n = 3
    H3_4 = [[np.cos(pt[n][0]), -np.sin(pt[n][0]), 0, pt[n][2]*np.cos(pt[n][0])],
            [np.sin(pt[n][0]), np.cos(pt[n][0]), 0, pt[n][2]*np.sin(pt[n][0])],
            [0, 0, 1, 0],
            [0, 0, 0, 1]]

    H0_2 = np.dot(H0_1, H1_2)
    H0_3 = np.dot(H0_2, H2_3)
    H0_4 = np.dot(H0_3, H3_4)

    endpoint_coordinates = []
    x_coordinate = round(H0_4[0][3], 2)
    endpoint_coordinates.append(x_coordinate)
    y_coordinate = round(H0_4[1][3], 2)
    endpoint_coordinates.append(y_coordinate)
    z_coordinate = round(H0_4[2][3], 2)
    endpoint_coordinates.append(z_coordinate)

    return endpoint_coordinates


if __name__ == "__main__":
    list_of_lists = []
    fi1 = []
    fi2 = []
    fi3 = []
    for i in range(-30, 31, 5):  # range of shoulder servo in angles
        fi1.append(i)
        for j in range(-90, 1, 10):  # range of arm servo in angles
            fi2.append(j)
            for k in range(0, 121, 10):  # range of knee servo in angles
                fi3.append(k)
                point = simple_kinematics(i, j, k)
                list_of_lists.append(point)
    x = []
    y = []
    z = []
    for element in list_of_lists:
        x.append(element[0])
        y.append(-(element[1]))
        z.append(element[2])

    #print(x)
    #print(y)
    #print(z)

    #plt.scatter(z, y, s=1)
    #plt.xlabel("odleglosc w przod")
    #plt.ylabel("odleglosc w dol")
    #plt.show()

    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')
    ax.scatter(x, z, y, label="Working space points", c="red", s=1)
    ax.legend()
    plt.xlabel("X axis")
    plt.ylabel("Z axis")
    plt.show()