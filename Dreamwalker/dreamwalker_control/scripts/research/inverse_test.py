#! /usr/bin/env python

from numpy import *


def generate_trajectory():
    # starting point
    x = 24
    y = 75
    z = 8
    point_list = []
    for _ in range(1, 2):
        point = [round(x, 1), y, z]
        point_list.append(point)
        #y -= 5
    print(point_list)
    return point_list


def inverse_kinematics(point):
    # Length of links in cm
    a1 = 24  # length of link a1 in mm
    a2 = 55  # length of link a2 in mm
    a3 = 75  # length of link a3 in mm

    # Desired Position of End effector
    x = point[0]
    y = point[1]
    z = point[2]

    # phi = 90
    # phi = deg2rad(phi)

    p = sqrt((x**2)+(y**2)-(a1**2))
    D = ((x**2)+(y**2)-(a1**2)+(z**2)-(a2**2)-(a3**2))/(2*a2*a3)

    theta_1 = arctan2(y, x) - arctan2(p, a1)
    theta_3 = arctan2(sqrt(1-(D**2)), D)
    theta_2 = arctan2(z, p) - arctan2((a3*sin(theta_3)), (a2+a3*cos(theta_3)))

    #print('theta_1: ', rad2deg(theta_1))
    #print('theta_2: ', rad2deg(theta_2))
    #print('theta_3: ', rad2deg(theta_3))

    theta1 = int(round(rad2deg(theta_1)))
    theta2 = int(round(rad2deg(theta_2)))
    theta3 = int(round(rad2deg(theta_3)))

    angles = [theta1, theta2, theta3]
    return angles


def main():
    trajectory = [[24, 95, 0], [24, 80, 15], [24, 95, 30], [24, 95, 0], [24, 95, -30], [24, 80, -15]]
    list_of_angles = []
    for item in generate_trajectory():
        list_of_angles.append(inverse_kinematics(item))
    print(list_of_angles)


if __name__ == '__main__':
    main()