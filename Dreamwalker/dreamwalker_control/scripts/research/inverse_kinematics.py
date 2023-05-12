#! /usr/bin/env python

from numpy import *

# Length of links in cm
a1 = 24  # length of link a1 in mm
a2 = 55  # length of link a2 in mm
a3 = 65  # length of link a3 in mm

# Desired Position of End effector
x = 24
y = 60
z = 0

# phi = 90
# phi = deg2rad(phi)

p = sqrt((x**2)+(y**2)-(a1**2))
D = ((x**2)+(y**2)-(a1**2)+(z**2)-(a2**2)-(a3**2))/(2*a2*a3)

theta_1 = arctan2(y, x) - arctan2(p, a1)
theta_3 = arctan2(sqrt(1-(D**2)), D)
theta_2 = arctan2(z, p) - arctan2((a3*sin(theta_3)), (a2+a3*cos(theta_3)))

theta1 = int(round(rad2deg(theta_1)))
theta2 = int(round(rad2deg(theta_2)))
theta3 = int(round(rad2deg(theta_3)))


print('theta_1: ', theta1)
print('theta_2: ', theta2)
print('theta_3: ', theta3)