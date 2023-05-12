#! /usr/bin/env python

import rospy
from std_msgs.msg import Float32
from random import randint

def generate_sensor_value():
	return 1 + randint(1,9)

def main():
	value1 = generate_sensor_value()
	value2 = generate_sensor_value()
	value3 = generate_sensor_value()
	rospy.loginfo("Sent values: %s, %s, %s", value1, value2, value3)
	pub.publish(value1)
	pub2.publish(value2)
	pub3.publish(value3)
	rate.sleep()

if __name__=='__main__':

    rospy.init_node('random_number')
    pub=rospy.Publisher('sensor1_value', Float32, queue_size=10)
    pub2=rospy.Publisher('sensor2_value', Float32, queue_size=10)
    pub3=rospy.Publisher('sensor3_value', Float32, queue_size=10)
    rate= rospy.Rate(10)
    while not rospy.is_shutdown():
    	main()