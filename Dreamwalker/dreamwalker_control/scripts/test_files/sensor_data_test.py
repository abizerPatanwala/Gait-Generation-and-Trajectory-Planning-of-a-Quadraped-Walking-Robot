#! /usr/bin/env python

import rospy
from sensor_msgs.msg import Range
from std_msgs.msg import Float32

class SonarValues():
	def __init__(self):
		rospy.loginfo("Setting up the SensorValues node...")

		#initialize the node
		rospy.init_node('Sensor_value_reader')

		#Creating subscribers
		self._sub_sensor1 = rospy.Subscriber('sensor/left_sonar', Range, self.read_sensor1)
		self._sub_sensor2 = rospy.Subscriber('sensor/middle_sonar', Range, self.read_sensor2)
		self._sub_sensor3 = rospy.Subscriber('sensor/right_sonar', Range, self.read_sensor3)

		self.value1 = 0
		self.value2 = 0
		self.value3 = 0
		self.sensor_values = [0, 0, 0]

	def read_sensor1(self, msg):
		self.value1 = round(msg.range, 2)
		self.sensor_values[0] = self.value1

	def read_sensor2(self, msg):
		self.value2 = round(msg.range, 2)
		self.sensor_values[1] = self.value2

	def read_sensor3(self, msg):
		self.value3 = round(msg.range, 2)
		self.sensor_values[2] = self.value3

	def run(self):
		rate =  rospy.Rate(10)
		while not rospy.is_shutdown():
			rospy.loginfo("Sensors values: %s", self.sensor_values)
			rate.sleep()


if __name__ == '__main__':
	
	try:
		sonar_system = SonarValues()
		sonar_system.run()


	except rospy.ROSInterruptException:
		pass
