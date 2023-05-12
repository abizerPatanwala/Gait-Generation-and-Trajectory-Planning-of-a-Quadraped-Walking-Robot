#! /usr/bin/env python
import rospy

from std_msgs.msg import Float32
from sensor_msgs.msg import Range
from geometry_msgs.msg import Point

sonar_node = {"left_sensor": 'sensor/left_sonar',
				"middle_sensor": 'sensor/middle_sonar',
				"right_sensor": 'sensor/right_sonar'}

class Sonar():
	""" Creates a subscriber to read simulated sensor value """
	def __init__(self, topic_name):
		self._name = topic_name
		self._value = rospy.Subscriber(topic_name, Range, self.reading_value)
		self.reading = 0

	def reading_value(self, msg):
		self.reading = msg.range
		self.reading = round(self.reading, 2)


class SonarSystem():
	""" Writes all sensor values into an array and sends it to fuzzy controller node """
	def __init__(self):
		rospy.init_node('sensor_system')

		rospy.loginfo("Setting up the sensor_system node...")

		self._sensor_pub = rospy.Publisher("sensor_array", Point, queue_size=10)

		self.left_sensor = Sonar(sonar_node["left_sensor"])
		self.middle_sensor = Sonar(sonar_node["middle_sensor"])
		self.right_sensor = Sonar(sonar_node["right_sensor"])
		self.value_list = [0, 0, 0]
		
	def run(self):
		rate =  rospy.Rate(10)
		while not rospy.is_shutdown():
			self.value_list[0] = self.left_sensor.reading if self.left_sensor.reading < 1 else 1.0
			self.value_list[1] = self.middle_sensor.reading if self.middle_sensor.reading < 1 else 1.0
			self.value_list[2] = self.right_sensor.reading if self.right_sensor.reading < 1 else 1.0
			rospy.loginfo("Sensors values: %s", self.value_list)

			self._sensor_pub.publish(x=self.value_list[0], y=self.value_list[1], z=self.value_list[2])

			rate.sleep()

if __name__ == '__main__':
	
	try:
		sonar_system = SonarSystem()
		sonar_system.run()


	except rospy.ROSInterruptException:
		pass
