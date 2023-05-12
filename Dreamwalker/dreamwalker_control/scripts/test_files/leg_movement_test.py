#!/usr/bin/env python

import rospy
from std_msgs.msg import Float64, String
from std_msgs.msg import Float32
import math
from dreamwalker_control.srv import Service_GUI_Command, Service_GUI_CommandResponse
import numpy
from threading import Thread, Lock

joint_node = { "shoulder1": "/dreamwalker/shoulder_joint1_position_controller/command", 
				"shoulder2": "/dreamwalker/shoulder_joint2_position_controller/command",
				"shoulder3": "/dreamwalker/shoulder_joint3_position_controller/command",
				"shoulder4": "/dreamwalker/shoulder_joint4_position_controller/command",
				"limb1": "/dreamwalker/limb_joint1_position_controller/command",
				"limb2": "/dreamwalker/limb_joint2_position_controller/command",
				"limb3": "/dreamwalker/limb_joint3_position_controller/command",
				"limb4": "/dreamwalker/limb_joint4_position_controller/command",
				"knee1": "/dreamwalker/knee_joint1_position_controller/command",
				"knee2": "/dreamwalker/knee_joint2_position_controller/command",
				"knee3": "/dreamwalker/knee_joint3_position_controller/command",
				"knee4": "/dreamwalker/knee_joint4_position_controller/command" }

swing1 = ([0.0, -50, 90], [30, -50, 90], [0, -50, 90])
swing2 = ([0.0, -50, 90], [-30, -50, 90], [0, -50, 90])

step_trajectory = ([0.0, -50.0, 90.0], [0.0, -46.0, 90.0], [0.0, -44.0, 90.0], [0.0, -50.0, 90.0], [0.0, -56.0, 90.0], [0.0, -53.0, 90.0])
trajectory = [[0.0, -61.0, 109.0], [0.0, -63.0, 112.0], [0.0, -64.0, 114.0], [0.0, -66.0, 116.0], [0.0, -67.0, 119.0], 
				[0.0, -69.0, 121.0], [1.0, -70.0, 123.0], [1.0, -72.0, 125.0], [1.0, -73.0, 128.0], [1.0, -75.0, 130.0], 
				[1.0, -77.0, 132.0], [1.0, -78.0, 134.0], [2.0, -80.0, 137.0]]


class Leg():
	""" Construct made of 3 servomotors """
	def __init__(self, name, servo1, servo2, servo3):
		self._name = name
		self._shoulder_joint = Joint(command=servo1, up_limit=30, low_limit=(-30))
		self._limb_joint = Joint(command=servo2, up_limit=90, low_limit=(-90))
		self._knee_joint = Joint(command=servo3, up_limit=150, low_limit=-50)
		
		# zmienne na potrzeby skakania jak kretyn
		self.base_position = [0, -50, 90]
		self.prowl_point = [0, -75, 110]
		self.jump_point = [0, 0, 0]
		self.swing1 = [30, -50, 90]
		self.swing2 = [-30, -50, 90]

		# przechowuje informacje na temat aktualnej pozycji nogi
		self.current_pos = [0, 0, 0]

		if self._name == "FL" or self._name == "BL":
			self.test_leg_pos = [30, -75, 110]
		elif self._name == "FR" or self._name == "BR":
			self.test_leg_pos = [-30, -75, 110]


	def set_idle_position(self):
		""" Sets all servomotors to default position """
		self._shoulder_joint._pub.publish(numpy.radians(self.base_position[0]))
		self._limb_joint._pub.publish(numpy.radians(self.base_position[1]))
		self._knee_joint._pub.publish(numpy.radians(self.base_position[2]))

		self.current_pos = [0, -45, 75]


	def move_to_point(self, next_position):
		""" Moves the leg to defined point """
		shoulder_angle = int(next_position[0])
		limb_angle = int(next_position[1])
		knee_angle = int(next_position[2])

		while(self.current_pos[0] != shoulder_angle or self.current_pos[1] != limb_angle or self.current_pos[2] != knee_angle):
			# shoulder joint
			if shoulder_angle > self.current_pos[0]:
				self.current_pos[0] = self.current_pos[0]+1
				self._shoulder_joint._pub.publish(numpy.radians(self.current_pos[0]))
			elif shoulder_angle < self.current_pos[0]:
				self.current_pos[0] = self.current_pos[0]-1
				self._shoulder_joint._pub.publish(numpy.radians(self.current_pos[0]))

			#limb joint
			if limb_angle > self.current_pos[1]:
				self.current_pos[1] = self.current_pos[1]+1
				self._limb_joint._pub.publish(numpy.radians(self.current_pos[1]))
			elif limb_angle < self.current_pos[1]:
				self.current_pos[1] = self.current_pos[1]-1
				self._limb_joint._pub.publish(numpy.radians(self.current_pos[1]))

			# knee joint
			if knee_angle > self.current_pos[2]:
				self.current_pos[2] = self.current_pos[2]+1
				self._knee_joint._pub.publish(numpy.radians(self.current_pos[2]))
			elif knee_angle < self.current_pos[2]:
				self.current_pos[2] = self.current_pos[2]-1
				self._knee_joint._pub.publish(numpy.radians(self.current_pos[2]))
			
			rospy.sleep(0.01)
			#print(self.current_pos)

	def move_trajectory(self, trajectory):
		""" simulates one full movement of a step """
		for item in trajectory:
			self.move_to_point(item)
			rospy.sleep(0.001)


class Joint():
	""" Simulating servomotor """
	def __init__(self, command, up_limit, low_limit):
		self.command = command
		self._pub = rospy.Publisher(self.command, Float64, queue_size=10)
		self.upper_limit = up_limit
		self.lower_limit = low_limit

	def set_joint_value(self, value):
		""" Publishes recieved value to simulated joint """
		self.value = self.deg2rad(value)
		if self.value >= self.upper_limit:
			self.value = self.upper_limit
		elif self.value <= self.lower_limit:
			self.value = self.lower_limit
		
		self._pub.publish(self.value)
		rospy.loginfo(self.value)


class Robot():
	def __init__(self):
		
		rospy.init_node('leg_test')

		rospy.loginfo("Initializing the robot")

		service=rospy.Service('command_service',Service_GUI_Command, self.turn_on_off)
		self.command = -1
		# leg objects
		self._leg1 = Leg(name="FL", servo1=joint_node["shoulder1"], servo2=joint_node["limb1"], servo3=joint_node["knee1"])
		self._leg2 = Leg(name="FR", servo1=joint_node["shoulder2"], servo2=joint_node["limb2"], servo3=joint_node["knee2"])
		self._leg3 = Leg(name="BL", servo1=joint_node["shoulder3"], servo2=joint_node["limb3"], servo3=joint_node["knee3"])
		self._leg4 = Leg(name="BR", servo1=joint_node["shoulder4"], servo2=joint_node["limb4"], servo3=joint_node["knee4"])

	def initialize(self):
		self._leg1.set_idle_position()
		self._leg2.set_idle_position()
		self._leg3.set_idle_position()
		self._leg4.set_idle_position()

	def work_threads(self, t1, t2, t3, t4):
		t1.start()
		t2.start()
		t3.start()
		t4.start()

		t1.join()
		t2.join()
		t3.join()
		t4.join()

	def set_idle(self):
		t1 = Thread(target=self._leg1.move_to_point, args=(self._leg1.base_position,))
		t2 = Thread(target=self._leg2.move_to_point, args=(self._leg2.base_position,))
		t3 = Thread(target=self._leg3.move_to_point, args=(self._leg3.base_position,))
		t4 = Thread(target=self._leg4.move_to_point, args=(self._leg4.base_position,))
		
		self.work_threads(t1, t2, t3, t4)


	def set_jump(self):
		t1 = Thread(target=self._leg1.move_to_point, args=(self._leg1.jump_point,))
		t2 = Thread(target=self._leg2.move_to_point, args=(self._leg2.jump_point,))
		t3 = Thread(target=self._leg3.move_to_point, args=(self._leg3.jump_point,))
		t4 = Thread(target=self._leg4.move_to_point, args=(self._leg4.jump_point,))
		
		self.work_threads(t1, t2, t3, t4)


	def prowl(self):
		t1 = Thread(target=self._leg1.move_to_point, args=(self._leg1.prowl_point,))
		t2 = Thread(target=self._leg2.move_to_point, args=(self._leg2.prowl_point,))
		t3 = Thread(target=self._leg3.move_to_point, args=(self._leg3.prowl_point,))
		t4 = Thread(target=self._leg4.move_to_point, args=(self._leg4.prowl_point,))
		
		self.work_threads(t1, t2, t3, t4)

	def trot(self):
		t1 = Thread(target=self._leg1.move_to_point, args=(self._leg1.base_position,))
		t2 = Thread(target=self._leg2.move_to_point, args=(self._leg2.jump_point,))
		t3 = Thread(target=self._leg3.move_to_point, args=(self._leg3.jump_point,))
		t4 = Thread(target=self._leg4.move_to_point, args=(self._leg4.base_position,))
		
		self.work_threads(t1, t2, t3, t4)

		t1 = Thread(target=self._leg1.move_to_point, args=(self._leg1.jump_point,))
		t2 = Thread(target=self._leg2.move_to_point, args=(self._leg2.base_position,))
		t3 = Thread(target=self._leg3.move_to_point, args=(self._leg3.base_position,))
		t4 = Thread(target=self._leg4.move_to_point, args=(self._leg4.jump_point,))
		
		self.work_threads(t1, t2, t3, t4)

	def swing1(self):
		t1 = Thread(target=self._leg1.move_to_point, args=(self._leg1.swing1,))
		t2 = Thread(target=self._leg2.move_to_point, args=(self._leg2.swing1,))
		t3 = Thread(target=self._leg3.move_to_point, args=(self._leg3.swing2,))
		t4 = Thread(target=self._leg4.move_to_point, args=(self._leg4.swing2,))

		self.work_threads(t1, t2, t3, t4)

	def swing2(self):
		t1 = Thread(target=self._leg1.move_to_point, args=(self._leg1.swing2,))
		t2 = Thread(target=self._leg2.move_to_point, args=(self._leg2.swing2,))
		t3 = Thread(target=self._leg3.move_to_point, args=(self._leg3.swing1,))
		t4 = Thread(target=self._leg4.move_to_point, args=(self._leg4.swing1,))

		self.work_threads(t1, t2, t3, t4)
	
	def turn_on_off(self, request):
		if request.command=="FORWARD":
			self.command = 1
			print("Robot jumps")
			return Service_GUI_CommandResponse("Robot going forward")
		elif request.command=="LEFT":
			self.command = 2
			return Service_GUI_CommandResponse("Robot going left")
		elif request.command=="RIGHT":
			self.command = 3
			return Service_GUI_CommandResponse("Robot going right")
		elif request.command=="SPIN_LEFT":
			self.command = 4
			return Service_GUI_CommandResponse("Robot spins left")
		elif request.command=="SPIN_RIGHT":
			self.command = 5
			return Service_GUI_CommandResponse("Robot spins right")
		elif request.command=="BACKWARD":
			self.command = 6
			return Service_GUI_CommandResponse("Robot goes back")
		elif request.command=="STOP":
			self.command = 0
			print("Robot goes to idle state")
			return Service_GUI_CommandResponse("State: IDLE")

	def run(self):
		rate = rospy.Rate(50)

		self.initialize()

		while not rospy.is_shutdown():
			if self.command == -1:
				self.initialize()
			elif self.command == 0:
				self.set_idle()
			elif self.command == 1:
				try:
					self.set_idle()
					rospy.sleep(0.1)
					self.set_jump()
					rospy.sleep(0.1)
				except TypeError:
					print("something is even worse")
			elif self.command == 2:
				self._leg1.move_trajectory(step_trajectory)
				self._leg1.move_trajectory(swing2)
			elif self.command == 3:
				self.set_idle()
				rospy.sleep(0.1)
				self.swing1()
				rospy.sleep(0.1)
				self.set_idle()
				rospy.sleep(0.1)
				self.swing2()
				rospy.sleep(0.1)
			elif self.command == 4:
				self.trot()
			elif self.command == 5:
				pass
			elif self.command == 6:
				try:
					self.prowl()
				except TypeError:
					print("Welcome to Uganda")	

			rate.sleep()


if __name__ == '__main__':
	try:
		dreamwalker = Robot()
		dreamwalker.run()

	except rospy.ROSInterruptException:
		pass