#!/usr/bin/env python

import math
from threading import Thread, Lock

import numpy
import rospy

from std_msgs.msg import Float64, String
from std_msgs.msg import Float32, Int8

from dreamwalker_control.srv import Service_GUI_Command, Service_GUI_CommandResponse


""" dictionary of joint command topic names """
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

gait_p1 = ([0, -77, 110], [0, -62, 111], [0, -48, 98])
gait_p2 = ([0, -52, 99], [0, -55, 99], [0, -58, 100])
gait_p3 = ([0, -60, 100], [0, -63, 100], [0, -65, 99])
gait_p4 = ([0, -68, 99], [0, -70, 98], [0, -72, 97])

step1 = ([0, -67, 99], [0, -68, 112], [0, -53, 99], [0, -60, 100])
step2 = ([0, -53, 99], [0, -60, 100], [0, -67, 99], [0, -68, 112])

step1_b = ([0, -53, 99], [0, -68, 112], [0, -67, 99], [0, -60, 100])
step2_b = ([0, -67, 99], [0, -60, 100], [0, -53, 99], [0, -68, 112])

left1 = ([0, -68, 112], [12, -55, 91])
right1 = ([0, -68, 112], [-12, -55, 91])

spin1 = ([0, -68, 112], [12, -55, 91])
spin2 = ([0, -68, 112], [-12,-55, 91])

side_to_base = ([0, -68, 112], [0, -60, 100])

swing1 = ([0.0, -64, 95], [10, -64, 95], [0, -64, 95])
swing2 = ([0.0, -64, 95], [-10, -64, 95], [0, -64, 95])


class Leg():
	""" Construct made of 3 servomotors """
	def __init__(self, name, servo1, servo2, servo3):
		self._name = name
		self._shoulder_joint = Joint(command=servo1, up_limit=30, low_limit=(-30))
		self._limb_joint = Joint(command=servo2, up_limit=90, low_limit=(-90))
		self._knee_joint = Joint(command=servo3, up_limit=150, low_limit=-50)
		
		# angles of leg's base position
		self.base_position = [0, -60, 100]
		self.enabled = None

		# current leg position angles
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

		self.current_pos = [0, -60, 100]


	def move_to_point(self, next_position, speed):
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
			
			rospy.sleep(speed)
			#print(self.current_pos)


	def make_step(self, trajectory, speed):
		""" simulates one full movement of a step """
		for item in trajectory:
			self.move_to_point(item, speed)
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
		
		rospy.init_node('robot_control')

		rospy.loginfo("Initializing the robot")

		service=rospy.Service('command_service',Service_GUI_Command, self.get_command)
		self.command = -1

		self._steering_value_sub = rospy.Subscriber('steering_value', Int8, self.update_steering_value)
		self.steering = -1

		self.mode = 0

		# leg objects
		self._leg1 = Leg(name="FL", servo1=joint_node["shoulder1"], servo2=joint_node["limb1"], servo3=joint_node["knee1"])
		self._leg2 = Leg(name="FR", servo1=joint_node["shoulder2"], servo2=joint_node["limb2"], servo3=joint_node["knee2"])
		self._leg3 = Leg(name="BL", servo1=joint_node["shoulder3"], servo2=joint_node["limb3"], servo3=joint_node["knee3"])
		self._leg4 = Leg(name="BR", servo1=joint_node["shoulder4"], servo2=joint_node["limb4"], servo3=joint_node["knee4"])

	def update_steering_value(self, value):
		self.steering = value.data

	def initialize(self):
		""" setting robot legs to base position"""
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
		self.speed = 0.01

		t1 = Thread(target=self._leg1.move_to_point, args=(self._leg1.base_position, self.speed,))
		t2 = Thread(target=self._leg2.move_to_point, args=(self._leg2.base_position, self.speed,))
		t3 = Thread(target=self._leg3.move_to_point, args=(self._leg3.base_position, self.speed,))
		t4 = Thread(target=self._leg4.move_to_point, args=(self._leg4.base_position, self.speed,))
		
		self.work_threads(t1, t2, t3, t4)

	def gait_init(self):
		self.speed = 0.01
		self._leg1.move_to_point(gait_p4[2], self.speed)
		self._leg4.move_to_point(gait_p3[2], self.speed)
		self._leg2.move_to_point(gait_p2[2], self.speed)
		self._leg3.move_to_point(gait_p1[2], self.speed)

	def gait(self):
		self.speed1 = 0.01
		self.speed2 = 0.01

		t1 = Thread(target=self._leg1.make_step, args=(gait_p1, self.speed1,))
		t2 = Thread(target=self._leg2.make_step, args=(gait_p3, self.speed2,))
		t3 = Thread(target=self._leg3.make_step, args=(gait_p2, self.speed2,))
		t4 = Thread(target=self._leg4.make_step, args=(gait_p4, self.speed2,))
		self.work_threads(t1, t2, t3, t4)

		t1 = Thread(target=self._leg1.make_step, args=(gait_p2, self.speed2,))
		t2 = Thread(target=self._leg2.make_step, args=(gait_p4, self.speed2,))
		t3 = Thread(target=self._leg3.make_step, args=(gait_p3, self.speed2,))
		t4 = Thread(target=self._leg4.make_step, args=(gait_p1, self.speed1,))
		self.work_threads(t1, t2, t3, t4)

		t1 = Thread(target=self._leg1.make_step, args=(gait_p3, self.speed2,))
		t2 = Thread(target=self._leg2.make_step, args=(gait_p1, self.speed1,))
		t3 = Thread(target=self._leg3.make_step, args=(gait_p4, self.speed2,))
		t4 = Thread(target=self._leg4.make_step, args=(gait_p2, self.speed2,))
		self.work_threads(t1, t2, t3, t4)

		t1 = Thread(target=self._leg1.make_step, args=(gait_p4, self.speed2))
		t2 = Thread(target=self._leg2.make_step, args=(gait_p2, self.speed2))
		t3 = Thread(target=self._leg3.make_step, args=(gait_p1, self.speed1))
		t4 = Thread(target=self._leg4.make_step, args=(gait_p3, self.speed2))
		self.work_threads(t1, t2, t3, t4)

	def trot(self):
		self.speed = 0.01
		t1 = Thread(target=self._leg1.make_step, args=(step1, self.speed,))
		t2 = Thread(target=self._leg2.make_step, args=(step2, self.speed,))
		t3 = Thread(target=self._leg3.make_step, args=(step2, self.speed,))
		t4 = Thread(target=self._leg4.make_step, args=(step1, self.speed,))
		
		self.work_threads(t1, t2, t3, t4)

	def go_left(self):
		self.speed = 0.01

		self._leg1.make_step(left1, self.speed)
		self._leg3.make_step(left1, self.speed)
		
		t1 = Thread(target=self._leg1.move_to_point, args=(self._leg1.base_position, self.speed,))
		t2 = Thread(target=self._leg2.move_to_point, args=(right1[1], self.speed,))
		t3 = Thread(target=self._leg3.move_to_point, args=(self._leg3.base_position, self.speed,))
		t4 = Thread(target=self._leg4.move_to_point, args=(right1[1], self.speed,))

		self.work_threads(t1, t2, t3, t4)

		self._leg2.make_step(side_to_base, self.speed)
		self._leg4.make_step(side_to_base, self.speed)

	def go_right(self):
		self.speed = 0.01

		self._leg2.make_step(right1, self.speed)
		self._leg4.make_step(right1, self.speed)

		t1 = Thread(target=self._leg1.move_to_point, args=(left1[1], self.speed,))
		t2 = Thread(target=self._leg2.move_to_point, args=(self._leg2.base_position, self.speed,))
		t3 = Thread(target=self._leg3.move_to_point, args=(left1[1], self.speed,))
		t4 = Thread(target=self._leg4.move_to_point, args=(self._leg4.base_position, self.speed,))

		self.work_threads(t1, t2, t3, t4)

		self._leg1.make_step(side_to_base, self.speed)
		self._leg3.make_step(side_to_base, self.speed)

	def spin_left(self):
		self.speed = 0.01

		self._leg1.make_step(left1, self.speed)
		self._leg4.make_step(right1, self.speed)

		t1 = Thread(target=self._leg1.move_to_point, args=(self._leg1.base_position, self.speed,))
		t2 = Thread(target=self._leg2.move_to_point, args=(right1[1], self.speed,))
		t3 = Thread(target=self._leg3.move_to_point, args=(left1[1], self.speed,))
		t4 = Thread(target=self._leg4.move_to_point, args=(self._leg4.base_position, self.speed,))

		self.work_threads(t1, t2, t3, t4)

		self._leg2.make_step(side_to_base, self.speed)
		self._leg3.make_step(side_to_base, self.speed)

	def spin_right(self):
		self.speed = 0.01

		self._leg2.make_step(right1, self.speed)
		self._leg3.make_step(left1, self.speed)

		t1 = Thread(target=self._leg1.move_to_point, args=(left1[1], self.speed,))
		t2 = Thread(target=self._leg2.move_to_point, args=(self._leg2.base_position, self.speed,))
		t3 = Thread(target=self._leg3.move_to_point, args=(self._leg3.base_position, self.speed,))
		t4 = Thread(target=self._leg4.move_to_point, args=(right1[1], self.speed,))

		self.work_threads(t1, t2, t3, t4)

		self._leg1.make_step(side_to_base, self.speed)
		self._leg4.make_step(side_to_base, self.speed)

	def go_backwards(self):
		self.speed = 0.01

		t1 = Thread(target=self._leg1.make_step, args=(step1_b, self.speed,))
		t2 = Thread(target=self._leg2.make_step, args=(step2_b, self.speed,))
		t3 = Thread(target=self._leg3.make_step, args=(step2_b, self.speed,))
		t4 = Thread(target=self._leg4.make_step, args=(step1_b, self.speed,))
		
		self.work_threads(t1, t2, t3, t4)

	
	def get_command(self, request):
		if request.command=="FORWARD":
			self.command = 1
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
			return Service_GUI_CommandResponse("State: IDLE")
		elif request.command=="AUTO":
			self.mode = 1
			return Service_GUI_CommandResponse("Switched to autonomous mode")
		elif request.command=="MANUAL":
			self.mode = 0
			return Service_GUI_CommandResponse("Switched to manual mode")

	def run(self):
		rate = rospy.Rate(50)

		while not rospy.is_shutdown():
			if self.mode == 0:
				if self.command == -1:
					self.initialize()
				elif self.command == 0:
					self.set_idle()
				elif self.command == 1:
					self.gait_init()
					while self.command == 1:
						self.gait()
						if self.steering == int(0):
							self.command = 0
						elif self.steering == int(2):
							self.command = 4
						elif self.steering == int(3):
							self.command = 5
				elif self.command == 2:
					self.go_left()
				elif self.command == 3:
					self.go_right()
				elif self.command == 4:
					self.spin_left()
				elif self.command == 5:
					self.spin_right()
				elif self.command == 6:
					self.go_backwards()
			elif self.mode == 1:
				if self.steering == 2:
					self.gait()
				elif self.steering == 1:
					self.spin_left()
				elif self.steering == 3:
					self.spin_right()	

			rate.sleep()


if __name__ == '__main__':
	try:
		dreamwalker = Robot()
		dreamwalker.run()

	except rospy.ROSInterruptException:
		pass