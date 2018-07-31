import driver
import config
import datetime
import requests
import numpy as np
from threading import Thread

class Planner(Thread):
	__instance = None
	degToRad = np.pi / 180
	radToDeg = 180 * np.pi

	@staticmethod
	def getInstance():
		""" Static access method. """
		if Planner.__instance == None:
			Planner()
		return Planner.__instance 

	"""docstring for Planner"""
	def __init__(self):
		if Planner.__instance != None:
			raise Exception("This class is a singleton!")
		else:
			Thread.__init__()
			Planner.__instance = self

		self.driver_base_l = driver.DriverMotor(config.url, config.BASE_MOTOR_ID_L)
		self.driver_base_r = driver.DriverMotor(config.url, config.BASE_MOTOR_ID_R)
		self.driver_lift_l = driver.DriverMotor(config.url, config.LIFT_MOTOR_ID_L)
		self.driver_lift_r = driver.DriverMotor(config.url, config.LIFT_MOTOR_ID_R)
		self.driver_middle = driver.DriverMotor(config.url, config.MIDDLE_MOTOR_ID)
		self.driver_turret = driver.DriverMotor(config.url, config.TURRET_MOTOR_ID)
		self.driver_forward = driver.DriverMotor(config.url, config.FORWARD_MOTOR_ID)
		# self.driver_end_effector = driver.DriverServo(config.url, config.END_EFFECTOR_MOTOR_ID)

		self.driver_base_l.set_pulse_per_mm(config.encoder_pulse_base_l)
		self.driver_base_r.set_pulse_per_mm(config.encoder_pulse_base_r)
		self.driver_lift_l.set_pulse_per_mm(config.encoder_pulse_lift_l)
		self.driver_lift_r.set_pulse_per_mm(config.encoder_pulse_lift_r)
		self.driver_middle.set_pulse_per_mm(config.encoder_pulse_middle)
		self.driver_turret.set_pulse_per_mm(config.encoder_pulse_turret)
		self.driver_forward.set_pulse_per_mm(config.encoder_pulse_forward)

		self.position = [0 for i in range(7)] # middle, lift, base, turret, forward
		self.goal_pos = [0 for i in range(7)] # middle, lift, base, turret, forward
		self.velocity = [0 for i in range(7)] # middle, lift, base, turret, forward
		self.goal_vel = [0 for i in range(7)] # middle, lift, base, turret, forward

		self.delay_time = datetime.datetime.now()
		self.update_count = 0

		self.__is_running = True
			
	def __getattr__(self, name):
		return getattr(self.instance, name)

	def move_x(self, pos_mm, vel = 100):
		pos = min(max(pos_mm, 0), config.workspace_x)
		self.goal_pos[0] = pos

	def move_y(self, pos_mm, vel = 100):
		pos = min(max(pos_mm, 0), config.workspace_y)
		self.goal_pos[1] = pos

	def move_z(self, pos_mm, vel = 100):
		pos = min(max(pos_mm, 0), config.workspace_z)
		self.goal_pos[2] = pos

	def move_arm_x(self, pos_mm, vel = 100):
		# prepare
		x_left = pos_mm - self.position[0]
		x = x_left if np.abs(x_left) < config.arm_workspace else (config.arm_dist_from_forward + config.arm_dist_from_joint_turret)
		x_left = 0 if x == x_left else x_left - (config.arm_dist_from_forward + config.arm_dist_from_joint_turret) * (-1 if np.abs(x_left) < 0 else 1)

		# pre outupt
		z = self.position[6]
		d = np.sqrt(x**2 + z**2) - config.arm_dist_from_joint_turret - config.arm_dist_from_forward
		theta = np.arctan(y/x)

		# set output
		self.move_x(self.position[0] + x_left, vel)
		self.goal_pos[3] = theta * Planner.radToDeg
		self.goal_pos[4] = d

	def move_arm_y(self, pos_mm, vel = 100):
		self.move_y(pos_mm, vel)

	def move_arm_z(self, pos_mm, vel = 100):
		# prepare
		z_left = pos_mm - self.position[2]
		z = x_left if np.abs(z_left) < config.arm_workspace else (config.arm_dist_from_forward + config.arm_dist_from_joint_turret)
		z_left = 0 if x == z_left else z_left - (config.arm_dist_from_forward + config.arm_dist_from_joint_turret) * (-1 if np.abs(z_left) < 0 else 1)

		# pre outupt
		x = self.position[5]
		d = np.sqrt(x**2 + z**2) - config.arm_dist_from_joint_turret - config.arm_dist_from_forward
		theta = np.arctan(y/x)

		# set output
		self.move_z(self.position[2] + x_left, vel)
		self.goal_pos[3] = theta * Planner.radToDeg
		self.goal_pos[4] = d

	def get_pos(self):
		return self.position[:3]

	def get_pos_arm(self):
		return [self.position[0] + self.position[5], 
				self.position[1], 
				self.position[2] + self.position[6]]

	def loop(self):
		diff_time = (datetime.datetime.now() - self.delay_time).total_milliseconds
		if diff_time > config.planner_update_time:
			for i in range(5):
				self.position[i] = self.position[i] + max(0, math.abs(self.goal_pos[i] - self.position[i])) * diff_time * (-1 if self.goal_pos[i] < 0 else 1)

			self.__send_position()
			self.delay_time = datetime.datetime.now()
			self.update_count += 1

		if self.update_count >= 10:
			self.__update()
			self.update_count = 0

	def __update(self):
		self.position[0], self.velocity[0] = driver_middle.get_current() # mm, (pulse / ms)
		self.position[1], self.velocity[1] = driver_lift_l.get_current() # mm, (pulse / ms)
		self.position[2], self.velocity[2] = driver_base_l.get_current() # mm, (pulse / ms)
		self.position[3], self.velocity[3] = driver_turret.get_current() # deg, (pulse / ms)
		self.position[4], self.velocity[4] = driver_forward.get_current() # mm, (pulse / ms)
		
		self.position[5] = (config.arm_dist_from_joint_turret + config.arm_dist_from_forward + self.position[4]) * np.cos(self.position[3] * Planner.degToRad) # arm x
		self.position[6] = (config.arm_dist_from_joint_turret + config.arm_dist_from_forward + self.position[4]) * np.sin(self.position[3] * Planner.degToRad) # arm y

	def __send_position(self):
		url = "{}/multi/status/set".format(config.url)
		
		data = {
			"token": generate_otp(),
			config.BASE_MOTOR_ID_L: {
				"goto_pos": self.position[2] * config.encoder_pulse_base_l, # pulse mm * (pulse per mm)
				"goto_velo": self.velocity[2],
			},
			config.BASE_MOTOR_ID_R: {
				"goto_pos": self.position[2] * config.encoder_pulse_base_r, # pulse mm * (pulse per mm)
				"goto_velo": self.velocity[2],
			},
			config.LIFT_MOTOR_ID_L: {
				"goto_pos": self.position[1] * config.encoder_pulse_lift_l, # pulse mm * (pulse per mm)
				"goto_velo": self.velocity[1],
			},
			config.LIFT_MOTOR_ID_R: {
				"goto_pos": self.position[1] * config.encoder_pulse_lift_r, # pulse mm * (pulse per mm)
				"goto_velo": self.velocity[1],
			},
			config.MIDDLE_MOTOR_ID: {
				"goto_pos": self.position[0] * config.encoder_pulse_middle, # pulse mm * (pulse per mm)
				"goto_velo": self.velocity[0],
			},
			config.TURRET_MOTOR_ID: {
				"goto_pos": self.position[3] * config.encoder_pulse_turret, # pulse mm * (pulse per deg)
				"goto_velo": self.velocity[3],
			},
			config.FORWARD_MOTOR_ID: {
				"goto_pos": self.position[4] * config.encoder_pulse_forward, # pulse mm * (pulse per mm)
				"goto_velo": self.velocity[4],
			},
		}

		result = requests.post(url, data=data)
		if (result.status_code != 200)
			print ('HTTP Error: {}'.format(result.status_code))

		return json.loads(result.text)

	def run(self):
		while self.__is_running:
			self.loop()

		self.__is_running = False
		
	def stop(self):
		self.__is_running = False