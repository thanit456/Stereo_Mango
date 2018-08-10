import driver
import config
import datetime
import requests
import numpy as np
import threading

class Planner:
	__instance = None
	degToRad = np.pi / 180
	radToDeg = 180 * np.pi

	sendMultiple = True

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
			Planner.__instance = self

		self.lock = threading.Lock()

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

		self.__is_running = True
			
	def __getattr__(self, name):
		return getattr(self.instance, name)

	def move_stereo_cam(self, x_pos, y_pos, z_pos, speed):
		self._update()

		return self.move_to_stereo_cam(self.position[0] + x_pos, self.position[1] + y_pos, self.position[2] + z_pos, speed)

	def move_to_stereo_cam(self, x_pos, y_pos, z_pos, speed, is_set = False): # pos in mm, velo in mm/s
		# check input
		x_pos = min(max(x_pos, 0), config.workspace_x)
		y_pos = min(max(y_pos, 0), config.workspace_y)
		z_pos = min(max(z_pos, 0), config.workspace_z)

		# cal velocity
		self._update()

		x_diff = np.abs(x_pos - self.position[0])
		y_diff = np.abs(y_pos - self.position[1])
		z_diff = np.abs(z_pos - self.position[2])
		
		line_length = np.sqrt(x_diff**2 + y_diff**2 + z_diff**2)
		overall_time = line_length / speed # in seconds (mm / mm * s)

		x_velo = x_diff / overall_time # mm / seconds
		y_velo = y_diff / overall_time
		z_velo = z_diff / overall_time

		# set output
		self.goal_pos[0] = x_pos
		self.goal_pos[1] = y_pos
		self.goal_pos[2] = z_pos
		# self.goal_pos[3] = self.position[3]
		# self.goal_pos[4] = self.position[4]

		self.goal_vel[0] = x_velo * 1000
		self.goal_vel[1] = y_velo * 1000
		self.goal_vel[2] = z_velo * 1000
		# self.goal_vel[3] = 0
		# self.goal_vel[4] = 0

		if is_set is False:
			self._send_position()		

	def move_single_cam(self, x_pos, y_pos, z_pos, deg, speed):
		# self._update()

		# prepare
		z_remain = 0 if self.position[4] + z_pos < config.arm_max_workspace else (self.position[4] + z_pos) - config.arm_max_workspace
		z_remain_x = z_remain * np.cos(self.position[3])
		z_remain_z = z_remain * np.sin(self.position[3])

		x_pos_x = x_pos * np.cos(self.position[3] - np.pi / 2)
		x_pos_z = x_pos * np.sin(self.position[3] - np.pi / 2)

		z_pos = min(max(self.position[4] + z_pos, config.arm_min_workspace), config.arm_max_workspace)
		z_diff = np.abs(self.position[4] - z_pos)

		line_length = np.sqrt(x_pos**2, y_pos**2, z_pos**2)
		overall_time = line_length / speed
		z_velo = z_diff / overall_time

		self.move_stereo_cam(z_remain_x + x_pos_x, y_pos, z_remain_z + x_pos_z, speed, True)
		self.turn_arm(deg, speed, True)
		self.goal_pos[4] = self.position[4] + z_pos
		self.goal_vel[4] = z_velo * 1000

		self._send_position()

	def turn_arm(self, deg, speed, is_set = False):
		self.turn_to_arm(self.position[3] * Planner.radToDeg + deg, speed)

	def turn_to_arm(self, deg, speed, is_set = False):
		rad = (deg  % 360) * Planner.degToRad
		rad_diff = np.abs(self.position[3] - rad)
		linear = rad_diff * self.position[4]
		time = linear / speed

		velo = rad_diff * Planner.radToDeg / time

		self.goal_pos[3] = self.position[3] + rad
		self.goal_vel[3] = velo * 1000

		if is_set is False:
			self._send_position()

	def get_pos(self):
		self._update()
		return self.position[:3]

	def get_pos_arm(self):
		self._update()
		return [self.position[0] + self.position[5], 
				self.position[1], 
				self.position[2] + self.position[6]]

	def _update(self):
		time_diff = (datetime.datetime.now() - self.delay_time).microseconds
		if time_diff < config.planner_update_time:
			return

		result = {}

		if Planner.sendMultiple:
			url = "{}/get".format(config.url)
			data = {
				"token": generate_otp(),
				str(config.BASE_MOTOR_ID_L): ["cur_pos", "cur_velo"],
				str(config.BASE_MOTOR_ID_R): ["cur_pos", "cur_velo"],
				str(config.LIFT_MOTOR_ID_L): ["cur_pos", "cur_velo"],
				str(config.LIFT_MOTOR_ID_R): ["cur_pos", "cur_velo"],
				str(config.MIDDLE_MOTOR_ID): ["cur_pos", "cur_velo"],
				str(config.TURRET_MOTOR_ID): ["cur_pos", "cur_velo"],
				str(config.FORWARD_MOTOR_ID): ["cur_pos", "cur_velo"],
			}

			tmp = requests.post(url, json=data).json()
			result[str(config.MIDDLE_MOTOR_ID)] = [tmp[str(config.MIDDLE_MOTOR_ID)]['cur_pos'] * driver_middle.ppmm, tmp[str(config.MIDDLE_MOTOR_ID)]['cur_velo']]
			result[str(config.LIFT_MOTOR_ID_L)] = [tmp[str(config.LIFT_MOTOR_ID_L)]['cur_pos'] * driver_middle.ppmm, tmp[str(config.LIFT_MOTOR_ID_L)]['cur_velo']]
			result[str(config.BASE_MOTOR_ID_L)] = [tmp[str(config.BASE_MOTOR_ID_L)]['cur_pos'] * driver_middle.ppmm, tmp[str(config.BASE_MOTOR_ID_L)]['cur_velo']]
			result[str(config.TURRET_MOTOR_ID)] = [tmp[str(config.TURRET_MOTOR_ID)]['cur_pos'] * driver_middle.ppmm, tmp[str(config.TURRET_MOTOR_ID)]['cur_velo']]
			result[str(config.FORWARD_MOTOR_ID)] = [tmp[str(config.FORWARD_MOTOR_ID)]['cur_pos'] * driver_middle.ppmm, tmp[str(config.FORWARD_MOTOR_ID)]['cur_velo']]
		else:
			result[str(config.MIDDLE_MOTOR_ID)] = driver_middle.get_current()
			result[str(config.LIFT_MOTOR_ID_L)] = driver_lift_l.get_current()
			result[str(config.BASE_MOTOR_ID_L)] = driver_base_l.get_current() 
			result[str(config.TURRET_MOTOR_ID)] = driver_turret.get_current() 
			result[str(config.FORWARD_MOTOR_ID)] = driver_forward.get_current() 

		self.position[0], self.velocity[0] = result[str(config.MIDDLE_MOTOR_ID)] # mm, (pulse / ms)
		self.position[1], self.velocity[1] = result[str(config.LIFT_MOTOR_ID_L)] # mm, (pulse / ms)
		self.position[2], self.velocity[2] = result[str(config.BASE_MOTOR_ID_L)] # mm, (pulse / ms)
		self.position[3], self.velocity[3] = result[str(config.TURRET_MOTOR_ID)] # deg, (pulse / ms)
		self.position[4], self.velocity[4] = result[str(config.FORWARD_MOTOR_ID)] # mm, (pulse / ms)
		
		self.position[5] = (config.arm_min_workspace + self.position[4]) * np.cos(self.position[3] * Planner.degToRad) # arm x
		self.position[6] = (config.arm_min_workspace + self.position[4]) * np.sin(self.position[3] * Planner.degToRad) # arm z

		self.delay_time = datetime.datetime.now()

	def _send_position(self):
		if Planner.sendMultiple:
			url = "{}/set".format(config.url)
			
			data = {
				"token": generate_otp(),
				config.BASE_MOTOR_ID_L: {
					"goto_pos": self.position[2] * config.encoder_pulse_base_l, # pulse mm * (pulse per mm)
					"goto_velo": self.velocity[2] * config.encoder_pulse_base_l,
				},
				config.BASE_MOTOR_ID_R: {
					"goto_pos": self.position[2] * config.encoder_pulse_base_r, # pulse mm * (pulse per mm)
					"goto_velo": self.velocity[2] * config.encoder_pulse_base_r,
				},
				config.LIFT_MOTOR_ID_L: {
					"goto_pos": self.position[1] * config.encoder_pulse_lift_l, # pulse mm * (pulse per mm)
					"goto_velo": self.velocity[1] * config.encoder_pulse_lift_l,
				},
				config.LIFT_MOTOR_ID_R: {
					"goto_pos": self.position[1] * config.encoder_pulse_lift_r, # pulse mm * (pulse per mm)
					"goto_velo": self.velocity[1] * config.encoder_pulse_lift_r,
				},
				config.MIDDLE_MOTOR_ID: {
					"goto_pos": self.position[0] * config.encoder_pulse_middle, # pulse mm * (pulse per mm)
					"goto_velo": self.velocity[0] * config.encoder_pulse_middle,
				},
				config.TURRET_MOTOR_ID: {
					"goto_pos": self.position[3] * config.encoder_pulse_turret, # pulse mm * (pulse per deg)
					"goto_velo": self.velocity[3] * config.encoder_pulse_turret,
				},
				config.FORWARD_MOTOR_ID: {
					"goto_pos": self.position[4] * config.encoder_pulse_forward, # pulse mm * (pulse per mm)
					"goto_velo": self.velocity[4] * config.encoder_pulse_forward,
				},
			}

			result = requests.post(url, json=data).json()
			if (result.status_code != 200)
				print ('HTTP Error: {}'.format(result.status_code))

			# return result
		else:
			try:
				self.driver_base_l.set_goal_pos(self.position[2], self.velocity[2])
				self.driver_base_r.set_goal_pos(self.position[2], self.velocity[2])
				self.driver_lift_l.set_goal_pos(self.position[1], self.velocity[1])
				self.driver_lift_r.set_goal_pos(self.position[1], self.velocity[1])
				self.driver_middle.set_goal_pos(self.position[0], self.velocity[0])
				self.driver_turret.set_goal_pos(self.position[3], self.velocity[3])
				self.driver_forward.set_goal_pos(self.position[4], self.velocity[4])
			except Exception as e:
				print (e)
