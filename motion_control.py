import driver
import config
import time, hashlib, datetime, requests
import numpy as np
import threading

servo_template =  {
	"en": 1,
	'min_pos': 900,
	'max_pos': 2000,
	'cur_pos': 1500,
}

motor_template = {
	'en' : 0,
	'cpr' : 8000,
	'cur_pos' : 0,
	'cur_velo': 0,
	'min_pos' : 0,
	'max_pos' : 0,
	'goto_pos' : 0,
	'goto_velo': 0,
}

secret_key = b'Eic981234'

def generate_otp():
	timestamp = int(time.time()*1000)
	timestamp = str(timestamp)
	h = hashlib.sha256()
	h.update(timestamp.encode() + secret_key)
	return h.digest()[:8].hex()+timestamp

class Planner:
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
			Planner.__instance = self

		self.lock = threading.Lock()

		self.position = [0 for i in range(7)] # middle, lift, base, turret, forward
		self.velocity = [0 for i in range(7)] # middle, lift, base, turret, forward
		self.depth = 0

		self.motor = {
			config.BASE_MOTOR_ID_L : dict(motor_template),
			config.BASE_MOTOR_ID_R : dict(motor_template),
			config.LIFT_MOTOR_ID_L : dict(motor_template),
			config.LIFT_MOTOR_ID_R : dict(motor_template),
			config.MIDDLE_MOTOR_ID : dict(motor_template),
			config.TURRET_MOTOR_ID : dict(motor_template),
			config.FORWARD_MOTOR_ID : dict(motor_template),
		}

		self.motor[config.BASE_MOTOR_ID_L]['cpr'] = config.encoder_pulse_base_l
		self.motor[config.BASE_MOTOR_ID_R]['cpr'] = config.encoder_pulse_base_r
		self.motor[config.LIFT_MOTOR_ID_L]['cpr'] = config.encoder_pulse_lift_l
		self.motor[config.LIFT_MOTOR_ID_R]['cpr'] = config.encoder_pulse_lift_r
		self.motor[config.MIDDLE_MOTOR_ID]['cpr'] = config.encoder_pulse_middle
		self.motor[config.TURRET_MOTOR_ID]['cpr'] = config.encoder_pulse_turret
		self.motor[config.FORWARD_MOTOR_ID]['cpr'] = config.encoder_pulse_forward

		self.servo = {
			config.SERVO_DOOR1 : dict(servo_template),
			config.SERVO_DOOR2 : dict(servo_template),
			config.SERVO_CUTTER : dict(servo_template),
		}

		self.delay_time = datetime.datetime.now()

		self.__is_running = True
			
	#def __getattr__(self, name):
	#	return getattr(self.instance, name)

	def move_stereo_cam(self, x_pos, y_pos, z_pos, speed, is_set = False):
		self._update()

		return self.move_to_stereo_cam(self.position[0] + x_pos, self.position[1] + y_pos, self.position[2] + z_pos, speed, is_set)

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

		x_velo = 0 if overall_time == 0 else  x_diff / overall_time # mm / seconds
		y_velo = 0 if overall_time == 0 else  y_diff / overall_time
		z_velo = 0 if overall_time == 0 else  z_diff / overall_time

		position = [self.position[0] + x_pos, self.position[1] + y_pos, self.position[2] + z_pos]
		velocity = [x_velo, y_velo, z_velo]

		# set output
		for i in range(0, 3):
			for motor_id in config.MOTOR_GROUP[i]:
				self.motor[motor_id]['en'] = 1
				self.motor[motor_id]['goal_pos'] = position[i]
				self.motor[motor_id]['goal_vel'] = velocity[i] * 1000

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

		line_length = np.sqrt(x_pos**2 + y_pos**2 + z_pos**2)
		overall_time = line_length / speed
		z_velo = 0 if overall_time == 0 else z_diff / overall_time

		self.move_stereo_cam(z_remain_x + x_pos_x, y_pos, z_remain_z + x_pos_z, speed, True)
		self.turn_arm(deg, speed, True)
		for motor_id in config.MOTOR_GROUP[4]:
			self.motor[motor_id]['en'] = 1
			self.motor[motor_id]['goal_pos'] = self.position[4] + z_pos
			self.motor[motor_id]['goal_vel'] = z_velo * 1000

		self._send_position()

	def turn_arm(self, deg, speed, is_set = False):
		self.turn_to_arm(self.position[3] * Planner.radToDeg + deg, speed)

	def turn_to_arm(self, deg, speed, is_set = False):
		rad = (deg  % 360) * Planner.degToRad
		rad_diff = np.abs(self.position[3] - rad)
		linear = rad_diff * self.position[4]
		overall_time = linear / speed

		velo = 0 if overall_time == 0 else rad_diff * Planner.radToDeg / overall_time

		for motor_id in config.MOTOR_GROUP[3]:
			self.motor[motor_id]['en'] = 1
			self.motor[motor_id]['goal_pos'] = (self.position[3] + rad) % 360 * Planner.radToDeg
			self.motor[motor_id]['goal_vel'] = velo * 1000

		if is_set is False:
			self._send_position()

	def cut_mango(self, is_cut, is_set = False):
		cutter = self.servo[config.SERVO_CUTTER]
		
		cutter['en'] = 1
		cutter['cur_pos'] = cutter['max_pos'] if is_cut is True else cutter['min_pos']				

		if is_set is False:
			self._send_position()

	def drop_mango(self, is_drop, is_set = False):
		door1 = self.servo[config.SERVO_DOOR_1]
		door2 = self.servo[config.SERVO_DOOR_2]

		door1['en'] = 1
		door2['en'] = 1
		door1['cur_pos'] = door1['max_pos'] if is_drop is True else door1['min_pos']
		door2['cur_pos'] = door2['max_pos'] if is_drop is True else door2['min_pos']

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

	def get_depth(self):
		self._update()
		return self.depth

	def _update(self):
		time_diff = (datetime.datetime.now() - self.delay_time).microseconds
		if time_diff < config.planner_update_time:
			return

		result = {}

		url = "{}/get".format(config.url)
		data = {"token": generate_otp()}

		for motor_id in self.motor.keys():
			data[str(motor_id)] = ["cur_pos", "cur_velo"]

		servo = []
		for servo_id in self.servo.keys():
			servo.append('ch{}_pos'.format(servo_id))
		servo.append('range')
		data[str(config.END_EFFECTOR_ID)] = servo
		
		tmp = None
		try:
			tmp = requests.post(url, json=data).json()
		except Exception as e:
			print (e)
			return 
		
		for motor_id in self.motor.keys():
			self.motor[motor_id]['cur_pos'] = tmp[str(motor_id)]['cur_pos']		# pulse
			self.motor[motor_id]['cur_velo'] = tmp[str(motor_id)]['cur_velo']	# pulse / ms
		for servo_id in self.servo.keys():
			self.servo[servo_id]['cur_pos'] = tmp[str(config.END_EFFECTOR_ID)]['ch{}_pos'.format(servo_id)]
		self.depth = tmp[str(config.END_EFFECTOR_ID)]['range']

		for i in self.MOTOR_GROUP:
			self.position[i] = self.motor[self.MOTOR_GROUP[i][0]]['cur_pos'] / self.motor[self.MOTOR_GROUP[i][0]]['cpr']  # mm, (pulse / ms)
			self.velocity[i] = self.motor[self.MOTOR_GROUP[i][0]]['cur_velo']

			if i == 3:
				self.position[i] *= Planner.degToRad
		
		self.position[5] = (config.arm_min_workspace + self.position[4]) * np.cos(self.position[3]) # arm x
		self.position[6] = (config.arm_min_workspace + self.position[4]) * np.sin(self.position[3]) # arm z

		self.delay_time = datetime.datetime.now()

	def _send_position(self):
		url = "{}/set".format(config.url)

		data = {}

		motor = {}
		for motor_id in self.motor.keys():
			motor[motor_id] = {
				'enable' : self.motor[motor_id]['en'],
				'goto_pos' : self.motor[motor_id]['goto_pos'] * self.motor[motor_id]['cpr'],
				'goto_velo' : self.motor[motor_id]['goto_velo'] * self.motor[motor_id]['cpr'],
			}
		data.update(motor)

		servo = {}
		for servo_id in self.servo.keys():
			servo.update({
				'ch{}_enable'.format(servo_id) : self.servo[servo_id]['en'],
				'ch{}_pos'.format(servo_id) : self.servo[servo_id]['cur_pos'],
			})
		data[config.END_EFFECTOR_ID] = servo
	
		
		result = None
		try:
			result = requests.post(url, json=data).json()
		except Exception as e:
			print (e)
			return 
		if (result.status_code != 200):
			print ('HTTP Error: {}'.format(result.status_code))

		# return result

