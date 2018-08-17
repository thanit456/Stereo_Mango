import time
import hashlib
import requests
import datetime

import pickle
import numpy as np
import cv2
#from imutils.video import VideoStream
import imutils

secret_key = b'Eic981234'

def generate_otp():
	timestamp = int(time.time()*1000)
	timestamp = str(timestamp)
	h = hashlib.sha256()
	h.update(timestamp.encode() + secret_key)
	return h.digest()[:8].hex()+timestamp

# class DriverMotor:
# 	"""docstring for DriverMotor"""
# 	def __init__(self, host, id):
# 		self.url_host = host
# 		self.id = str(id)
# 		self.ppmm = 0

# 		self.moving_threshold = 1
# 		self.delay_time = 4 + 1.5 + 3.5 #ms

# 		self.set_mode(1)

# 		self.working_length = 1000 # mm

# 	def set_pulse_per_mm(self, ppr):
# 		self.ppmm = ppr
# 		return True

# 	def set_max_movement(self, max_m):
# 		self.max_length = max_m

# 		if self.ppmm == 0:
# 			raise Exception('Set pulse per mm before call this function.')

# 		try:
# 			return self._post('set', {"max_pos": self.max_length * self.ppmm})
# 		except Exception as e:
# 			raise e

# 	def set_min_movement(self, min_m):
# 		self.min_length = min_m

# 		if self.ppmm == 0:
# 			raise Exception('Set pulse per mm before call this function.')

# 		try:
# 			return self._post('set', {"min_pos": self.min_length * self.ppmm})
# 		except Exception as e:
# 			raise e
	
# 	def set_mode(self, mode = 1):
# 		self.mode = mode
# 		try:
# 			return self._post('set', {"mode": self.mode})
# 		except Exception as e:
# 			raise e

# 	def get_goal(self):
# 		if self.ppmm == 0:
# 			raise Exception('Set pulse per mm before call this function.')


# 		if self.mode == 1:			
# 			try:
# 				result = self._post('get', ['goal_pos'])
# 			except Exception as e:
# 				raise e
# 			return result['goal_pos'] / self.ppmm
# 		else:
# 			try:
# 				result = self._post('get', ['goal_pwm'])
# 			except Exception as e:
# 				raise e
# 			return result['goal_pwm']

# 	def get_current(self):
# 		if self.ppmm == 0:
# 			raise Exception('Set pulse per mm before call this function.')
		
# 		if self.mode == 1:
# 			try:
# 				result = self._post('get', ['cur_pos', 'cur_velo'])
# 			except Exception as e:
# 				raise e
# 			return [result['cur_pos'] / self.ppmm, result['cur_velo']]
# 		else:
# 			try:
# 				result = self._post('get', ['cur_pwm', 'cur_velo'])
# 			except Exception as e:
# 				raise e
# 			return [result['cur_pwm'], result['cur_velo']]

# 	def get_gpio(self):
# 		try:
# 			result = self._post('get', ['gpio1', 'gpio2', 'gpio3', 'gpio4', 'gpio_limit_min', 'gpio_limit_max'])
# 		except Exception as e:
# 			raise e

# 		# min, max, 1, 2, 3, 4
# 		return [result['gpio_limit_min'], result['gpio_limit_max'], result['gpio1'], result['gpio2'], result['gpio3'], result['gpio4']]

# 	def get_hw_error(self):
# 		try:
# 			result = self._post('get', ['hwerr'])
# 		except Exception as e:
# 			raise e

# 		return result['hwerr']

# 	def get_adc(self, ch):
# 		ch = 'adc{}'.format(ch)
# 		try:
# 			result = self._post('get', [ch])
# 		except Exception as e:
# 			raise e

# 		return result[ch]

# 	def set_goal_pwm(self, pwm):
# 		try:
# 			return self._post('set', {"goal_pwm": pwm})
# 		except Exception as e:
# 			raise e

# 	def enable(self, en = 1):
# 		try:
# 			return self._post('set', {'enable': 1})
# 		except Exception as e:
# 			raise e

# 	def set_goal_pos(self, pos, vel = 100):
# 		try:
# 			return self._post('set', {"goto_pos": pos * self.ppmm, "goto_velo": vel})
# 		except Exception as e:
# 			raise e

# 	def set_moving_threshould(self, th):
# 		self.moving_threshold = th

# 	def scan_working_length(self, pwm):
# 		# set mode to pwm
# 		self.set_mode(0)
# 		# move element to min position
# 		self.set_goal_pwm(-1 * pwm)
# 		# wait element hit min switch
# 		gpio_min = self.get_gpio()[0]
# 		time = datetime.datetime.now()
# 		while not gpio_min:
# 			if (datetime.datetime.now() - time).total_milliseconds > self.delay_time:
# 				gpio_min = self.get_gpio()[0]
# 				time = datetime.datetime.now()
# 		# stop movement
# 		self.set_goal_pwm(0)
# 		# reset position on controller
# 		self.reset_position()
# 		# set min movement
# 		self.set_min_movement(0)
# 		# move element to max position
# 		self.set_goal_pwm(pwm)
# 		# wait element hit max switch
# 		gpio_max = self.get_gpio()[1]
# 		time = datetime.datetime.now()
# 		while not gpio_max:
# 			if (datetime.datetime.now() - time).total_milliseconds > self.delay_time:
# 				gpio_max = self.get_gpio()[1]
# 				time = datetime.datetime.now()
# 		# stop movement
# 		self.set_goal_pwm(0)
# 		# change mode to position
# 		self.set_mode(1)		
# 		# get working length
# 		self.working_length = self.get_current()
# 		# set max movement		
# 		self.set_max_movement(self.working_length)

# 		print ("Working length: {}".format(self.working_length))

# 	def wait_util_stop(self):
# 		tmp_time = datetime.datetime.now()

# 		while 1:
# 			if (datetime.datetime.now - tmp_time).milliseonds <= self.delay_time:
# 				continue

# 			result = self.__post('get', ['cur_velo'])

# 			cur_velo = result['cur_velo']
# 			if cur_velo < self.moving_threshold:
# 				break

# 			print ("Waiting motor {} ...".format(self.id))

# 	def reset_position(self):
# 		try:
# 			return self._post('cmd', ['pos_reset'])
# 		except Exception as e:
# 			raise e

# 	def _post(self, action, post):
# 		url = "{}/{}".format(self.url_host, action)
		
# 		data = {"token": generate_otp()}
# 		data.update({self.id: post})

# 		result = requests.post(url, json=data)
# 		if (result.status_code != 200):
# 			raise Exception('HTTP Error: {}'.format(result.status_code))
# 		# print (result.json())
# 		if action == "get":
# 			return result.json()[self.id]

# 		return result.json()['success']

# class DriverServo:
# 	"""docstring for DriverServo"""
# 	def __init__(self, host, id):
# 		self.url_host = host
# 		self.id = id
# 		self.ppdeg = 0

# 	def set_pulse_per_deg(self, ppdeg):
# 		self.ppdeg = ppdeg

class DriverCamera:
	"""docstring for DriverCamera"""
	def __init__(self, id):
		self.id = id

		#self.vs = VideoStream(src=id)
		self.vs = cv2.VideoCapture(id)
		self.camera = None

		self.obj_points = []
		self.img_points = []
		self.camera_mtx = []
		self.camera_distortion = []
		self.camera_new_mtx = []

		self.offset_x = 0
		self.offset_y = 0

		self.__load()

	def start(self):
		#self.camera = self.vs.start()
		self.camera = self.vs

	def read(self):
		if self.camera == None:
			raise Exception('The camera doesn\'t start yet')

		frame = self.camera.read()
		if self.img_points:
			return cv2.undistort(frame, self.camera_mtx, self.camera_distortion, None, self.camera_new_mtx)

		return frame

	def stop(self):
		#self.camera.stop()
		self.camera = None

	def calibrate(self, col = 6, row = 7, size = 25, count = 15, roi = 1):
		self.start()

		print ("Start calibration on camera {}.".format(self.id))

		# termination criteria
		criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, size, 0.001)

		# prepare object points, like (0,0,0), (1,0,0), (2,0,0) ....,(6,5,0)
		objp = np.zeros((col * row, 3), np.float32)
		objp[:, :2] = np.mgrid[0:row, 0:col].T.reshape(-1,2)

		# Arrays to store object points and image points from all the images.
		self.obj_points = [] # 3d point in real world space
		self.img_points = [] # 2d points in image plane.

		img_height = 480
		img_width = 640

		calibrate_count = count
		while calibrate_count > 0:
			img = self.camera.read()
			img_height, img_width = img.shape[:2]

			gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

			# Find the chess board corners
			ret, corners = cv2.findChessboardCorners(gray, (row, col), None)

			# If found, add object points, image points (after refining them)
			if ret == True:
				self.obj_points.append(objp)

				cv2.cornerSubPix(gray, corners, (11, 11), (-1, -1), criteria)
				self.img_points.append(corners)

				# Draw and display the corners
				cv2.drawChessboardCorners(img, (row, col), corners, ret)
				# print (corners)

				cv2.imshow('Images that use calibrate', img)
				cv2.waitKey(500)
				calibrate_count -= 1

				print ("Image {}".format(count - calibrate_count))

		cv2.destroyAllWindows()
		self.stop()

		ret, self.camera_mtx, self.camera_distortion, rvecs, tvecs = cv2.calibrateCamera(self.obj_points, self.img_points, gray.shape[::-1], None, None)
		self.camera_new_mtx, roi = cv2.getOptimalNewCameraMatrix(self.camera_mtx, self.camera_distortion, (img_width, img_height), roi, (img_width, img_height))

		print ("Calibration success.")

		self.__save()

	def set_offset(self, x, y):
		self.offset_x = x
		self.offset_y = y

	def __load(self):
		try:
			with open('camera_calibrate_{}.pkl'.format(self.id), 'rb') as input:
				self.obj_points = pickle.load(input)
				self.img_points = pickle.load(input)
				self.camera_mtx = pickle.load(input)
				self.camera_distortion = pickle.load(input)
				self.camera_new_mtx = pickle.load(input)
		except Exception as e:
			print (e)

	def __save(self):
		with open('camera_calibrate_{}.pkl'.format(self.id), 'wb') as output:
			pickle.dump(self.obj_points, output, pickle.HIGHEST_PROTOCOL)
			pickle.dump(self.img_points, output, pickle.HIGHEST_PROTOCOL)
			pickle.dump(self.camera_mtx, output, pickle.HIGHEST_PROTOCOL)
			pickle.dump(self.camera_distortion, output, pickle.HIGHEST_PROTOCOL)
			pickle.dump(self.camera_new_mtx, output, pickle.HIGHEST_PROTOCOL)
