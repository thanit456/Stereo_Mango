import time
import hashlib
import json
import requests

import pickle
import numpy as np
import cv2
from imutils.video import VideoStream
import imutils

secret_key = b'Eic981234'

def generate_otp():
	timestamp = int(time.time()*1000)
	timestamp = str(timestamp)
	h = hashlib.sha256()
	h.update(timestamp.encode() + secret_key)
	return h.digest()[:8].hex()+timestamp


class DriverMotor(object):
	"""docstring for DriverMotor"""
	def __init__(self, host, id):
		super(DriverMotor, self).__init__()
		self.url_host = host
		self.id = id
		self.ppmm = 0

		self.set_mode(1)

	def set_pulse_per_mm(self, ppr):
		self.ppmm = ppr

	def set_max_movement(self, max_m):
		self.max_length = max_m

		if self.ppmm == 0:
			raise Exception('Set pulse per mm before call this function.')

		url = "{}/{}/eeprom/set".format(self.url_host, self.id)
		result = requests.post(url, data={"token": generate_otp(), "max_pos": max_m * self.ppmm})

		if (result.status_code != 200)
			raise Exception('HTTP Error: {}'.format(result.status_code))

		return json.loads(result.text)

	def set_min_movement(self, min_m):
		self.min_length = min_m

		if self.ppmm == 0:
			raise Exception('Set pulse per mm before call this function.')

		url = "{}/{}/eeprom/set".format(self.url_host, self.id)
		result = requests.post(url, data={"token": generate_otp(), "min_pos": -1 * min_m * self.ppmm})

		if (result.status_code != 200)
			raise Exception('HTTP Error: {}'.format(result.status_code))

		return json.loads(result.text)
	
	def set_mode(self, mode = 1):
		self.mode = mode
		try:
			return self.__post('eeprom/set', {"mode": self.mode})
		except Exception as e:
			raise e

	def get_goal(self):
		if self.ppmm == 0:
			raise Exception('Set pulse per mm before call this function.')

		try:
			result = self.__post('status/goal')
		except Exception as e:
			raise e

		result_json = json.loads(result)

		if self.mode == 1:
			return result_json['goal_pos'] / self.ppmm

		return result_json['goal_pwm']

	def get_current(self):
		if self.ppmm == 0:
			raise Exception('Set pulse per mm before call this function.')
		try:
			result = self.__post('status/motion')
		except Exception as e:
			raise e

		result_json = json.loads(result)

		if self.mode == 1:
			return result_json['cur_pos'] / self.ppmm

		return result_json['cur_pwm']

	def get_others(self):
		try:
			return self.__post('status/others')
		except Exception as e:
			raise e

	def set_goal_pwm(self, pwm)
		try:
			return self.__post('status/set', {"goal_pwm": pwm})
		except Exception as e:
			raise e

	def set_goal_pos(self, pos)
		try:
			return self.__post('status/set', {"goal_pos": pos * self.ppmm})
		except Exception as e:
			raise e

	def scan_working_length(self, pwm):

	def __post(self, action = 'status/motion', post = {}):
		url = "{}/{}/{}".format(self.url_host, self.id, action)
		
		data = {"token": generate_otp()}
		data.append(post)

		result = requests.post(url, data=data)
		if (result.status_code != 200)
			raise Exception('HTTP Error: {}'.format(result.status_code))

		return json.loads(result.text)

class DriverServo(object):
	"""docstring for DriverServo"""
	def __init__(self, host, id):
		super(DriverServo, self).__init__()
		self.url_host = host
		self.id = id
		self.ppdeg = 0
		
	def set_pulse_per_deg(self, ppdeg):
		self.ppdeg = ppdeg

class DriverLaser(object):
	"""docstring for DriverLaser"""
	def __init__(self, host, id):
		super(DriverLaser, self).__init__()
		self.url_host = host
		self.id = id

	def get_length(self):
		pass
		
class DriverCamera(object):
	"""docstring for DriverCamera"""
	def __init__(self, id):
		super(DriverCamera, self).__init__()
		self.id = id
		
		self.vs = VideoStream(src=id)
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
		self.camera = self.vs.start()

	def read(self):
		if self.camera == None:
			raise Exception('The camera doesn\'t start yet')

		frame = self.camera.read()
		if self.img_points:
			return cv2.undistort(frame, self.camera_mtx, self.camera_distortion, None, self.camera_new_mtx)

		return frame

	def stop(self):
		self.camera.stop()
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
