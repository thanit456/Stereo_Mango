import numpy as np
import cv2
from imutils.video import VideoStream
import imutils
# import glob
import pickle

class Camera(object):
	"""docstring for Camera"""
	def __init__(self, id):
		super(Camera, self).__init__()
		self.id = id
		
		self.vs = VideoStream(src=id)
		self.camera = None

		self.obj_points = []
		self.img_points = []
		self.camera_mtx = []
		self.camera_distortion = []
		self.camera_new_mtx = []

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


if __name__ == '__main__':
	cam = Camera(0)

	# cam.calibrate(count = 30)

	cam.start()
	while 1:
		frame = cam.read()
		cv2.imshow("Realtime", frame)
		cv2.waitKey(1)