import cv2
import config
import math
import driver
import datetime
from motion_control import Planner

class VS(object):
	"""docstring for VS"""
	def __init__(self, driver_camera):
		super(VS, self).__init__()
		self.camera = driver_camera

		frame = self.__get_frame()
		

		self.detector = None

	def __get_frame(self):
		return self.camera.read()

	def __get_image_center(self):
		return (self.image_width/2, self.image_height/2) # x, y

	def loop(self):
		def get_distance(prev, cur):
			return math.sqrt((cur[0]-prev[0])**2, (cur[1]-prev[1])**2, (cur[2]-prev[2])**2)

		planner = Planner.getInstance()

		cur_pos = planner.get_pos_arm()
		prev_time = cur_time = datetime.datetime.now()

		while 1:
			cur_time = datetime.datetime.now()
			cur_pos = planner.get_pos()

			if (cur_time - prev_time).milliseconds >= config.time_delay:
				
				# move servo
				frame = self.__get_frame()
				frame_height, frame_width = frame.shape[: 2]
				frame_center = [frame_height/2, frame_width/2]

				result = self.detector(frame)
				mango_center = [result[0][1] + result[0][3] / 2, result[0][0] + result[0][2] / 2]

				diff_x = mango_center[0] - frame_center[0]
				diff_y = mango_center[1] - frame_center[1]



				prev_time = cur_time