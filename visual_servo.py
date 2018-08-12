import cv2
import config
import math
import driver
import datetime
from motion_control import Planner
from Tensorflow.object_detection import mod

class VisualServo:
	"""docstring for VisualServo"""
	def __init__(self, driver_camera):
		self.camera = driver_camera
		frame = self.__get_frame()		

		self.detector = mod.setup()

	def set_point_cloud(self, x, y, z):
		self.target_x = x
		self.target_y = y
		self.target_z = z

	def __get_frame(self):
		return self.camera.read()

	def __get_image_center(self):
		return (self.image_width/2, self.image_height/2) # x, y

	def __rorate_about_point(self, origin, point, deg): # dict of x, y, z | deg in radian
		x_diff = origin[0] - point[0]
		z_diff = origin[2] - point[2]
		return (origin[0] + (np.cos(deg) * x_diff - np.sin(deg) * z_diff),
				origin[1],
				origin[2] + (np.sin(deg) * x_diff + np.cos(deg) * z_diff))
		

	def loop(self):
		def get_distance(prev, cur):
			return math.sqrt((cur[0]-prev[0])**2, (cur[1]-prev[1])**2, (cur[2]-prev[2])**2)

		def get_dist_from_center(center, pixel, depth):
			return (center - pixel) * depth / config.camera_focus_length1

		planner = Planner.getInstance()

		cur_pos = planner.get_pos_arm()
		prev_time = cur_time = datetime.datetime.now()

		planner.move_single_cam(self.target_x - cur_pos[0], self.target_y - cur_pos[1], self.target_z - cur_pos[2], cur_pos[3], config.visual_speed)

		while self.__is_running:
			cur_time = datetime.datetime.now()

			if (cur_time - prev_time).milliseconds >= config.visual_time_delay:				
				depth = planner.get_depth()

				if depth <= 100:
					planner.cut_mango(True, False)
					time.sleep(5)
					planner.cut_mango(False, False)
					break
				
				# move servo
				frame = self.__get_frame()
				frame_height, frame_width = frame.shape[: 2]
				frame_center = (frame_height/2, frame_width/2)

				result = mod.detect(frame, self.detector) # return was wrong type
				mango_center = [result[0][1] + result[0][3] / 2, result[0][0] + result[0][2] / 2]

				# diff_x = mango_center[0] - frame_center[0]
				# diff_y = mango_center[1] - frame_center[1]

				diff_x = get_dist_from_center(frame_center[0], mango_center[0], depth) # mm
				diff_y = get_dist_from_center(frame_center[1], mango_center[1], depth) * -1 # mm

				diff_z = 0
				if np.sqrt(diff_x**2 + diff_y**2) <= config.visual_radius_accept:
					diff_z = 10

				planner.move_single_cam(diff_x, diff_y, diff_z, cur_pos[3], config.visual_speed)

				if np.sqrt(diff_x**2 + diff_y**2) <= config.visual_radius_accept:
					planner.move_arm_z(cur_pos[2] + config.visual_forward_length)

				prev_time = cur_time

	def start(self):
		self.__is_running = True
		self.run()

	def run(self):
		self.loop()
		self.__is_running = False

	def stop(self):
		self.__is_running = False