import cv2
import config
import math
import driver
import datetime
import motion_control as mc

class VS(object):
	"""docstring for VS"""
	def __init__(self, driver_camera):
		super(VS, self).__init__()
		self.camera = driver_camera

		frame = self.__get_frame()
		self.image_height, self.image_width = frame.shape[: 2]

		self.detector = None

	def __get_frame(self):
		return self.camera.read()

	def __get_mango_center(self):
		result = self.detector(self.__get_frame())

		if len(result > 1):
			raise Exception('Found mango more than one')

		return result[0]

	def __get_image_center(self):
		return (self.image_width/2, self.image_height/2) # x, y

	def loop(self):
		def get_distance(prev, cur):
			return math.sqrt((cur[0]-prev[0])**2, (cur[1]-prev[1])**2, (cur[2]-prev[2])**2)

		mc.update_current()

		prev_cur_pos_z = cur_pos_z = mc.current_pos_z + mc.current_pos_arm * math.sin(mc.current_ro_arm)
		prev_cur_pos_y = cur_pos_y = mc.current_pos_y
		prev_cur_pos_x = cur_pos_x = mc.current_pos_x
		prev_time = cur_time = datetime.datetime.now()

		while 1:
			cur_time = datetime.datetime.now()
			mc.update_current()
			cur_pos_z = mc.current_pos_z + mc.current_pos_arm * math.sin(mc.current_ro_arm)
			cur_pos_y = mc.current_pos_y
			cur_pos_x = mc.current_pos_x

			if (cur_time - prev_time).milliseconds >= config.time_delay:
				
				# move servo

				prev_time = cur_time