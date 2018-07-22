import cv2
import config
import math
import driver

class VS(object):
	"""docstring for VS"""
	def __init__(self, driver_camera):
		super(VS, self).__init__()
		self.camera = driver_camera

		self.driver_base_l = DriverMotor(config.url, config.BASE_MOTOR_ID_L)
		self.driver_lift_l = DriverMotor(config.url, config.LIFT_MOTOR_ID_L)
		self.driver_middle = DriverMotor(config.url, config.MIDDLE_MOTOR_ID)
		self.driver_forward = DriverMotor(config.url, config.FORWARD_MOTOR_ID)

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

		prev_cur_pos_z = cur_pos_z = self.driver_base_l.get_current()
		prev_cur_pos_y = cur_pos_y = self.driver_lift_l.get_current()
		prev_cur_pos_x = cur_pos_x = self.driver_middle.get_current()
		prev_time = cur_time = datetime.datetime.now()

		while 1:
			cur_time = datetime.datetime.now()
			cur_pos_z = self.driver_base_l.get_current()
			cur_pos_y = self.driver_lift_l.get_current()
			cur_pos_x = self.driver_middle.get_current()

			if (cur_time - prev_time).milliseconds >= config.time_delay:
				
				# move servo

				prev_time = cur_time