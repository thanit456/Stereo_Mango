import driver
import config
import cv2
import datetime
from visual_servoing import VS_Ojudge
from motion_control import Planner
from multiprocessing.dummy import Pool as ThreadPool
from threading import Thread
import util

class Harvest(Thread):
	"""docstring for Harvest"""
	def __init__(self):
		Thread.__init__()
		self.__running = True
		
		self.cam_1 = driver.DriverCamera(config.CAMERA_STEREO_L_ID)
		self.cam_2 = driver.DriverCamera(config.CAMERA_STEREO_R_ID)
		self.cam_3 = driver.DriverCamera(config.CAMERA_END_EFFECTOR_ID)

		self.harvest_timeout = 1000 * 60 * 15 # ms

		self.mc = Planner.getInstance()

	def run(self):
		self.harvest_start_time = datetime.datetime.now()
		while self.__running and self.check_stop_keep_condition():
			pass

		self.__running = False

	def check_stop_keep_condition(self):
		time_now = datetime.datetime.now()
		if (time_now - self.harvest_start_time).total_milliseconds > self.harvest_timeout:
			return False

		return True

	def stop(self):
		self.__running = False

	def fn_scan_mango(self):
		frame_l = self.cam_1.read()
		img_height, img_width = frame_l.shape[:2]

		obj_height = img_height * config.scan_dist_interesting / config.camera_focus_length1
		obj_width = img_width * config.scan_dist_interesting / config.camera_focus_length1 - config.camera_D0

		scan_x = obj_height
		scan_y = obj_width

		scan_i = int(workspace_y / scan_y) + 1
		scan_j = int(workspace_x / scan_x) + 1
		scan_k = int(workspace_z / config.scan_dist_interesting)

		for k in range(0, scan_k):
			self.mc.move_z(scan_k * k)

			for i in range(1, scan_i):
				self.mc.move_y(int(scan_y/2) + scan_y * i)

				for j in range(1, scan_j):
					self.mc.move_x(int(scan_x/2) + scan_x * j)

					frame_l = self.cam_1.read()
					frame_r = self.cam_2.read()
					img_height_l, img_width_l = frame_l.shape[:2]
					img_height_r, img_width_r = frame_r.shape[:2]

					pool = ThreadPool(2)
					list_mango_frame_l, list_mango_frame_r = poll.map(mango_detector.detect, [frame_l, frame_r])
					# close the pool and wait for the work to finish 
					pool.close() 
					pool.join()

					# get current position in mm unit (revo is center)
					curr_pos_x, curr_pos_y, curr_pos_z = mc.get_pos()

					mango_len = min(len(list_mango_frame_l), len(list_mango_frame_r))

					# หาวิธีคิดใหม่ พิกัด x, y, z บนโลกจริง และเก็บใน list โดยอ้างอิงจาก โครงสร้าง ของหุ่น
					# เพิ่มวิธีเช็คด้วยว่า ใน list นั้น ใช้มะม่วงผลเดียวกัน
					for k in range(mango_len):
						d = config.camera_focus_length * config.camera_D0 / (list_mango_frame_l[k, 1] - list_mango_frame_r[k, 1]) # mm

						x1 = (list_mango_frame_l[k, 0] - (img_height_l / 2)) * d / config.camera_focus_length1 + camera_l.offset_x # mm
						x2 = (list_mango_frame_r[k, 0] - (img_height_r / 2)) * d / config.camera_focus_length2 + camera_r.offset_x # mm

						y1 = (list_mango_frame_l[k, 1] - (img_width_l / 2)) * d / config.camera_focus_length1 + camera_l.offset_y # mm
						y2 = (list_mango_frame_r[k, 1] - (img_width_r / 2)) * d / config.camera_focus_length2 + camera_r.offset_y # mm

						x = (x1 + x2) / 2
						y = (y1 + y2) / 2

						if d <= config.scan_dist_interesting:
							vs = VS_Ojudge(self.cam_3)
							vs.set_point_cloud(curr_pos_x + x, curr_pos_y + y, curr_pos_z + d)
							vs.start()
						

		
# Planner.getInstance().loop()

if __name__ == '__main__':
	mango_algor = Harvest()
	planner = Planner.getInstance()
	planner.start()

	while 1:
		cmd = raw_input().strip()

		if cmd == "start":
			print ("Starting harvest mango algorithm")
			mango.start()
			print ("Started")
		else cmd == "stop"
			print ("Stopping harvest mango algorithm")
			mango.stop()
			print ("Stopped")
			break

		else cmd == "calibrate":
			if mango_algor.is_alive():
				print ("Stop harvest mango algorithm before calibrate camera, Please")
				continue

			util.fn_carlibrate_stereo_camera()

		else:
			print ("start - start keep mango")
			print ("stop - stop keep mango")
			# print ("home - go home")
			print ("calibrate - calibrate camera")
			print ("Ctrl+C - exit")

	planner.stop()