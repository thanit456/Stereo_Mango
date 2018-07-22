import driver
import config
import cv2
from mango_database import Mango, MangoStorage
from multiprocessing.dummy import Pool as ThreadPool

driver_base_l = driver.DriverMotor(config.url, config.BASE_MOTOR_ID_L)
driver_base_r = driver.DriverMotor(config.url, config.BASE_MOTOR_ID_R)
driver_lift_l = driver.DriverMotor(config.url, config.LIFT_MOTOR_ID_L)
driver_lift_r = driver.DriverMotor(config.url, config.LIFT_MOTOR_ID_R)
driver_middle = driver.DriverMotor(config.url, config.MIDDLE_MOTOR_ID)
driver_turret = driver.DriverMotor(config.url, config.TURRET_MOTOR_ID)
driver_forward = driver.DriverMotor(config.url, config.FORWARD_MOTOR_ID)
driver_end_effector = driver.DriverServo(config.url, config.END_EFFECTOR_MOTOR_ID)
driver_ridar = driver.DriverLaser(config.url, config.LASER_ID)

mango_detector = None

driver_base_l.set_pulse_per_mm(config.encoder_pulse_base_l)
driver_base_r.set_pulse_per_mm(config.encoder_pulse_base_r)
driver_lift_l.set_pulse_per_mm(config.encoder_pulse_lift_l)
driver_lift_r.set_pulse_per_mm(config.encoder_pulse_lift_r)
driver_middle.set_pulse_per_mm(config.encoder_pulse_middle)
driver_turret.set_pulse_per_mm(config.encoder_pulse_turret)
driver_forward.set_pulse_per_mm(config.encoder_pulse_forward)
# driver_end_effector


def fn_scan_mango(camera_l, canera_r):
	global mango_detector

	frame_l = camera_l.read()
	img_height, img_width = frame_l.shape[:2]

	obj_height = img_height * config.scan_dist_interesting / config.camera_focus_length1
	obj_width = img_width * config.scan_dist_interesting / config.camera_focus_length1 - config.camera_D0

	scan_x = obj_height
	scan_y = obj_width

	scan_i = int(workspace_y / scan_y) + 1
	scan_j = int(workspace_x / scan_x) + 1

	list_mango = MangoStorage()

	for i in range(1, scan_i):
		driver_lift_l.set_goal_pos(int(scan_y/2) + scan_y * i)
		driver_lift_r.set_goal_pos(int(scan_y/2) + scan_y * i)

		for j in range(1, scan_j):
			driver_middle.set_goal_pos(int(scan_x/2) + scan_x * j)

			frame_l = camera_l.read()
			frame_r = camera_r.read()
			img_height_l, img_width_l = frame_l.shape[:2]
			img_height_r, img_width_r = frame_r.shape[:2]

			pool = ThreadPool(2)
			list_mango_frame_l, list_mango_frame_r = poll.map(mango_detector.detect, [frame_l, frame_r])
			# close the pool and wait for the work to finish 
			pool.close() 
			pool.join()

			curr_pos_x = int((driver_lift_r.get_current() - driver_lift_l.get_current()) * driver_middle.get_current() / workspace_x)
			curr_pos_y = int((driver_lift_r.get_current() + driver_lift_l.get_current()) / 2.0)
			curr_pos_z = int((driver_base_l.get_current() + driver_base_r.get_current()) / 2.0)

			mango_len = min(len(list_mango_frame_l), len(list_mango_frame_r))

			# หาวิธีคิดใหม่ พิกัด x, y, z บนโลกจริง และเก็บใน list โดยอ้างอิงจาก โครงสร้าง ของหุ่น
			# เพิ่มวิธีเช็คด้วยว่า ใน list นั้น ใช้มะม่วงผลเดียวกัน
			for k in range(mango_len):
				d = config.camera_focus_length * config.camera_D0 / (list_mango_frame_l[k, 1] - list_mango_frame_r[k, 1]) # mm

				x1 = (list_mango_frame_l[k, 0] - (img_height_l / 2)) * d / config.camera_focus_length1 + camera_l.offset_x # mm
				x2 = (list_mango_frame_r[k, 0] - (img_height_r / 2)) * d / config.camera_focus_length2 + camera_r.offset_x # mm

				y1 = (list_mango_frame_l[k, 1] - (img_width_l / 2)) * d / config.camera_focus_length1 + camera_l.offset_y # mm
				y2 = (list_mango_frame_r[k, 1] - (img_width_r / 2)) * d / config.camera_focus_length2 + camera_r.offset_y # mm

				m = Mango(curr_pos_x + x, curr_pos_y + y, curr_pos_z + d) # mm

				if d <= config.scan_dist_interesting:
					list_mango.add_mango(m)

	return list_mango

def fn_scan_object():
	tree_radius = -1

	list_ridar_length = []

	scan_x = 100
	scan_y = 300

	scan_i = int(workspace_y / scan_y)
	scan_j = int(workspace_x / scan_x)

	for i in range(scan_i):
		driver_lift_l.set_goal_pos(int(scan_y/2) + scan_y * i)
		driver_lift_r.set_goal_pos(int(scan_y/2) + scan_y * i)

		ridar_length = []
		for j in range(scan_j):
			driver_middle.set_goal_pos(int(scan_x/2) + scan_x * j)

			ridar_length.append(driver_ridar.get_length())

	return tree_radius

def fb_carlibrate_stereo_camera(camera_l, camera_r):
	# make the Pool of workers
	pool = ThreadPool(2)

	poll.apply_async(camera_l.calibrate(), ())
	poll.apply_async(camera_r.calibrate(), ())

	# close the pool and wait for the work to finish 
	pool.close() 
	pool.join()
