import driver
import config
import cv2

driver_base_l = DriverMotor(config.url, config.BASE_MOTOR_ID_L)
driver_base_r = DriverMotor(config.url, config.BASE_MOTOR_ID_R)
driver_lift_l = DriverMotor(config.url, config.LIFT_MOTOR_ID_L)
driver_lift_r = DriverMotor(config.url, config.LIFT_MOTOR_ID_R)
driver_middle = DriverMotor(config.url, config.MIDDLE_MOTOR_ID)
driver_turret = DriverMotor(config.url, config.TURRET_MOTOR_ID)
driver_forward = DriverMotor(config.url, config.FORWARD_MOTOR_ID)
driver_end_effector = DriverMotor(config.url, config.END_EFFECTOR_MOTOR_ID, True)
driver_ridar = DriverLaser(config.url, config.LASER_ID)

mango_detector = None

# cv2.VideoCapture(0)

def fn_scan_mango(camera_l, canera_r):
	global mango_detector

	scan_x = 0
	scan_y = 0

	scan_i = int(workspace_y / scan_y)
	scan_j = int(workspace_x / scan_x)

	list_mango = []

	for i in range(scan_i):
		driver_lift_l.set_goal_pos(int(scan_y/2) + scan_y * i)
		driver_lift_r.set_goal_pos(int(scan_y/2) + scan_y * i)

		for j in range(scan_j):
			driver_middle.set_goal_pos(int(scan_x/2) + scan_x * j)

			frame_l = camera_l.read()
			frame_r = camera_r.read()

			list_mango_frame_l = mango_detector.detect(frame_l)
			list_mango_frame_r = mango_detector.detect(frame_r)

			curr_pos_x = int((driver_lift_r.get_current() - driver_lift_l.get_current()) * driver_middle.get_current() / workspace_x)
			curr_pos_y = int((driver_lift_r.get_current() - driver_lift_l.get_current()) / 2.0)

			mango_len = min(len(list_mango_frame_l), len(list_mango_frame_r))

			# หาวิธีคิดใหม่ พิกัด x, y, z บนโลกจริง และเก็บใน list โดยอ้างอิงจาก โครงสร้าง ของหุ่น
			# เพิ่มวิธีเช็คด้วยว่า ใน list นั้น ใช้มะม่วงผลเดียวกัน
			for k in range(mango_len):
				x = (list_mango_frame_l[k, 0] - list_mango_frame_r[k, 0])
				y = (list_mango_frame_l[k, 1] + list_mango_frame_r[k, 1]) / 2
				d = config.camera_focus_length * config.camera_D0 / x

				if d <= config.scan_dist_interesting:
					list_mango.append(x, y, d)

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

