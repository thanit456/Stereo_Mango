import driver
import config
import math

driver_base_l = driver.DriverMotor(config.url, config.BASE_MOTOR_ID_L)
driver_base_r = driver.DriverMotor(config.url, config.BASE_MOTOR_ID_R)
driver_lift_l = driver.DriverMotor(config.url, config.LIFT_MOTOR_ID_L)
driver_lift_r = driver.DriverMotor(config.url, config.LIFT_MOTOR_ID_R)
driver_middle = driver.DriverMotor(config.url, config.MIDDLE_MOTOR_ID)
driver_turret = driver.DriverMotor(config.url, config.TURRET_MOTOR_ID)
driver_forward = driver.DriverMotor(config.url, config.FORWARD_MOTOR_ID)
driver_end_effector = driver.DriverServo(config.url, config.END_EFFECTOR_MOTOR_ID)

driver_base_l.set_pulse_per_mm(config.encoder_pulse_base_l)
driver_base_r.set_pulse_per_mm(config.encoder_pulse_base_r)
driver_lift_l.set_pulse_per_mm(config.encoder_pulse_lift_l)
driver_lift_r.set_pulse_per_mm(config.encoder_pulse_lift_r)
driver_middle.set_pulse_per_mm(config.encoder_pulse_middle)
driver_turret.set_pulse_per_mm(config.encoder_pulse_turret)
driver_forward.set_pulse_per_mm(config.encoder_pulse_forward)

current_pos_x = 0
current_pos_y = 0
current_pos_z = 0
current_ro_arm = 0
current_pos_arm = 0

def move_x(pos_mm):
	global current_pos_x
	return move_to_x(current_pos_x + pos_mm)

def move_y(pos_mm):
	global current_pos_y
	return move_to_y(current_pos_y + pos_mm)

def move_z(pos_mm):
	global current_pos_z
	return move_to_z(current_pos_z + pos_mm)

def rotate_arm(pos_deg):
	global current_ro_arm
	return rotate_to_arm(current_ro_arm + pos_deg)

def move_arm(pos_mm):
	global current_pos_arm
	return move_to_arm(current_pos_arm + pos_mm)

def move_to_x(pos_mm):
	global driver_lift_l, driver_lift_r
	driver_lift_l.set_goal_pos(pos_mm)
	driver_lift_r.set_goal_pos(pos_mm)

	driver_lift_l.wait_util_stop()
	driver_lift_r.wait_util_stop()

	return True

def move_to_y(pos_mm):
	global driver_middle
	driver_middle.set_goal_pos(pos_mm)
	driver_middle.wait_util_stop()

	return True

def move_to_z(pos_mm):
	global driver_base_l, driver_base_r
	driver_base_l.set_goal_pos(pos_mm)
	driver_base_r.set_goal_pos(pos_mm)

	driver_base_l.wait_util_stop()
	driver_base_r.wait_util_stop()

	return True

def rotate_to_arm(pos_deg):
	global driver_turret
	driver_turret.set_goal_pos(pos_deg)

	return True

def move_to_arm(pos_mm):
	global driver_forward
	driver_forward.set_goal_pos(pos_mm)
	driver_forward.wait_util_stop()

	return True

def update_current():
	global current_pos_x, current_pos_y, current_pos_z, current_ro_arm, current_pos_arm
	current_pos_x = driver_middle.get_current()
	current_pos_y = driver_lift_l.get_current()
	current_pos_z = driver_base_l.get_current()
	current_ro_arm = driver_turret.get_current() * math.pi / 180
	current_pos_arm = driver_forward.get_current()

