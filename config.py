# Section ID
BASE_MOTOR_ID_L 	= 0x01
BASE_MOTOR_ID_R 	= 0x02
LIFT_MOTOR_ID_L 	= 0x03
LIFT_MOTOR_ID_R 	= 0x04
MIDDLE_MOTOR_ID		= 0x05
TURRET_MOTOR_ID		= 0x06
FORWARD_MOTOR_ID	= 0x07

MOTOR_GROUP = [
	(MIDDLE_MOTOR_ID, ),					# x
	(LIFT_MOTOR_ID_L, LIFT_MOTOR_ID_R),		# y
	(BASE_MOTOR_ID_L, BASE_MOTOR_ID_R),		# z
	(TURRET_MOTOR_ID, ),					# a
	(FORWARD_MOTOR_ID, ),					# b
]

CAMERA_STEREO_L_ID	= 0
CAMERA_STEREO_R_ID 	= 1
CAMERA_END_EFFECTOR_ID = 2

SERVO_DOOR1				= 0x01
SERVO_DOOR2				= 0x02
SERVO_CUTTER			= 0x03
END_EFFECTOR_ID 		= 0x10

# Section encoder
encoder_pulse_base_l	= 2000 * 4
encoder_pulse_base_r	= 2000 * 4
encoder_pulse_lift_l	= 2000 * 4
encoder_pulse_lift_r	= 2000 * 4
encoder_pulse_middle	= 2000 * 4
encoder_pulse_turret	= 2000 * 4
encoder_pulse_forward	= 2000 * 4


# Section api
url = "localhost:8080/api"

# section camera
camera_D0			= 128
camera_focus_length1 = 50
camera_focus_length2 = 50
camera_focus_length3 = 50

# section robot
workspace_z			= 6000
workspace_y			= 1800
workspace_x			= 4000

arm_dist_from_joint_turret = 1000
arm_dist_from_joint_forward = 1000
arm_forward_max_length = 1000
arm_min_workspace = arm_dist_from_joint_turret + arm_dist_from_joint_forward
arm_max_workspace = arm_min_workspace + arm_forward_max_length

# section algorithm
scan_dist_interesting = 1500
visual_time_delay 	= 2000 # ms
visual_radius_accept = 50
visual_forward_length = 100 # mm
visual_speed = 10 # mm/s
planner_update_time = 4 + 1.5 + 3.5 * (7 - 1) #ms