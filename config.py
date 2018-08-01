# Section ID
BASE_MOTOR_ID_L 	= 1
BASE_MOTOR_ID_R 	= 2
LIFT_MOTOR_ID_L 	= 3
LIFT_MOTOR_ID_R 	= 4
MIDDLE_MOTOR_ID		= 5
TURRET_MOTOR_ID		= 6
FORWARD_MOTOR_ID	= 7
END_EFFECTOR_MOTOR_ID = 8

CAMERA_STEREO_L_ID	= 0
CAMERA_STEREO_R_ID 	= 1
CAMERA_END_EFFECTOR_ID = 2

LASER_ID			= 10

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
arm_workspace = arm_forward_max_length + arm_dist_from_joint_forward + arm_dist_from_joint_turret


# section algorithm
scan_dist_interesting = 1500
visual_time_delay 	= 2000 # ms
visual_radius_accept = 10
visual_forward_length = 100 # mm
planner_update_time = 4 + 1.5 + 3.5 * (7 - 1) #ms