import numpy as np

# Section ID
BASE_MOTOR_ID_L     = 'drive_a'
BASE_MOTOR_ID_R     = 'drive_b'
LIFT_MOTOR_ID_L     = 'elev_a'
LIFT_MOTOR_ID_R     = 'elev_b'
MIDDLE_MOTOR_ID     = 'middle'
TURRET_MOTOR_ID     = 'turret'
FORWARD_MOTOR_ID    = 'forward'

MOTOR_GROUP = [
    (MIDDLE_MOTOR_ID, ),                    # x
    (LIFT_MOTOR_ID_L, LIFT_MOTOR_ID_R),     # y
    (BASE_MOTOR_ID_L, BASE_MOTOR_ID_R),     # z
    (TURRET_MOTOR_ID, ),                    # a
    (FORWARD_MOTOR_ID, ),                   # b
]

# Section encoder
encoder_pulse_base_l    = (2000 * 4) / (2 * np.pi * 75)
encoder_pulse_base_r    = (2000 * 4) / (2 * np.pi * 75)
encoder_pulse_lift_l    = (2000 * 4) / (8 * 20)
encoder_pulse_lift_r    = (2000 * 4) / (8 * 20)
encoder_pulse_middle    = (160 * 1024 * 4) / (8 * 20)
encoder_pulse_turret    = (2000 * 4) / 360
encoder_pulse_forward   = (160 * 1024 * 4) / (5 * 24)

default_spd = [300, 2, 2, 2, 300]

_moving_threshold = {
	BASE_MOTOR_ID_L: 5,
	BASE_MOTOR_ID_R: 5,
	LIFT_MOTOR_ID_L: 5,
	LIFT_MOTOR_ID_R: 5,
	MIDDLE_MOTOR_ID: 17,
	TURRET_MOTOR_ID: 3,
	FORWARD_MOTOR_ID: 17,
}

CAMERA_END_EFFECTOR = 'http://127.0.0.1:8082/stream.mjpg?w=1280&h=720'
camera_focus_length = 250

SERVO_DOOR              = 0x03
SERVO_CUTTER            = 0x01
END_EFFECTOR_ID         = 'endeff'

# Section servo position
servo_door_close        = 1880
servo_door_open         = 1150
servo_cutter_cut        = 1000
servo_cutter_open       = 1800

basket = {
    0: 182.25,  # tune
    1: 0,  # tune
} # deg

middle_position = {
    0: 195.3125, # mm, tune
    1: 1074.21875, # mm, tune
}

# Section api
url = "http://localhost:8080/api"

# section robot
workspace_z         = 15000 # tune
workspace_y         = 1260 # tune
workspace_x         = 1269.5 # tune
workspace_arm_offset_x = 195.3125 # tune

offset_x_min = 0 - middle_position[0] + arm_min_workspace
offset_x_max = workspace_x - middle_position[1] + arm_min_workspace

# Section arm configure
arm_start_position = 0 # tune
arm_dist_from_joint_turret = 840 # tune
arm_forward_max_length = 543.640136719 # 555.17578125 # tune

arm_min_workspace = arm_dist_from_joint_turret
arm_max_workspace = arm_min_workspace + arm_forward_max_length

# Section algorithm
planner_update_time = 4 + 1.5 + 3.5 * (8 - 1) #ms

# section visual servo
visual_max_move = 10 # mm  # tune
visual_failed_count = 10  # tune

# object tracking
object_tracker = 'kcf' # csrt, boosting, mil, tld, mediainflow, mosse
object_track_count = 10

camera_K_offset_x = 30 # pixel

# section state move turret for find max score
deg_find_score = 0.5# deg  # tunev

# section operate basket
cut_count = 2
cut_length = 120

# section general
default_speed = 50 # mm/s  # not use in new control
accept_move = 100 # mm  # tune
moving_threshold = 20 # mm/ s  # tune