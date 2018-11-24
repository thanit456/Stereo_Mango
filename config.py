import numpy as np

# Section ID
BASE_MOTOR_ID_L     = 'drive_a'
BASE_MOTOR_ID_R     = 'drive_b'
LIFT_MOTOR_ID_L     = 'elev_a'
LIFT_MOTOR_ID_R     = 'elev_b'
MIDDLE_MOTOR_ID     = 'middle'
TURRET_MOTOR_ID     = 'turret'
FORWARD_MOTOR_ID    = 'forward'
END_EFFECTOR_ID     = 'endeff'

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
encoder_pulse_middle    = (175 * 1024 * 4) / (8 * 20)
encoder_pulse_turret    = (2000 * 4) / 360
encoder_pulse_forward   = (175 * 1024 * 4) / (5 * 24)

default_spd = [540*2/3, 2, 2, 2, 540]
k_moving_threshold = 1.5
_moving_threshold = {
	BASE_MOTOR_ID_L: default_spd[2] * k_moving_threshold,
	BASE_MOTOR_ID_R: default_spd[2] * k_moving_threshold,
	LIFT_MOTOR_ID_L: default_spd[1] * k_moving_threshold,
	LIFT_MOTOR_ID_R: default_spd[1] * k_moving_threshold,
	MIDDLE_MOTOR_ID: default_spd[0] * k_moving_threshold,
	TURRET_MOTOR_ID: default_spd[3] * k_moving_threshold,
	FORWARD_MOTOR_ID: 50, # default_spd[4] * k_moving_threshold,
}

# Camera
CAMERA_ON_ARM = 'http://127.0.0.1:8082/stream.mjpg?w=1280&h=720'
CAMERA_END_EFFECTOR = 'http://127.0.0.1:8083/stream.mjpg?w=1280&h=720'

# Stero Camera //OmniVision_OV9750
CAMERA_SENSOR_SIZE_WIDTH = 4860 * 1e-6
CAMERA_SENSOR_SIZE_HEIGHT = 3660 * 1e-6
OV9750_PIXEL_SIZE_X = 3.75 * 1e-3 * 4# mm
OV9750_PIXEL_SIZE_Y = 3.75 * 1e-3 * 4# mm

CAMERA_MARIX_LEFT = np.array([[749.48103027, 0, 700.3758884], [0, 747.05263007, 490.55260203], [0, 0, 1]])
CAMERA_MARIX_RIGHT = np.array([[750.09851697, 0, 679.89629617], [0, 748.27845873, 485.2441566], [0, 0, 1]])

CAMERA_FOCAL_LEFT = [CAMERA_MARIX_LEFT[0][0] * OV9750_PIXEL_SIZE_X, CAMERA_MARIX_LEFT[1][1] * OV9750_PIXEL_SIZE_Y]
CAMERA_FOCAL_RIGHT = [CAMERA_MARIX_RIGHT[0][0] * OV9750_PIXEL_SIZE_X, CAMERA_MARIX_RIGHT[1][1] * OV9750_PIXEL_SIZE_Y]

position_cam_from_end = -100

# Section api
url = "http://localhost:8080/api"

# section robot
workspace_z         = 15000 # tune
workspace_y         = 1360 # tune
workspace_x         = 1160.714285714 # tune
# workspace_arm_offset_x = 195.3125 # tune

SERVO_CUTTER            = 0x01
# Section servo position
servo_cutter_cut        = 1000
servo_cutter_open       = 2000

drop_position = {
    MIDDLE_MOTOR_ID: [0, 24.4140625, 1232.91015625, workspace_x],
    TURRET_MOTOR_ID: [180, 180, 0, 0],
    FORWARD_MOTOR_ID: [119.018554688, 0, 0, 109.86328125],
}

# Section arm configure
arm_start_position = 0 # tune
arm_dist_from_joint_turret = 570 # tune
arm_forward_max_length = 507.589285714 # 555.17578125 # tune 543.640136719
# arm_all_length = 1090

arm_min_workspace = arm_dist_from_joint_turret
arm_max_workspace = arm_min_workspace + arm_forward_max_length

offset_x_min = (drop_position[FORWARD_MOTOR_ID][0] + arm_min_workspace) - drop_position[MIDDLE_MOTOR_ID][0]
offset_x_max = arm_min_workspace - workspace_x + (drop_position[MIDDLE_MOTOR_ID][3] + drop_position[FORWARD_MOTOR_ID][3])

# Section algorithm
planner_update_time = 4 + 1.5 + 3.5 * (8 - 1) #ms

#Avoid Mango
avoid_range = 300

# section visual servo
# visual_max_move = 10 # mm  # tune
visual_failed_count = 15  # tune
visual_low_pass_count = 10

# object tracking
object_tracker = 'kcf' # kcf, csrt, boosting, mil, tld, mediainflow, mosse
object_track_count = 10

end_cam_offset = np.array([0, 70, 70], dtype=np.float64) # x, y
arm_cam_offset = np.array([0, 0, 0], dtype=np.float64) # x, y

# section state move turret for find max score
# deg_find_score = 0.5# deg  # tunev

# section operate basket
# cut_count = 2
cut_length = 160

# section general
# default_speed = 50 # mm/s  # not use in new control
# accept_move = 100 # mm  # tune
# moving_threshold = 20 # mm/ s  # tune