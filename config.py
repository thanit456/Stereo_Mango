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
encoder_pulse_base_l    = (2000 * 4) / (2 * np.pi * 750)
encoder_pulse_base_r    = (2000 * 4) / (2 * np.pi * 750)
encoder_pulse_lift_l    = (2000 * 4) / (8 * 20)
encoder_pulse_lift_r    = (2000 * 4) / (8 * 20)
encoder_pulse_middle    = (160 * 1024 * 4) / (8 * 20)
encoder_pulse_turret    = (2000 * 4) / 360
encoder_pulse_forward   = (160 * 1024 * 4) / (5 * 24)

CAMERA_END_EFFECTOR = 'http://127.0.0.1:8082/stream.mjpg?w=1280&h=720&fps=15'
camera_focus_length = 250

SERVO_DOOR              = 0x03
SERVO_CUTTER            = 0x01
END_EFFECTOR_ID         = 'endeff'

# Section servo position
servo_door_close        = 2000
servo_door_open         = 1300
servo_cutter_cut        = 1000
servo_cutter_open       = 1800

basket = {
    0: 180,
    1: 0,
} # deg

# Section api
url = "http://localhost:8080/api"

# section camera
camera_focus_length = 50 # end-effector

# section robot
workspace_z         = 6000
workspace_y         = 1800
workspace_x         = 1277
workspace_arm_offset_x = 300

overlab_x = 0.9
overlab_y = 0.9

arm_dist_from_joint_turret = 990
arm_forward_max_length = 1550 - arm_dist_from_joint_turret
arm_min_workspace = arm_dist_from_joint_turret
arm_max_workspace = arm_min_workspace + arm_forward_max_length

# section algorithm
planner_update_time = 4 + 1.5 + 3.5 * (8 - 1) #ms

visual_max_move = 100 # mm
visual_failed_count = 30

default_speed = 50 # mm/s
accept_move = 100 # mm