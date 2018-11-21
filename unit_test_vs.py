# import visual_servo
# import driver,time
# from mango_detection import mango_detector as md
# from motion_control import Planner
# import tensorflow as tf
import config

import cv2
import numpy as np

import time

from Driver.stereo import DriverStereo
from mango_detection import yolo
from visual_servo import VisualServo, AvoidMango
from motion_control import Planner

# def turn_arm_drop_mango(color = 1):
#     if color not in [1, 2]:
#         color = 1
#     #print (config.basket[color])
#     deg_go = config.basket[color] - planner.get_arm_deg()
#     forward = config.arm_min_workspace - planner.get_arm_length()
#     planner.move_single_cam(0, 0, forward, deg_go, config.default_speed)
    
#     return True if not planner.is_moving(4) else False


def drop_fruit(planner, color, state = 0):
    print ("cut fruit")
    # servo
    planner.add(Planner.CUT, [True])
    planner.add(Planner.SET_POSITION, [config.FORWARD_MOTOR_ID, 0, config.default_spd[0], 0])

    idx = 2 * (state in [2, 3]) + color
    # print (idx)
    # move to drop
    planner.add(Planner.SET_POSITION, [config.TURRET_MOTOR_ID, config.drop_position[config.TURRET_MOTOR_ID][idx], config.default_spd[3], 0], False)
    planner.add(Planner.SET_POSITION, [config.MIDDLE_MOTOR_ID, config.drop_position[config.MIDDLE_MOTOR_ID][idx], config.default_spd[0], 0])
    planner.add(Planner.SET_POSITION, [config.FORWARD_MOTOR_ID, config.drop_position[config.FORWARD_MOTOR_ID][idx], config.default_spd[4], 0])

    print ("drop fruit")
    # servo
    planner.add(Planner.CUT, [False])

def main():
    net = yolo.load('yolov3_3300.weights')
    cam_end_arm = DriverStereo(2)
    planner = Planner().getInstance()
    # time.sleep(5)
    # planner.get_control().stop()
    # planner.stop()

    avoidMango = AvoidMango()
    vs = VisualServo(net, cam_end_arm, avoidMango, True)
    if vs.start(0):
        drop_fruit(planner, vs.get_color(), 1)

    while not planner.is_empty() or planner.is_moving():
        time.sleep(0.5)
        print ("wait ...")
    print ("Exit Visual Servo")
    planner.stop()

if __name__ == '__main__':
    main()