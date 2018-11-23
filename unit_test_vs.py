# import visual_servo
# import driver,time
# from mango_detection import mango_detector as md
# from motion_control import Planner
# import tensorflow as tf
import config

import cv2
import numpy as np

import datetime, time, sys

from Driver.stereo import DriverStereo
from mango_detection import yolo
from visual_servo import VisualServo, AvoidMango
from motion_control import Planner


def wait(planner, timeout = 0): # ms
    start = datetime.datetime.now()
    while not planner.is_empty() or planner.is_moving():
        if timeout > 0 and ((datetime.datetime.now() - start).microseconds / 1000) > timeout:
            return 0
        time.sleep(0.7)

    return 1


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
    net = yolo.load('yolov3_4500.weights')
    cam_end_arm = DriverStereo(2)
    cam_end_arm.start()
    planner = Planner().getInstance()
    # time.sleep(5)
    # planner.get_control().stop()
    # planner.stop()
    time.sleep(1)
    avoidMango = AvoidMango()
    vs = VisualServo(net, cam_end_arm, avoidMango, True)
    if vs.start(0):
        drop_fruit(planner, vs.get_color(), 1)

    # i = 1
    while not planner.is_empty() or planner.is_moving():
        time.sleep(0.7)
        # sys.stdout.write("wait " + "." * i)
        # i += 1
        # print ("wait ...")
    # print ("\n")

    wait(planner)
    print ("Exit Visual Servo")
    planner.stop()

if __name__ == '__main__':
    main()