# import visual_servo
# import driver,time
# from mango_detection import mango_detector as md
# from motion_control import Planner
# import tensorflow as tf
import config

import cv2
import numpy as np

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

def main():
    net = yolo.load('yolov3_3300.weights')
    cam_end_arm = DriverStereo(2)
    planner = Planner().getInstance()

    avoidMango = AvoidMango()
    vs = VisualServo(net, cam_end_arm, AvoidMango)
    vs.start()

    print ("Exit Visual Servo")
    planner.stop()

if __name__ == '__main__':
    main()