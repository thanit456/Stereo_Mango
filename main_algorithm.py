#!/usr/bin/python3

import cv2
import config
import driver
import math
import time
import numpy as np

# visual servo
from visual_servo import VisualServo
from motion_control import Planner

# detction
import mango_detection.yolo as yolo

net = yolo.load()
cam_on_arm = driver.DriverCamera(config.CAMERA_ON_ARM)
cam_end_arm = driver.DriverCamera(config.CAMERA_END_EFFECTOR)
planner = Planner()

tree_position = [1000-1200, 1000+1200, 3000-1200, 3000+1200, 5000-1200, 5000+1200]
y_pos_for_turn_arm = 1000
y_pos_start = 500
y_pos_before_turn = 0

show_images = True

def lift_up(state = 0):
    print ("lift up, state:", state)
    planner.add(planner.SET_PULSE, [config.FORWARD_MOTOR_ID, 0, config.default_spd[4], 0])
    # lift up
    planner.add(planner.SET_POSITION, [config.LIFT_MOTOR_ID_L, y_pos_for_turn_arm, config.default_spd[1], 0])
    planner.add(planner.SET_POSITION, [config.MIDDLE_MOTOR_ID, config.workspace_x / 2, config.default_spd[0], 0])
    # turn arm
    if state > 0:
        planner.add(planner.SET_POSITION, [config.TURRET_MOTOR_ID, 0 if state == 1 else 180, config.default_spd[3], 0])

def pass_tree(tree_idx, state):
    print ("pass across the tree")
    lift_up(state)
    # lift up
    planner.add(planner.SET_POSITION, [config.LIFT_MOTOR_ID_L, config.workspace_y, config.default_spd[1], 0])
    # move base
    planner.add(planner.SET_POSITION, [config.BASE_MOTOR_ID_L, config.tree_position[tree_idx], config.default_spd[2], 0])
    # lift down
    planner.add(planner.SET_POSITION, [config.LIFT_MOTOR_ID_L, y_pos_start, config.default_spd[1], 0])

def left_to_right():
    print ("turn arm from right to left but move kart from left to right")
    lift_up(3)
    # move kart
    planner.add(planner.SET_POSITION, [config.MIDDLE_MOTOR_ID, config.workspace_x, config.default_spd[0], 0])
    # lift down
    planner.add(planner.SET_POSITION, [config.LIFT_MOTOR_ID_L, y_pos_before_turn, config.default_spd[1], 0])

def right_to_left():
    print ("turn arm from left to right but move kart from right to left")
    lift_up(1)
    # move kart
    planner.add(planner.SET_POSITION, [config.MIDDLE_MOTOR_ID, 0, config.default_spd[0], 0])
    # lift down
    planner.add(planner.SET_POSITION, [config.LIFT_MOTOR_ID_L, y_pos_before_turn, config.default_spd[1], 0])

def drop_fruit(color):
    print ("drop fruit")
    # side = (planner.get_control().get_pos()[0] > config.workspace_x / 2)
    lift_up()
    # turn arm
    planner.add(planner.SET_POSITION, [config.TURRET_MOTOR_ID, config.basket[color], config.default_spd[3], 0])
    # move to drop
    planner.add(planner.SET_POSITION, [config.MIDDLE_MOTOR_ID, config.middle_position[color], config.default_spd[0], 0])

    # servo

    # lift down
    planner.add(planner.SET_POSITION, [config.LIFT_MOTOR_ID_L, y_pos_before_turn, config.default_spd[1], 0])

def s1(): # turn arm into qaudrant 1
    planner.add(planner.SET_POSITION, [config.MIDDLE_MOTOR_ID, 0, config.default_spd[0], 0])
    planner.add(planner.SET_POSITION, [config.TURRET_MOTOR_ID, 62.1, config.default_spd[3], 0])

def s2():
    planner.add(planner.SET_POSITION, [config.MIDDLE_MOTOR_ID, config.workspace_x, config.default_spd[0], 0])
    planner.add(planner.SET_POSITION, [config.TURRET_MOTOR_ID, (180-62.1), config.default_spd[3], 0])

def s3():
    planner.add(planner.SET_POSITION, [config.MIDDLE_MOTOR_ID, config.workspace_x, config.default_spd[0], 0])
    planner.add(planner.SET_POSITION, [config.TURRET_MOTOR_ID, -(180-62.1), config.default_spd[3], 0])

def s4():
    planner.add(planner.SET_POSITION, [config.MIDDLE_MOTOR_ID, 0, config.default_spd[0], 0])
    planner.add(planner.SET_POSITION, [config.TURRET_MOTOR_ID, -62.1, config.default_spd[3], 0])

def s5():
    # task a photo by using camera on arm
    frame = cam_on_arm.read()
    # find mango on image
    result = yolo.detect(frame, net, 0.8, show_images)
    # if have mango at least one
    if result != {}:
    #   create visual servo
        vs = VisualServo(net, cam_end_arm) # pass the cam
    #   get result from vs
        result = vs.start()
        # move to drop fruit
        if result:
            drop_fruit(0)
    #   do previous state again
        return True

    return False

def main():
    tree_idx = -1
    quadrant = 0 # count by state - 5555
    state = 1

    print ("Start")
    while tree_idx < 6:
        if planner.is_empty() and not planner.get_control().is_moving():
            print ("state:", state)
            if state == 1:
                if quadrant != state:
                    tree_idx += 1
                    if tree_idx >= 6: break
                    pass_tree(tree_idx)
                quadrant = 1
                s1()
                state = 5
            elif state == 2:
                if quadrant != state:
                    left_to_right()
                quadrant = 2
                s2()
                state = 5
            elif state == 3:
                if quadrant != state:
                    tree_idx += 1
                    if tree_idx >= 6: break
                    pass_tree(tree_idx) 
                quadrant = 3
                s3()
                state = 5
            elif state == 4:
                if quadrant != state:
                    right_to_left()
                quadrant = 4 
                s4()
                state = 5
            elif state == 5:
                if not s5():
                    state = ((quadrant + 1) % 4) + 1
        else:
            print ("wait ...")

        time.sleep(0.5)

    print ("Finish")

if __name__ == '__main__':
    try:
        cam_on_arm.start()
        cam_end_arm.start()
        main()
    finally:
        planner.stop()
        cam_on_arm.stop()
        cam_end_arm.stop()