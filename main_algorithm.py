#!/usr/bin/python3

import time
import cv2
import sys
import numpy as np

import time, datetime

import config
import util
# driver
from Driver.camera import DriverCamera
from Driver.stereo import DriverStereo
# visual servo
import motion_control as mc
import visual_servo
from visual_servo import VisualServo, AvoidMango
from motion_control import Planner
# detction
import mango_detection.yolo as yolo

cam1 = 1
cam2 = 2

net = yolo.load('yolov3_3300.weights')
cam_on_arm = DriverCamera(cam1)
cam_end_arm = DriverStereo(cam2)
planner = Planner().getInstance()


tree_position = [
    38862 / config.encoder_pulse_base_l, 
    72753 / config.encoder_pulse_base_l,
    72753 / config.encoder_pulse_base_l, 
    106987 / config.encoder_pulse_base_l, 
    106987 / config.encoder_pulse_base_l, 
    140380 / config.encoder_pulse_base_l, 
]
# tree_position = [
#     1362 / config.encoder_pulse_base_l, 
#     32555 / config.encoder_pulse_base_l, 
# ]
hieght = [
    37459 / config.encoder_pulse_lift_l,
    26236 / config.encoder_pulse_lift_l,
    25123 / config.encoder_pulse_lift_l
]
y_pos_for_turn_arm = 40000 / config.encoder_pulse_lift_l

show_images = True
avoidMango = AvoidMango()

turret_constant = -0.04
lift_up_constant = 0.400
lift_up_accept = 5 # pixel

def wait(timeout = 0): # ms
    start = datetime.datetime.now()
    while not planner.is_empty() or planner.is_moving():
        if timeout > 0 and ((datetime.datetime.now() - start).microseconds / 1000) > timeout:
            return 0
        time.sleep(0.7)

    return 1

def makeRectangle(frame, boxes):
    x, y, w, h = boxes
    fh, fw = frame.shape[:2]
    p1 = (x, y)
    p2 = (x + w, y + h)
    # print (p1, p2)
    cv2.rectangle(frame, p1, p2, (255,0,0), 2, 1)
    pc = tuple(util.get_center(boxes))
    fc = tuple(util.get_center([0, 0, fw, fh]))
    cv2.line(frame, fc, pc, (255, 127, 0), 8)
    
    return frame

def pass_tree(idx, state = 0):
    print ("pass across the tree")
    if state in [3]:
        planner.add(Planner.SET_POSITION, [config.FORWARD_MOTOR_ID, 0, config.default_spd[4], 0])
        # turn arm
        planner.add(Planner.SET_POSITION, [config.TURRET_MOTOR_ID, 180, config.default_spd[3], 0])
        # move kart
        planner.add(Planner.SET_POSITION, [config.MIDDLE_MOTOR_ID, config.workspace_x, config.default_spd[0], 0])
        # lift up
        planner.add(Planner.SET_POSITION, [config.LIFT_MOTOR_ID_L, config.workspace_y, config.default_spd[1], 0])
    # move base
    planner.add(Planner.SET_POSITION, [config.BASE_MOTOR_ID_L, tree_position[idx], config.default_spd[2], 0])
    # lift down
    planner.add(Planner.SET_POSITION, [config.LIFT_MOTOR_ID_L, hieght[0], config.default_spd[1], 0])

def left_to_right():
    print ("turn arm from right to left but move kart from left to right")
    planner.add(Planner.SET_POSITION, [config.FORWARD_MOTOR_ID, 0, config.default_spd[4], 0], False)
    # move kart
    # planner.add(Planner.SET_POSITION, [config.MIDDLE_MOTOR_ID, 0, config.default_spd[0], 0])
    # turn arm
    planner.add(Planner.SET_POSITION, [config.TURRET_MOTOR_ID, 180, config.default_spd[3], 0])
    # move kart
    planner.add(Planner.SET_POSITION, [config.MIDDLE_MOTOR_ID, config.workspace_x, config.default_spd[0], 0])
    # lift down
    # planner.add(Planner.SET_POSITION, [config.LIFT_MOTOR_ID_L, y_pos_before_turn, config.default_spd[1], 0])

def right_to_left():
    print ("turn arm from left to right but move kart from right to left")

    planner.add(Planner.SET_POSITION, [config.FORWARD_MOTOR_ID, 0, config.default_spd[4], 0], False)
    # move kart
    # planner.add(Planner.SET_POSITION, [config.MIDDLE_MOTOR_ID, config.workspace_x, config.default_spd[0], 0])
    # turn arm
    planner.add(Planner.SET_POSITION, [config.TURRET_MOTOR_ID, 0, config.default_spd[3], 0])
    # move kart
    planner.add(Planner.SET_POSITION, [config.MIDDLE_MOTOR_ID, 0, config.default_spd[0], 0])
    # lift down
    # planner.add(Planner.SET_POSITION, [config.LIFT_MOTOR_ID_L, y_pos_before_turn, config.default_spd[1], 0])

def drop_fruit(color, state):
    print ("cut fruit")
    # servo
    planner.add(Planner.CUT, [True])
    planner.add(Planner.SET_POSITION, [config.FORWARD_MOTOR_ID, 0, config.default_spd[0], 0])

    wait()
    ck = 1
    depth = planner.get_control().get_depth()
    if depth <= config.cut_length:
        side = (state in [2, 3])
        # move to drop
        planner.add(Planner.SET_POSITION, [config.MIDDLE_MOTOR_ID, config.drop_position[config.MIDDLE_MOTOR_ID][2 * side + color], config.default_spd[0], 0])
        planner.add(Planner.SET_POSITION, [config.TURRET_MOTOR_ID, config.drop_position[config.TURRET_MOTOR_ID][2 * side + color], config.default_spd[3], 0])
        planner.add(Planner.SET_POSITION, [config.FORWARD_MOTOR_ID, config.drop_position[config.FORWARD_MOTOR_ID][2 * side + color], config.default_spd[4], 0])
    else:
        ck = 0
    print ("drop fruit")
    # servo
    planner.add(Planner.CUT, [False])
    return ck

def s1(): # turn arm into qaudrant 3th
    planner.add(Planner.SET_POSITION, [config.MIDDLE_MOTOR_ID, 0, config.default_spd[0], 0])
    planner.add(Planner.SET_POSITION, [config.TURRET_MOTOR_ID, 57.6, config.default_spd[3], 0])

def s2(): # turn arm into qaudrant 4th
    planner.add(Planner.SET_POSITION, [config.MIDDLE_MOTOR_ID, config.workspace_x, config.default_spd[0], 0])
    planner.add(Planner.SET_POSITION, [config.TURRET_MOTOR_ID, (180-57.6), config.default_spd[3], 0])

def s3(): # turn arm into qaudrant 1st
    planner.add(Planner.SET_POSITION, [config.MIDDLE_MOTOR_ID, config.workspace_x, config.default_spd[0], 0])
    planner.add(Planner.SET_POSITION, [config.TURRET_MOTOR_ID, -(180-57.6), config.default_spd[3], 0])

def s4(): # turn arm into qaudrant 2nd
    planner.add(Planner.SET_POSITION, [config.MIDDLE_MOTOR_ID, 0, config.default_spd[0], 0])
    planner.add(Planner.SET_POSITION, [config.TURRET_MOTOR_ID, -57.6, config.default_spd[3], 0])

def s5(state):
    for i in [1, 1]:
        time.sleep(0.5)
        planner.wait()
        
    # task a photo by using camera on arm
    frame = cam_end_arm.read()[1]
    # find mango on image
    result = yolo.detect(frame, net, 0.8, False, "MainControl")
    # if have mango at least one
    if 'boxes' in result:
        visual_servo.lift_up(cam_end_arm, planner, net)
        # create visual servo
        vs = VisualServo(net, cam_end_arm, avoidMango, state, True) # pass the cam
        # get result from vs
        result = vs.start()
        # print (result)
        # move to drop fruit
        if result:
            # y_pos_before_turn = planner.get_control().get_pos()[1]
            ck = drop_fruit(vs.get_color(), state)
            if not ck:
                return -1
            return 1
        else:
            return -1
        # do previous state again

    return 0

def main():
    global y_pos_before_turn

    tree_idx = -1
    quadrant = 0 # count by state - 5555
    state = 1
    i = 1

    print ("Start")
    while tree_idx < 6:
        if planner.is_empty() and not planner.is_moving():
            sys.stdout.write("\n")
            print ("state:", state)
            if state == 1:
                if quadrant != state:
                    tree_idx += 1
                    if (tree_idx >= len(tree_position)): break
                    pass_tree(tree_idx, 1)
                    # avoidMango.clear()
                
                quadrant = 1
                state = 5
            elif state == 2:
                if quadrant != state:
                    # y_pos_before_turn = y_pos_start
                    left_to_right()
                    # avoidMango.clear()
                
                quadrant = 2
                state = 5
            elif state == 3:
                if quadrant != state:
                    tree_idx += 1
                    if (tree_idx >= len(tree_position)): break
                    pass_tree(tree_idx, 3)
                    # avoidMango.clear() 
                
                quadrant = 3
                state = 5
            elif state == 4:
                if quadrant != state:
                    # y_pos_before_turn = y_pos_start
                    right_to_left()
                    # avoidMango.clear()
                
                quadrant = 4 
                state = 5
            elif state == 5:
                avoidMango.clear()
                for h in hieght:
                    ck = 1
                    cc = 0
                    while ck:
                        planner.add(Planner.SET_POSITION, [config.LIFT_MOTOR_ID_L, h, config.default_spd[1], 0])
                        planner.add(Planner.SET_POSITION, [config.FORWARD_MOTOR_ID, 0, config.default_spd[4], 0])
                        
                        if quadrant == 1: s1()
                        elif quadrant == 2: s2()
                        elif quadrant == 3: s3()
                        elif quadrant == 4: s4()


                        for i in [1, 1]:
                            time.sleep(0.5)
                            planner.wait(0)

                        res = s5(quadrant)
                        if res == 0:
                            ck = 0
                        elif res < 0:
                            cc += 1

                        if cc >= 1:
                            break
                    
                state = (quadrant % 4) + 1

                # print (planner)
        else:
            sys.stdout.write("\rWaiting " + "." * i)

        i += 1

        time.sleep(0.5)

    print ("Finish")

if __name__ == '__main__':
    try:
        cam_on_arm.start()
        cam_end_arm.start()
        main()
    except Exception as e:
        print ("Error in main:", e)
    finally:
        cam_on_arm.stop()
        cam_end_arm.stop()
    

    planner.stop()