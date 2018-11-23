from motion_control import Planner, Control
import motion_control as mc
import numpy as np
import cv2

import config
import time, datetime
import pprint

import config
# driver
from Driver.camera import DriverCamera
from Driver.stereo import DriverStereo
# visual servo
import visual_servo as vs
# detction
import mango_detection.yolo as yolo


def test_control():
    def wait(con, t = 5):
        st = datetime.datetime.now()
        while con.is_moving(): # or (datetime.datetime.now() - st).seconds < t:
            print (con)
            time.sleep(1)
    print ("Test Control")
    con = Control.getInstance()
    
    # print (con)
    con.set_position(config.BASE_MOTOR_ID_L, 0, config.default_spd[2], 0)
    wait(con)
    print (con)

    # con.move_to(500, 500, 1500)
    # wait(con)
    # print (con)
    
    # con.move_to(500, 500, 1000)
    # wait(con)
    # print (con)
    
    # con.move_to(1000, 500, 1000)
    # wait(con)
    # print (con)

    con.stop()

def test_planner():
    print ("Test Planner")

def test_ik(continues = 0):

    mid_pos = np.array([0 + config.arm_min_workspace, 0], dtype=np.float64)
    for z in range(0, 580,100):
        end_pos = np.array([0 + config.arm_min_workspace, z], dtype=np.float64)

        x, z, turret, forward = mc.inverse_kinematics(mid_pos, end_pos)
        print (x - config.arm_min_workspace, z, turret, forward)
        if continues:
            mid_pos[0] = x
            mid_pos[1] = z

def move_plane():
    control = Control.getInstance()
    control.set_position(config.TURRET_MOTOR_ID, 90, config.default_spd[3], 0)
    control.set_position(config.FORWARD_MOTOR_ID, 0, config.default_spd[4], 0)
    time.sleep(5)
    print (control.get_pos())
    print (control.get_pos_arm())
    control.plane_move(0, 0, 100, 0)
    time.sleep(3)
    control.stop()

def test_lift():
    net = yolo.load('yolov3_4500.weights')
    planner = Planner().getInstance()
    cam_on_arm = DriverCamera(1)

    lift_up_constant = 0.400
    lift_up_accept = 2 # pixel

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
        pc = tuple(vs.get_center(boxes))
        fc = tuple(vs.get_center([0, 0, fw, fh]))
        cv2.line(frame, fc, pc, (255, 127, 0), 8)
        
        return frame

    wait()

    cam_on_arm.start()
    print ("start")
    while True:
        frame = cam_on_arm.read()[1]
        result = yolo.detect(frame, net, 0.8, False, "MainControl")

        tmp = frame.copy() 
        if 'boxes' in result:
            cv2.imshow("Test Lift", makeRectangle(tmp, result['boxes']))
            cv2.waitKey(1)
        else:
            cv2.imshow("Test Lift", tmp)
            cv2.waitKey(1)
            print ("Not Found")
            continue

        fh, fw = frame.shape[:2]
        fc = vs.get_center([0, 0, fw, fh])
        dif_y = fc[1] - result['center'][1]

        if abs(dif_y) < lift_up_accept:
            break

        print ("diff", dif_y, "move", dif_y * lift_up_constant)
        planner.get_control().move(0, dif_y * lift_up_constant, 0, 0)
        wait()

    print ("final lift up")
    planner.get_control().move(0, 120, 0, 0)
    print ("Finish")
    planner.stop()


def main():
    # test_control()
    # test_planner()
    # test_ik()
    # move_plane()
    test_lift()
    

if __name__ == '__main__':    
    main()