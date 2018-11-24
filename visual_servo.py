import cv2
import numpy as np
import datetime, time

import config
import util
from Driver.stereo import DriverStereo
from motion_control import Planner
from mango_detection import yolo

from pprint import pprint

class AvoidMango:
    """docstring for AvoidMango"""
    def __init__(self):
        self.list_mango = []

    def add(self, pos = None):
        # if pos == None:
        #     return
        self.list_mango.append(pos)

    def clear(self):
        self.list_mango.clear()

    def avoid_this(self, pos, avoid_range = 30): # pos is np.array, range has unit was millimeters;
        for a in self.list_mango:
            dif_pos = a - pos
            length = np.linalg.norm(dif_pos)
            if length < avoid_range:
                return True
        return False
        
class VisualServo:
    """docstring for VisualServo"""
    def __init__(self, net, cam, avoidMango, quadrant, show_image = False):
        self.show_image = show_image
        self.color = -1

        self.net = net
        self.camera = cam
        self.avoidMango = avoidMango
        # self.quadrant = quadrant
        
    def get_color(self):
        return self.color

    def _get_pos(self, pos, cur_pos):
        rad = (cur_pos[3]) * np.pi / 180
        pos[2] += config.position_cam_from_end

        x = pos[0] * np.sin(rad) + pos[2] * np.cos(rad)
        z = -pos[0] * np.cos(rad) + pos[2] * np.sin(rad)
        # print (cur_pos[5], x, cur_pos[6], z)
        pos = np.array([cur_pos[5] + x, cur_pos[1] + pos[1], cur_pos[6] + z])
        return pos

    def _add_avoid(self, pos, cur_pos):
        pos = self._get_pos(pos, cur_pos)
        self.avoidMango.add(pos)    

    def loop(self, DEBUG = False):
        planner = Planner.getInstance()
        planner.wait()
        
        print ("VisualServo", "Enter Loop")

        thres = 0.85
        alpha = 0.864
        
        error_count = 0
        low_pass_count = 0
        low_pass_pos = [0, 0, 0]        
        track = None
        while True:
            if not self._is_running: return False

            control = planner.get_control()
            cur_pos = control.get_all_pos()

            # move servo
            frame = self.camera.read()
            if not frame[0]:
                print ("Camera not ready.")
                error_count += 1
                continue

            canvas = frame[1].copy()
            resultL = yolo.detect(frame[1], self.net, thres, get_list=True)
            resultR = yolo.detect(frame[2], self.net, thres, get_list=True, show_image=None)
            [track, diff_x, diff_y, diff_z] = util.get_mango(frame, resultL, resultR, thres, track, canvas)
            
            util.showImage(canvas)
            if track is None:
                error_count += 1
                continue
            
            depth = control.get_depth() if not DEBUG else 300
            if depth <= config.cut_length:
                break

            low_pass_pos[0] = low_pass_pos[0] * alpha + diff_x * (1 - alpha)
            low_pass_pos[1] = low_pass_pos[1] * alpha + diff_y * (1 - alpha)
            low_pass_pos[2] = low_pass_pos[2] * alpha + diff_z * (1 - alpha)
            low_pass_count += 1
            if config.visual_low_pass_count < low_pass_count:
                x = low_pass_pos[0] + config.end_cam_offset[0] # * 0.5
                y = low_pass_pos[1] + config.end_cam_offset[1] # * 0.5
                z = low_pass_pos[2] * 1.1 + config.end_cam_offset[2]

                print ("low pass move distance x:{}, y:{}, z:{}, d:{}".format(x, y, z, diff_z))                    
                # get current position
                if not control.plane_move(x, y, z, 0):
                    self._add_avoid(low_pass_pos, cur_pos)
                    print ("out of range.")
                    return -1

                planner.wait()
                low_pass_count = 0
                low_pass_pos = [0, 0, 0]
                break

            error_count = 0
        print ("VisualServo", "Exits Loop")

        return True if error_count < config.visual_failed_count else False

    def makeRectangle(self, frame, frame_alt, boxes, curr_z):
        cen = boxes[0] + boxes[2]/2, boxes[1] + boxes[3]/2
        w, h = boxes[2:4]
        scale = self.detect_pos[2] / curr_z
        w *= scale
        h *= scale
        #x, y, w, h = makeInt(boxes)
        x, y, w, h = makeInt((cen[0] - w/2, cen[1] - h/2, w, h))
        fh, fw = frame.shape[:2]
        p1 = (x, y)
        p2 = (x + w, y + h)
        # print (p1, p2)
        cv2.rectangle(frame, p1, p2, (255,0,0), 2, 1)
        pc = tuple(get_center(boxes))
        fc = tuple(get_center([0, 0, fw, fh]))
        cv2.line(frame, fc, pc, (255, 127, 0), 8)
        try:
            frame[y:y+h,x:x+w] = frame[y:y+h,x:x+w] // 2 + frame_alt[y:y+h,x-self.disparity:x+w-self.disparity] // 2
        except ValueError:
            pass
        return frame


    def start(self, DEBUG = False):
        self._is_running = True
        return self.run(DEBUG)

    def run(self, DEBUG):
        # self.camera.start()
        res = self.loop(DEBUG)
        # self.camera.stop()

        self._is_running = False
        return res

    def stop(self):
        if self._is_running:
            cv2.destroyAllWindows()
            self.camera.stop()
        self._is_running = False

def lift_up(camera, planner, net):
    for i in [1, 1]:
        planner.wait()
        time.sleep(0.7)

    print ("Enter lift up")

    thres = 0.85
    
    low_pass = [0, 0, 0]
    low_pass_c = 0
    state = 0
    error_c = 0
    ck = 0
    track = None
    while error_c < 10:
        frame = camera.read()
        if frame:
            # cv2.imshow('', canvas)
            canvas = frame[1].copy()
            resultL = yolo.detect(frame[1], net, thres, get_list=True)
            resultR = yolo.detect(frame[2], net, thres, get_list=True, show_image=None)
            [track, dif_x, dif_y, dif_z] = util.get_mango(frame, resultL, resultR, thres, track, canvas)

            if track is None:
                error_c += 1
                continue

            self.color = track[1]

            low_pass[0] = low_pass[0] * 0.864 + dif_x * 0.136
            low_pass[1] = low_pass[1] * 0.864 + dif_y * 0.136
            low_pass[2] = low_pass[2] * 0.864 + dif_z * 0.136
            low_pass_c += 1

            util.showImage(canvas)
            print ("Lift UP", low_pass)
            if low_pass_c == 5:
                if not planner.is_moving():
                    if state  == 0:
                        print ("turn arm")
                        control = planner.get_control()
                        cur_pos = control.get_all_pos()
                        
                        deg = np.arctan(low_pass[0] / low_pass[2]) * 180 / np.pi
                        turn = cur_pos[3] + deg
                        control.set_position(config.TURRET_MOTOR_ID, turn, config.default_spd[3], 0)

                        state = 1
                    elif state == 1:
                        print ("move lift and kart")

                        planner.get_control().plane_move(0, low_pass[1], 0, 0)
                        planner.wait()
                        ck = 1
                        break

                low_pass_c = 0
                low_pass = [0, 0, 0]
            elif planner.is_moving():
                low_pass = [0, 0, 0]
        else:
            error_c += 1

    # planner.get_control().move(0, 130, 0, 0)
    # planner.wait()
    print ("Exit lift up")

    return ck
