import cv2
import numpy as np
import datetime, time

import config
from Driver.stereo import DriverStereo
from motion_control import Planner
from mango_detection import yolo

from pprint import pprint

def makeInt(box):
    return [int(i) for i in box]

def get_center(rect, num = 0):
    x, y, w, h = rect
    a = [int(x + w/2), int(y + h / 2)]
    if num:
        return np.array(a, dtype=np.float64)
    else:
        return a

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


def get_pos_from_stereo(frame, lbox, scale=None):
    scale = 1
    if scale is not None:
        cen = lbox[0] + lbox[2]/2, lbox[1] + lbox[3]/2
        w, h = lbox[2:4]
        w *= scale
        h *= scale
        lbox = makeInt((cen[0] - w/2, cen[1] - h/2, w, h))
    else:
        lbox = makeInt(lbox)
    rbox, disparity = DriverStereo.find_lower_mean_r(frame[1], frame[2], lbox)
    # disparity = DriverStereo.find_disparity_from_box(lbox, rbox, frame[1].shape[:2])
    disparity = max(1, disparity)
    diff_z = get_depth(disparity)

    # print (disparity, diff_z)
    
    clbox = get_center(lbox)
    crbox = get_center(rbox)
    # print (clbox, crbox)

    return np.array([int(clbox[0] + disparity/2), int(clbox[1] + disparity / 2), diff_z], dtype=np.float64), disparity
    # return np.array([int()])

def get_depth(disparity = 1):
    if (124<=disparity <= 167):
        depth = 47409/disparity + 15.856
    elif (98<=disparity<=123):
        depth = 49208/disparity + 0.3388
    elif (81<=disparity<=97):
        depth = 46381/disparity + 26.896
    elif (69<=disparity<=80):
        depth = 46044/disparity + 36.039
    else:
        depth = 750*65.3 / disparity
    return depth

def makeRectangle(frame, frame_alt, boxes, disparity):
    cen = boxes[0] + boxes[2]/2, boxes[1] + boxes[3]/2
    w, h = boxes[2:4]
    scale = 1
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
        frame[y:y+h,x:x+w] = frame[y:y+h,x:x+w] // 2 + frame_alt[y:y+h,x-disparity:x+w-disparity] // 2
    except ValueError:
        pass
    return frame

stereo = DriverStereo(2)
thres = 0.5
try:
    net = yolo.load('yolov3_4500.weights')
    stereo.start()

    track = None
    while True:
        frame = stereo.read()
        if frame:
            # cv2.imshow('Cam',frame[1])
            resultL = yolo.detect(frame[1], net, thres, get_list=True)
            resultR = yolo.detect(frame[2], net, thres, get_list=True, show_image=None)

            canvas = frame[1].copy()
            resultF, disparities = yolo.fuse_result(resultL[1], resultR[1], thres, canvas, frame[2])


            if track is not None:
                trackn = yolo.track_result(*track, frame[1], resultF, canvas = canvas)
                if trackn != -1:
                    track = (resultF[trackn][2], track[1])
            else:
                (i, color) = yolo.find_max_scores(frame[1], resultF, thres, canvas)
                if i!=-1:
                    track = (resultF[i][2], color)
            # resultFused = find_same_mango(resultL, resultR)
            # if i >= 0:

            if track is not None:
                imh, imw = frame[1].shape[:2]
                cv2.line(canvas, (int(imw/2),int(imh/2)), tuple(get_center(track[0])), (255, 127, 0), 8)
            cv2.imshow('Fused', canvas)
            #     bounds = (results[1][i][2])
            #     posa, disparity = get_pos_from_stereo(frame, bounds)
            #     print(disparity, get_depth(disparity))
            #     cv2.imshow("Output", makeRectangle(frame[1], frame[2], bounds, disparity))
        cv2.waitKey(1)
except KeyboardInterrupt:
    pass
stereo.stop()