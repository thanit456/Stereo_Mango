from Driver.stereo import DriverStereo
from mango_detection import yolo
import cv2
import config

def makeInt(box):
    if not isinstance(box, list):
        box = [box]
    return [int(i) for i in box]

def get_center(rect, num = 0):
    x, y, w, h = rect
    a = [int(x + w/2), int(y + h / 2)]
    if num:
        return np.array(a, dtype=np.float64)
    else:
        return a

def get_mango(frame, resultL, resultR, thres, track, canvas = None):
    idx = -1
    resultF, disparities = yolo.fuse_result(resultL[1], resultR[1], thres, canvas, frame[2])

    if track is not None:
        idx = yolo.track_result(*track, frame[1], resultF, canvas = canvas)
        if idx != -1:
            track = (resultF[idx][2], track[1])
    else:
        (idx, color) = yolo.find_max_scores(frame[1], resultF, thres, canvas)
        if idx != -1 :
            track = (resultF[idx][2], color)

    if idx == -1:
        return [None, 0, 0, 0]

    fh, fw = frame[1].shape[:2]
    fc = get_center([0, 0, fw + disparities[idx], fh])
    box = resultF[idx][2]
    cbox = get_center(box)
    
    dif_z = DriverStereo.get_depth(disparities[idx])
    dif_x = (cbox[0] + disparities[idx] / 2) - fc[0]
    dif_y = fc[1] - cbox[1]
    (dif_x, dif_y) = DriverStereo.get_true_depth([dif_x, dif_y], dif_z)

    dif_z += config.position_cam_from_end
    return [track, dif_x, dif_y, dif_z]


def showImage(frame):
    cv2.imshow("MainControl", frame)
    cv2.waitKey(1)

class Display:
    """docstring for Display"""
    def __init__(self, count):
        self.count = count
    
        