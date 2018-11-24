import pickle
import numpy as np
import cv2
import imutils

import sys
from threading import Thread, Lock
# import the Queue class from Python 3
if sys.version_info >= (3, 0):
    from queue import Queue, Empty
# otherwise, import the Queue class for Python 2.7
else:
    from Queue import Queue, Empty

import config

REMAP_INTERPOLATION = cv2.INTER_LINEAR
class DriverStereo:
    """docstring for DriverCamera"""
    def __init__(self, id, queueSize = 3):
        self.id = id
        # initialize the queue used to store frames read from
        # the video file
        self.Q = Queue(maxsize=max(queueSize, 2))

        self.rectify = True
        self._load()
        self.lock = Lock()

        self.vs = cv2.VideoCapture(self.id)
        self.vs.set(cv2.CAP_PROP_FRAME_WIDTH, 2560)
        self.vs.set(cv2.CAP_PROP_FRAME_HEIGHT, 960)
        self.vs.set(cv2.CAP_PROP_BUFFERSIZE, 1)
        self.stopped = True
        # codec = cv2.VideoWriter_fourcc(*'X264')
        # self.vs.set(cv2.CAP_PROP_FOURCC, codec)

    def clear(self):
        if self.Q.empty(): return
        while not self.Q.empty():
            self.read()

    def start(self):
        self.stopped = False
        # self.clear()
        # start a thread to read frames from the file video stream
        self.thread = Thread(target=self.update, args=())
        self.thread.daemon = True
        self.thread.start()

    def update(self):
        # keep looping infinitely
        while not self.stopped:
            # if the thread indicator variable is set, stop the
            # thread
 
            # otherwise, ensure the queue has room in it
            if not self.Q.full():
                # read the next frame from the file
                (grabbed, frame) = self.vs.read()
                if grabbed:
                    [left, right] = self.split_frame(frame)

                # add the frame to the queue
                    self.Q.put((grabbed, left, right))
            elif not self.Q.empty():
                self.read()
        self.vs.release()

    def read(self):
        # return next frame in the queue
        # try:
        if self.stopped:
            (grabbed, frame) = self.vs.read()
            if grabbed:
                [left, right] = self.split_frame(frame)
            return (grabbed, left, right)
        return self.Q.get()
        # except Empty:
        #     pass

    def stop(self):
        with self.lock:
            self.stopped = True
            self.thread.join()  

    @staticmethod
    def get_depth(disparity = 1):
        disparity = max(1, disparity)
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

    @staticmethod
    def get_true_depth(pos, dif_z):
        x = pos[0] * dif_z * (20/24/600)
        y = pos[1] * dif_z * (20/24/600)
        return (x, y)

    def _load(self):
        try:
            calibration = np.load('../Stereo/output.npz', allow_pickle=None)
        except Exception as e:
            print ("Stereo Driver -", e)
            self.rectify = False
        else:
            self.rectify = True

            self.imageSize = tuple(calibration["imageSize"])
            self.leftMapX = calibration["leftMapX"]
            self.leftMapY = calibration["leftMapY"]
            self.leftROI = tuple(calibration["leftROI"])
            self.rightMapX = calibration["rightMapX"]
            self.rightMapY = calibration["rightMapY"]
            self.rightROI = tuple(calibration["rightROI"])

    def split_frame(self, frame):
        frame_height, frame_width, ch = frame.shape
        right = frame[::-1, :int(frame_width/2)]
        left = frame[::-1, int(frame_width/2):]

        left = left[::, ::-1]
        right = right[::, ::-1]

        if self.rectify:
            left = cv2.remap(left, self.leftMapX, self.leftMapY, REMAP_INTERPOLATION)
            right = cv2.remap(right, self.rightMapX, self.rightMapY, REMAP_INTERPOLATION)

        return [left, right]

    @staticmethod
    def find_lower_mean_r(imgl, imgr, rect_base_l):
        x, y, w, h = rect_base_l
        # print('@',w,h)
        skip = 1# skip = max(1, w//150)

        height, width, ch = imgr.shape

        start_x = max(0, x - w)
        end_x = min(x, width - w)
        w = end_x - start_x
        lower_i = start_x
        lower_mean = None
        framel = imgl[y:y+h:skip, x:x+w:skip]
        for i in range(start_x, end_x):
            framer = imgr[y:y+h:skip, i:i+w:skip]
            mean = sum(cv2.mean(np.square(framer - framel)))
            # print('='*20,mean)

            if (lower_mean is None or lower_mean > mean):
                lower_i = i
                lower_mean = mean
        # print('#',x-lower_i,)

        return [lower_i, y, w, h], x-lower_i

    @staticmethod
    def find_disparity_from_box(boxl, boxr, img_size): # img_size = height, width
        x1, y1, w1, h1 = boxl
        x2, y2, w2, h2 = boxr

        d1 = (x1 + int(w1/2))
        d2 = (x2 + int(w2/2))

        return d1 - d2

    @staticmethod
    def get_pos(pos, cur_pos):
        rad = (cur_pos[3]) * np.pi / 180
        x_pos = true_depth(pos[0], pos[2])
        y_pos = true_depth(pos[1], pos[2])
        x = x_pos * np.sin(rad) + pos[2] * np.cos(rad)
        z = -x_pos * np.cos(rad) + pos[2] * np.sin(rad) + config.position_cam_from_end

        pos = np.array([cur_pos[5] + x, cur_pos[1] + y_pos, cur_pos[6] + z])
        return pos