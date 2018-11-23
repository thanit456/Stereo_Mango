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
        self.stopped = False

        self.rectify = False
        self._load()
        self.lock = Lock()

    def clear(self):
        if self.Q.empty(): return
        while not self.Q.empty():
            self.read()

    def start(self):
        self.vs = cv2.VideoCapture(self.id)
        self.vs.set(cv2.CAP_PROP_FRAME_WIDTH, 2560)
        self.vs.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)
        self.clear()
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
                    frame_height, frame_width, ch = frame.shape
                    right = frame[::-1, :int(frame_width/2)]
                    left = frame[::-1, int(frame_width/2):]

                    left = left[::, ::-1]
                    right = right[::, ::-1]

                    if self.rectify:
                        left = cv2.remap(left, self.leftMapX, self.leftMapY, REMAP_INTERPOLATION)
                        right = cv2.remap(right, self.rightMapX, self.rightMapY, REMAP_INTERPOLATION)

                # add the frame to the queue
                    self.Q.put((grabbed, left, right))
            elif not self.Q.empty():
                self.read()
        self.vs.release()

    def read(self):
        # return next frame in the queue
        # try:
        return self.Q.get()
        # except Empty:
        #     pass

    def stop(self):
        with self.lock:
            self.stopped = True
            self.thread.join()  

    def get_depth(self, disparity = 1):
        # if 123.0 < disparity:
        #     return -2.2714 * disparity + 675.22
        # elif (97.0 < disparity <= 123.0) :
        #     return -4.1764 * disparity + 908.14
        # elif (80.0 < disparity <= 97.0) :
        #     return -5.9514 * disparity + 1079.2
        # else: # elif disparity <= 80.0:
        #     return -8.3246 * disparity + 1275.6
        # return 94558 / disparity - 196.78
        return 750 * 69 / disparity

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