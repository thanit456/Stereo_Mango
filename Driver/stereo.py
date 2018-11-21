import pickle
import numpy as np
import cv2
import imutils

import sys
from threading import Thread, Lock
# import the Queue class from Python 3
if sys.version_info >= (3, 0):
    from queue import Queue
# otherwise, import the Queue class for Python 2.7
else:
    from Queue import Queue

import config

class DriverStereo:
    """docstring for DriverCamera"""
    def __init__(self, id, queueSize = 3):
        self.id = id
        # initialize the queue used to store frames read from
        # the video file
        self.Q = Queue(maxsize=max(queueSize, 2))
        self.stopped = False

        self.vs = cv2.VideoCapture(id)
        self.lock = Lock()

    def start(self):
        # start a thread to read frames from the file video stream
        self.thread = Thread(target=self.update, args=())
        self.thread.daemon = True
        self.thread.start()

    def update(self):
        # keep looping infinitely
        while True:
            # if the thread indicator variable is set, stop the
            # thread
            with self.lock:
                if self.stopped:
                    return
 
            # otherwise, ensure the queue has room in it
            if not self.Q.full():
                # read the next frame from the file
                (grabbed, frame) = self.vs.read()
                frame_height, frame_width, ch = frame.shape
                right = frame[::-1, :int(frame_width/2)]
                left = frame[::-1, int(frame_width/2):]

                left = left[::, ::-1]
                right = right[::, ::-1]

                # add the frame to the queue
                self.Q.put((grabbed, left, right))
            else:
                self.read()

    def read(self):
        # return next frame in the queue
        return self.Q.get()

    def stop(self):
        with self.lock:
            self.stopped = True

    def get_depth(self, diparity = 0):
        if 123.0 < diparity:
            return -2.2714 * diparity + 675.22
        elif (97.0 < diparity <= 123.0) :
            return -4.1764 * diparity + 908.14
        elif (80.0 < diparity <= 97.0) :
            return -5.9514 * diparity + 1079.2
        else: # elif diparity <= 80.0:
            return -8.3246 * diparity + 1275.6
            # return 94558 / diparity - 196.78

    @staticmethod
    def find_lower_mean_r(imgl, imgr, rect_base_l):
        x, y, w, h = rect_base_l
        framel = imgl[y:y+h, x:x+w]

        height, width, ch = imgr.shape

        start_x = max(0, x - int(w/2))
        end_x = min(x + int(w/2), width - w - x)
        lower_i = -1
        lower_mean = None
        for i in range(start_x, end_x):
            framer = imgr[y:y+h, i:i+w]
            mean = cv2.mean(framer - framel)

            if (lower_mean is None or lower_mean > mean):
                lower_i = i
                lower_mean = mean

        return [lower_i, y, w, h]

    @staticmethod
    def find_lower_mean_l(imgl, imgr, rect_base_r):
        x, y, w, h = rect_base_r
        framel = imgr[y:y+h, x:x+w]

        height, width, ch = imgr.shape

        start_x = max(0, x - int(w/2))
        end_x = min(x + int(w/2), width - w - x)
        lower_i = -1
        lower_mean = None
        for i in range(start_x, end_x):
            framer = imgl[y:y+h, i:i+w]
            mean = cv2.mean(framel - framer)

            if (lower_mean is None or lower_mean > mean):
                lower_i = i
                lower_mean = mean

        return [lower_i, y, w, h]

    @staticmethod
    def find_disparity_from_box(boxl, boxr, img_size): # img_size = height, width
        x1, y1, w1, h1 = boxl
        x2, y2, w2, h2 = boxr

        d1 = (x1 + int(w1/2)) - int(img_size[1] / 2)
        d2 = (x2 + int(w2/2)) - int(img_size[1] / 2)

        return d1 - d2