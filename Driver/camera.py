import pickle
import numpy as np
import cv2
import imutils

import sys
from threading import Thread
# import the Queue class from Python 3
if sys.version_info >= (3, 0):
    from queue import Queue
# otherwise, import the Queue class for Python 2.7
else:
    from Queue import Queue

import config

class DriverCamera:
    """docstring for DriverCamera"""
    def __init__(self, id, queueSize = 3):
        self.id = id
        # initialize the queue used to store frames read from
        # the video file
        self.Q = Queue(maxsize=max(queueSize, 2))
        self.stopped = False

        self.vs = cv2.VideoCapture(id)
        self.vs.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
        self.vs.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)

        # self.obj_points = []
        # self.img_points = []
        # self.camera_mtx = []
        # self.camera_distortion = []
        # self.camera_new_mtx = []

        # self._load()

    def start(self):
        # start a thread to read frames from the file video stream
        self.thread = Thread(target=self.update, args=())
        self.thread.daemon = True
        self.thread.start()
        self.stopped = False

    def update(self):
        # keep looping infinitely
        while not self.stopped:
            # if the thread indicator variable is set, stop the
            # thread
            # if self.stopped:
            #     return
 
            # otherwise, ensure the queue has room in it
            if not self.Q.full():
                # read the next frame from the file
                (grabbed, frame) = self.vs.read()
                if self.img_points:
                    tmp = cv2.undistort(frame, self.camera_mtx, self.camera_distortion, None, self.camera_new_mtx)
                    frame = tmp
 
                # add the frame to the queue
                self.Q.put((grabbed, frame))
            else:
                self.read()

    def read(self):
        # return next frame in the queue
        return self.Q.get()

    def stop(self):
        self.thread = None
        self.stopped = True

    # def calibrate(self, col = 6, row = 7, size = 25, count = 15, roi = 1):
    #     self.start()

    #     print ("Start calibration on camera {}.".format(self.id))

    #     # termination criteria
    #     criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, size, 0.001)

    #     # prepare object points, like (0,0,0), (1,0,0), (2,0,0) ....,(6,5,0)
    #     objp = np.zeros((col * row, 3), np.float32)
    #     objp[:, :2] = np.mgrid[0:row, 0:col].T.reshape(-1,2)

    #     # Arrays to store object points and image points from all the images.
    #     self.obj_points = [] # 3d point in real world space
    #     self.img_points = [] # 2d points in image plane.

    #     img_height = 480
    #     img_width = 640

    #     calibrate_count = count
    #     while calibrate_count > 0:
    #         img = self.camera.read()
    #         img_height, img_width = img.shape[:2]

    #         gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

    #         # Find the chess board corners
    #         ret, corners = cv2.findChessboardCorners(gray, (row, col), None)

    #         # If found, add object points, image points (after refining them)
    #         if ret == True:
    #             self.obj_points.append(objp)

    #             cv2.cornerSubPix(gray, corners, (11, 11), (-1, -1), criteria)
    #             self.img_points.append(corners)

    #             # Draw and display the corners
    #             cv2.drawChessboardCorners(img, (row, col), corners, ret)
    #             # print (corners)

    #             cv2.imshow('Images that use calibrate', img)
    #             cv2.waitKey(500)
    #             calibrate_count -= 1

    #             print ("Image {}".format(count - calibrate_count))

    #     cv2.destroyAllWindows()
    #     self.stop()

    #     ret, self.camera_mtx, self.camera_distortion, rvecs, tvecs = cv2.calibrateCamera(self.obj_points, self.img_points, gray.shape[::-1], None, None)
    #     self.camera_new_mtx, roi = cv2.getOptimalNewCameraMatrix(self.camera_mtx, self.camera_distortion, (img_width, img_height), roi, (img_width, img_height))

    #     print ("Calibration success.")

    #     self._save()

    # def _load(self):
    #     try:
    #         with open('camera_calibrate_{}.pkl'.format(self.id), 'rb') as input:
    #             self.obj_points = pickle.load(input)
    #             self.img_points = pickle.load(input)
    #             self.camera_mtx = pickle.load(input)
    #             self.camera_distortion = pickle.load(input)
    #             self.camera_new_mtx = pickle.load(input)
    #     except Exception as e:
    #         print (e)

    # def _save(self):
    #     with open('camera_calibrate_{}.pkl'.format(self.id), 'wb') as output:
    #         pickle.dump(self.obj_points, output, pickle.HIGHEST_PROTOCOL)
    #         pickle.dump(self.img_points, output, pickle.HIGHEST_PROTOCOL)
    #         pickle.dump(self.camera_mtx, output, pickle.HIGHEST_PROTOCOL)
    #         pickle.dump(self.camera_distortion, output, pickle.HIGHEST_PROTOCOL)
    #         pickle.dump(self.camera_new_mtx, output, pickle.HIGHEST_PROTOCOL)
