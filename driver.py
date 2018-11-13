import time
import hashlib
import requests
import datetime

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

secret_key = b'Eic981234'

def generate_otp():
    timestamp = int(time.time()*1000)
    timestamp = str(timestamp)
    h = hashlib.sha256()
    h.update(timestamp.encode() + secret_key)
    return h.digest()[:8].hex()+timestamp

class DriverMotor:
    """docstring for DriverMotor"""
    def __init__(self, id, host = config.url, ppmm = 0):
        self.url_host = host
        self.id = str(id)
        self.ppmm = ppmm

        self.moving_threshold = 1
        self.delay_time = 4 + 1.5 + 3.5 #ms

        # self.set_mode(1)
        # self.working_length = 1000 # mm

        self.en = 0
        self.cur_pos = 0
        self.cur_velo = 0
        self.goal_pos = 0
        self.goal_velo = 0
        self.goto_pos = 0
        self.goto_velo = 0

        self.group_id = -1
        self.sync_group = None

        # self.lock = Lock()

    def add_sync_group(self, group):
        self.sync_group = group
        self.group_id = group.get_index()

    def get_id(self):
        return self.id

    def set_pulse_per_mm(self, ppr):
        self.ppmm = ppr
        return True

    def set_max_movement(self, max_m):
        self.max_length = max_m

        if self.group_id < 0:
            if self.ppmm == 0:
                raise Exception('Set pulse per mm before call this function.')

            try:
                return self._post('set', {"max_pos": self.max_length * self.ppmm})
            except Exception as e:
                raise e

    def set_min_movement(self, min_m):
        self.min_length = min_m

        if self.self.group_id < 0:
            if self.ppmm == 0:
                raise Exception('Set pulse per mm before call this function.')

            try:
                return self._post('set', {"min_pos": self.min_length * self.ppmm})
            except Exception as e:
                raise e

    def set_mode(self, mode = 1):
        self.mode = mode
        try:
            return self._post('set', {"mode": self.mode})
        except Exception as e:
            raise e

    def get_gpio(self):
        try:
            result = self._post('get', ['gpio1', 'gpio2', 'gpio3', 'gpio4', 'gpio_limit_min', 'gpio_limit_max'])
        except Exception as e:
            raise e

        # min, max, 1, 2, 3, 4
        return [result['gpio_limit_min'], result['gpio_limit_max'], result['gpio1'], result['gpio2'], result['gpio3'], result['gpio4']]

    def get_hw_error(self):
        try:
            result = self._post('get', ['hwerr'])
        except Exception as e:
            raise e

        return result['hwerr']

    def get_adc(self, ch):
        ch = 'adc{}'.format(ch)
        try:
            result = self._post('get', [ch])
        except Exception as e:
            raise e

        return result[ch]

    def get_curr(self):
        if self.ppmm == 0:
            raise Exception('Set pulse per mm before call this function.')
        
        if self.group_id < 0:
            try:
                result = self._post('get', ['cur_pos', 'cur_velo'])
            except Exception as e:
                raise e
            else:
                self.cur_pos = result['cur_pos']
                self.cur_velo = result['cur_velo']
        
        return [self.cur_pos / self.ppmm, self.cur_velo]

    def set_goal(self, goal, velo, is_pulse = False):
        if not is_pulse:
            self.goal_pos = goal * self.ppmm
        else:
            self.goal_pos = goal
        self.goal_velo = velo

        if self.group_id < 0:
            try:
                return self._post('set', {"goto_pos": self.goal_pos, 'goto_velo': self.goal_velo})
            except Exception as e:
                raise e

    def enable(self, en = 1):
        self.en = en

        if self.group_id < 0:
            try:
                return self._post('set', {'enable': 1})
            except Exception as e:
                raise e

    def set_moving_threshould(self, th):
        self.moving_threshold = th

    def scan_working_length(self, pwm):
        # set mode to pwm
        self.set_mode(0)
        # move element to min position
        self.set_goal_pwm(-1 * pwm)
        # wait element hit min switch
        gpio_min = self.get_gpio()[0]
        time = datetime.datetime.now()
        while not gpio_min:
            if (datetime.datetime.now() - time).total_milliseconds > self.delay_time:
                gpio_min = self.get_gpio()[0]
                time = datetime.datetime.now()
        # stop movement
        self.set_goal_pwm(0)
        # reset position on controller
        self.reset_position()
        # set min movement
        self.set_min_movement(0)
        # move element to max position
        self.set_goal_pwm(pwm)
        # wait element hit max switch
        gpio_max = self.get_gpio()[1]
        time = datetime.datetime.now()
        while not gpio_max:
            if (datetime.datetime.now() - time).total_milliseconds > self.delay_time:
                gpio_max = self.get_gpio()[1]
                time = datetime.datetime.now()
        # stop movement
        self.set_goal_pwm(0)
        # change mode to position
        self.set_mode(1)        
        # get working length
        self.working_length = self.get_current()
        # set max movement      
        self.set_max_movement(self.working_length)

        print ("Working length: {}".format(self.working_length))


    def is_moving(self):
        return np.abs(self.cur_velo) > self.moving_threshold

    def reset_position(self):
        try:
            return self._post('cmd', ['pos_reset'])
        except Exception as e:
            raise e

    def _post(self, action, post):
        url = "{}/{}".format(self.url_host, action)
        
        data = {"token": generate_otp()}
        data.update({self.id: post})

        result = requests.post(url, json=data)
        if (result.status_code != 200):
            raise Exception('HTTP Error: {}'.format(result.status_code))
        # print (result.json())
        if action == "get":
            return result.json()[self.id]

        return result.json()['success']

class DriverServo:
    servo_template =  {
        "en": 1,
        'min_pos': 1000,
        'max_pos': 2000,
        'cur_pos': 1500,
        'goal_pos': 1500,
    }
    """docstring for DriverServo"""
    def __init__(self, id, host = config.url):
        self.id = str(id)
        self.url = host

        self.servo = [dict(DriverServo.servo_template) for i in range(4)]
        self.range_finder = 0

        self.sync_group = None
        self.group_id = -1

    def add_sync_group(self, group):
        self.sync_group = group
        self.group_id = group.get_index()

    def get_id(self):
        return self.id

    def get_range(self):
        if self.group_id < 0:
            a = self._post('get', ['range'])
            self.range_finder = a['range'] if a < 65535 else 0

        return self.range_finder

    def enable(self, idx, en):
        self.servo[idx]['en'] = en

    def set_min_max(self, idx, minp = 1000, maxp = 2000):
        self.servo[idx]['min_pos'] = minp
        self.servo[idx]['max_pos'] = maxp

    def set(self, idx, pos):
        pos = max(self.servo[idx]['min_pos'], min(self.servo[idx]['max_pos'], pos))
        self.servo[idx]['goal_pos'] = pos

        if self.group_id < 0:
            servo = {}
            for i, x in enumberate(self.servo):
                servo.update({
                    'ch{}_enable'.format(i) : x['en'],
                    'ch{}_pos'.format(i) : x['goal_pos'],
                })
            self._post('set', servo)

    def get(self, idx):
        if group_id < 0:
            a = self._post('get', ['ch{}_pos'.format(i) for i, x in enumberate(self.servo)])
            for i, x in self.servo:
                x['cur_pos'] = a['ch{}_pos'.format(i)]

        return self.servo[idx]['cur_pos']

    def _post(self, action, post):
        url = "{}/{}".format(self.url_host, action)
        data = {"token": generate_otp()}
        data.update({self.id: pos})

        result = requests.post(url, json=data)
        if (result.status_code != 200):
            raise Exception('HTTP Error: {}'.format(result.status_code))
        # print (result.json())
        if action == "get":
            return result.json()[self.id]

        return result.json()['success']

        
class DriverCamera:
    """docstring for DriverCamera"""
    def __init__(self, id, queueSize = 3):
        self.id = id
        # initialize the queue used to store frames read from
        # the video file
        self.Q = Queue(maxsize=max(queueSize, 2))
        self.stopped = False

        self.vs = cv2.VideoCapture(id)

        self.obj_points = []
        self.img_points = []
        self.camera_mtx = []
        self.camera_distortion = []
        self.camera_new_mtx = []

        # self._load()

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
            if self.stopped:
                return
 
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
        self.stopped = True
    
    def more(self):
        # return True if there are still frames in the queue
        return self.Q.qsize() > 0

    def calibrate(self, col = 6, row = 7, size = 25, count = 15, roi = 1):
        self.start()

        print ("Start calibration on camera {}.".format(self.id))

        # termination criteria
        criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, size, 0.001)

        # prepare object points, like (0,0,0), (1,0,0), (2,0,0) ....,(6,5,0)
        objp = np.zeros((col * row, 3), np.float32)
        objp[:, :2] = np.mgrid[0:row, 0:col].T.reshape(-1,2)

        # Arrays to store object points and image points from all the images.
        self.obj_points = [] # 3d point in real world space
        self.img_points = [] # 2d points in image plane.

        img_height = 480
        img_width = 640

        calibrate_count = count
        while calibrate_count > 0:
            img = self.camera.read()
            img_height, img_width = img.shape[:2]

            gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

            # Find the chess board corners
            ret, corners = cv2.findChessboardCorners(gray, (row, col), None)

            # If found, add object points, image points (after refining them)
            if ret == True:
                self.obj_points.append(objp)

                cv2.cornerSubPix(gray, corners, (11, 11), (-1, -1), criteria)
                self.img_points.append(corners)

                # Draw and display the corners
                cv2.drawChessboardCorners(img, (row, col), corners, ret)
                # print (corners)

                cv2.imshow('Images that use calibrate', img)
                cv2.waitKey(500)
                calibrate_count -= 1

                print ("Image {}".format(count - calibrate_count))

        cv2.destroyAllWindows()
        self.stop()

        ret, self.camera_mtx, self.camera_distortion, rvecs, tvecs = cv2.calibrateCamera(self.obj_points, self.img_points, gray.shape[::-1], None, None)
        self.camera_new_mtx, roi = cv2.getOptimalNewCameraMatrix(self.camera_mtx, self.camera_distortion, (img_width, img_height), roi, (img_width, img_height))

        print ("Calibration success.")

        self._save()

    def set_offset(self, x, y, vertical_deg, horizontal_deg):
        self.offset_x = x
        self.offset_y = y

    def _load(self):
        try:
            with open('camera_calibrate_{}.pkl'.format(self.id), 'rb') as input:
                self.obj_points = pickle.load(input)
                self.img_points = pickle.load(input)
                self.camera_mtx = pickle.load(input)
                self.camera_distortion = pickle.load(input)
                self.camera_new_mtx = pickle.load(input)
        except Exception as e:
            print (e)

    def _save(self):
        with open('camera_calibrate_{}.pkl'.format(self.id), 'wb') as output:
            pickle.dump(self.obj_points, output, pickle.HIGHEST_PROTOCOL)
            pickle.dump(self.img_points, output, pickle.HIGHEST_PROTOCOL)
            pickle.dump(self.camera_mtx, output, pickle.HIGHEST_PROTOCOL)
            pickle.dump(self.camera_distortion, output, pickle.HIGHEST_PROTOCOL)
            pickle.dump(self.camera_new_mtx, output, pickle.HIGHEST_PROTOCOL)
