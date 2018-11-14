import time
import hashlib
import requests
import datetime

import numpy as np

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
