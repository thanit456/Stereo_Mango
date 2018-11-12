import time
import hashlib
import requests
import datetime
import config
from pprint import pprint
from driver import DriverMotor, DriverServo

secret_key = b'Eic981234'
group_id = 1

def generate_otp():
    timestamp = int(time.time()*1000)
    timestamp = str(timestamp)
    h = hashlib.sha256()
    h.update(timestamp.encode() + secret_key)
    return h.digest()[:8].hex()+timestamp

class Group:
    def __init__(self, name):
        global group_id
        self.name = name
        self.idx = group_id
        self.list_driver = {}

        group_id += 1

    def add_driver(self, driver):
        driver.add_sync_group(self)
        id = driver.get_id()
        self.list_driver[id] = driver

    def get_index(self):
        return self.idx

    def get(self):
        result = {}

        url = "{}/get".format(config.url)
        data = {"token": generate_otp()}

        for motor_id in self.list_driver.keys():
            if isinstance(self.list_driver[motor_id], DriverMotor):
                data[str(motor_id)] = ["cur_pos", "cur_velo"]
            elif isinstance(self.list_driver[motor_id], DriverServo):
                data[str(motor_id)] = ['ch{}_pos'.format(i) for i in range(4)]
        
        tmp = None
        try:
            tmp = requests.post(url, json=data).json()
        except Exception as e:
            print ("Motor Control", e)
            return 
        
        for motor_id in self.list_driver.keys():
            if isinstance(self.list_driver[motor_id], DriverMotor):
                self.list_driver[motor_id].cur_pos = tmp[str(motor_id)]['cur_pos'] # pulse
                self.list_driver[motor_id].cur_velo = tmp[str(motor_id)]['cur_velo'] # pulse / ms
            elif isinstance(self.list_driver[motor_id], DriverServo):
                pprint (tmp)
                for i in range(4):
                    self.list_driver[motor_id][i]['cur_pos'] = tmp[str(motor_id)]['ch{}_pos'.format(i)]

    def is_moving(self):
        # self.get()
        for i in self.list_driver.keys():
            if self.list_driver[i].is_moving():
                return True
        return False

    def update(self):
        for i in self.list_driver.keys():
            if isinstance(self.list_driver[i], DriverMotor):
                self.list_driver[i].goto_velo = self.list_driver[i].goal_velo

                length = self.list_driver[i].goal_pos - self.list_driver[i].cur_pos
                length2 = self.list_driver[i].goal_velo * config.planner_update_time * 1000
                length = max(min(length, length2), -length2)
                self.list_driver[i].goto_pos = int(self.list_driver[i].cur_pos + length)
    #                     pprint.pprint (self.motor['forward'])

    def send(self):
        url = "{}/set".format(config.url)
        data = {"token": generate_otp()}

        motor = {}
        for motor_id in self.list_driver.keys():
            if isinstance(self.list_driver[motor_id], DriverMotor):
                motor[motor_id] = {
                    'mode': 1,
                    'enable' : self.list_driver[motor_id].en,
                    'goto_pos' : self.list_driver[motor_id].goto_pos,
                    'goto_velo' : self.list_driver[motor_id].goto_velo,
                }
            elif isinstance(self.list_driver[motor_id], DriverServo):
                servo = {}
                for i, x in enumberate(self.list_driver[motor_id].servo):
                    servo.update({
                        'ch{}_enable'.format(i) : x['en'],
                        'ch{}_pos'.format(i) : x['goal_pos'],
                    })
                motor[motor_id] = servo
        data.update(motor)

        result = None
        try:
            result = requests.post(url, json=data)
            #pprint.pprint (data)
        except Exception as e:
            print ("sync_group - send: \t", e)
            return 
        if (result.status_code != 200):
            print ('HTTP Error: {}'.format(result.status_code))

        # pprint (data)