import driver
import config
import time, hashlib, datetime, requests
import numpy as np
from threading import Thread, Lock
import pprint
import math

import sync_group

servo_template =  {
    "en": 1,
    'min_pos': 1000,
    'max_pos': 2000,
    'cur_pos': 1500,
}

secret_key = b'Eic981234'

def generate_otp():
    timestamp = int(time.time()*1000)
    timestamp = str(timestamp)
    h = hashlib.sha256()
    h.update(timestamp.encode() + secret_key)
    return h.digest()[:8].hex()+timestamp

def get_how_to_turn(source_pos, sink_pos, radius): # pos np.array([x, z])
    dif_pos = sink_pos - source_pos
    length = np.linalg.norm(dif_pos)
    rad = np.arctan(dif_pos[1] / dif_pos[0])
    
    radius = radius if length > radius else length
    x = sink_pos[0] - radius * np.cos(rad)
    z = sink_pos[1] - radius * np.sin(rad)

    return [rad * 180 / np.pi, x, z]

class Planner:
    __instance = None
    degToRad = np.pi / 180
    radToDeg = 180 * np.pi
    
    Debug = False

    @staticmethod
    def getInstance():
        """ Static access method. """
        if Planner.__instance == None:
            Planner()
        return Planner.__instance 

    """docstring for Planner"""
    def __init__(self):
        if Planner.__instance != None:
            raise Exception("This class is a singleton!")
        else:
            Planner.__instance = self

        self.lock = Lock()

        self.position = [0 for i in range(7)] # middle, lift, base, turret, forward
        self.velocity = [0 for i in range(7)] # middle, lift, base, turret, forward
        self.depth = 0

        self.motor = {}
        self.motor[config.BASE_MOTOR_ID_L] = driver.DriverMotor(config.BASE_MOTOR_ID_L, ppmm=config.encoder_pulse_base_l)
        self.motor[config.BASE_MOTOR_ID_R] = driver.DriverMotor(config.BASE_MOTOR_ID_R, ppmm=config.encoder_pulse_base_r)
        self.motor[config.LIFT_MOTOR_ID_L] = driver.DriverMotor(config.LIFT_MOTOR_ID_L, ppmm=config.encoder_pulse_lift_l)
        self.motor[config.LIFT_MOTOR_ID_R] = driver.DriverMotor(config.LIFT_MOTOR_ID_R, ppmm=config.encoder_pulse_lift_r)
        self.motor[config.MIDDLE_MOTOR_ID] = driver.DriverMotor(config.MIDDLE_MOTOR_ID, ppmm=config.encoder_pulse_middle)
        self.motor[config.TURRET_MOTOR_ID] = driver.DriverMotor(config.TURRET_MOTOR_ID, ppmm=config.encoder_pulse_turret)
        self.motor[config.FORWARD_MOTOR_ID] = driver.DriverMotor(config.FORWARD_MOTOR_ID, ppmm=config.encoder_pulse_forward)
        self.motor_group1 = sync_group.Group('overall')
        for motor_id in self.motor.keys(): self.motor_group1.add_driver(self.motor[motor_id])
        for motor_id in self.motor.keys(): self.motor[motor_id].set_moving_threshould(config._moving_threshold[motor_id])

        self.servo = {
            config.SERVO_DOOR : dict(servo_template),
            config.SERVO_CUTTER : dict(servo_template),
        }
        self.servo[config.SERVO_DOOR]['min_pos'] = config.servo_door_open
        self.servo[config.SERVO_DOOR]['max_pos'] = config.servo_door_close
        self.servo[config.SERVO_CUTTER]['min_pos'] = config.servo_cutter_cut
        self.servo[config.SERVO_CUTTER]['max_pos'] = config.servo_cutter_open

        self.delay_time = datetime.datetime(1970,1,1)
        self.MOTOR_GROUP = config.MOTOR_GROUP
        
        self.start()

    def move_to(self, x_pos, y_pos, z_pos, speed): # pos in mm, velo in mm/s
        # check input
        x_pos = min(max(x_pos, 0), config.workspace_x)
        y_pos = min(max(y_pos, 0), config.workspace_y)
        z_pos = min(max(z_pos, 0), config.workspace_z)

        new_pos = np.array([x_pos, z_pos])
        cur_pos = np.array(self.position[5:7])
        cur1_pos = np.array([self.position[0], self.position[2]])
        
        dif_pos = new_pos - cur_pos
        move_length = np.linalg.norm(dif_pos)

        pos = list(self.position[0:5])
        pos[1] = y_pos
        velo = list(config.default_spd)

        if config.arm_min_workspace < self.position[4] + move_length < config.arm_max_workspace:
            pos[4] += move_length            
        else:
            pos[4] = config.arm_min_workspace + (config.arm_forward_max_length * 0.0)

        rad, x, z = get_how_to_turn(cur1_pos, new_pos, pos[4])
        pos[0] = x
        pos[2] = z
        pos[3] = rad

        self.set_all_position(pos, velo)

    # def cut_mango(self, is_cut):
    #     cutter = self.servo[config.SERVO_CUTTER]
        
    #     cutter['en'] = 1
    #     cutter['cur_pos'] = cutter['min_pos'] if is_cut is True else cutter['max_pos']

    # def drop_mango(self, is_drop):
    #     door = self.servo[config.SERVO_DOOR]

    #     door['en'] = 1
    #     door['cur_pos'] = door['min_pos'] if is_drop is True else door['max_pos']
    
    def stop_move(self):
        for i, item in enumerate(self.MOTOR_GROUP):
            for mtd in item:
                self.motor[mtd].set_goal(self.motor[item[0]].get_curr()[0], 0)

    def is_moving(self): # 
        for id in self.motor:
            if self.motor[id].is_moving():
                return True
        return False

    def get_pos(self):
        return (self.position[0], self.position[1], self.position[2])

    def get_pos_arm(self):
        return (self.position[5], self.position[1], self.position[6])
    
    # def get_depth(self):
    #     return self.depth

    def set_all_position(self, pos, velo): # x, y, z, deg, arm
        for i in range(len(config.MOTOR_GROUP)):
            for id in config.MOTOR_GROUP[i]:
                self.motor[id].enable(1)
                self.motor[id].set_goal(pos[i], velo[i])

    def set_position(self, motor_id, pulse, velo, error_wait = 0):
        self.motor[motor_id].enable(1)
        self.motor[motor_id].set_goal(pulse, velo, False)
        if wait:
            while abs(self.motor[motor_id].get_curr()[0] - pulse) >= error_wait:
                time.sleep(0.5)

    def set_pulse(self, motor_id, pulse, velo, error_wait = 0):
        self.motor[motor_id].enable(1)
        self.motor[motor_id].set_goal(pulse, velo, True)
        if wait:
            while abs(self.motor[motor_id].get_curr()[0] - pulse) >= error_wait:
                time.sleep(0.5)

    def _update(self):
        result = {}

        url = "{}/get".format(config.url)
        data = {"token": generate_otp()}

        # servo = []
        # for servo_id in self.servo.keys():
        #     servo.append('ch{}_pos'.format(servo_id))
        # servo.append('range')
        # data[str(config.END_EFFECTOR_ID)] = servo
        
        self.motor_group1.get()
        tmp = None
        try:
            tmp = requests.post(url, json=data).json()
        except Exception as e:
            print ("Motor Control", e)
            return 
        
#         for servo_id in self.servo.keys():
#             self.servo[servo_id]['cur_pos'] = tmp[str(config.END_EFFECTOR_ID)]['ch{}_pos'.format(servo_id)]

        # self.depth = tmp[str(config.END_EFFECTOR_ID)]['range']
        # if self.depth == 65535:
        #     self.depth = 100

        for i in range(len(config.MOTOR_GROUP)):
            self.position[i], self.velocity[i] = self.motor[config.MOTOR_GROUP[i][0]].get_curr()

        self.position[3] += config.arm_start_position
        self.position[4] += config.arm_min_workspace
        self.position[5] = self.position[0] + self.position[4] * np.cos(self.position[3] * Planner.degToRad) # arm x
        self.position[6] = self.position[2] + self.position[4] * np.sin(self.position[3] * Planner.degToRad) # arm z

    def _send_position(self):
        url = "{}/set".format(config.url)
        data = {"token": generate_otp()}

        # servo = {}
        # for servo_id in self.servo.keys():
        #     servo.update({
        #         'ch{}_enable'.format(servo_id) : self.servo[servo_id]['en'],
        #         'ch{}_pos'.format(servo_id) : self.servo[servo_id]['cur_pos'],
        #     })
        # data[config.END_EFFECTOR_ID] = servo
#         pprint.pprint(self.position)

        result = None
        self.motor_group1.send()
        try:
            if Planner.Debug:
                pprint.pprint (data)
            result = requests.post(url, json=data)
            #pprint.pprint (data)
        except Exception as e:
            print ("Motor_Control", e)
            return 
        if (result.status_code != 200):
            print ('HTTP Error: {}'.format(result.status_code))
    
    def loop(self):
        i = 0
        while self._is_running:
            time_diff = (datetime.datetime.now() - self.delay_time).microseconds / 1000
            if time_diff < config.planner_update_time: # ms
                sleep_more = config.planner_update_time - time_diff
                time.sleep(sleep_more / 1000)
                            
            if i % 2 == 0:
                self._update()
                self.motor_group1.update()
            else:                
                self._send_position()

            self.delay_time = datetime.datetime.now()
            i = (i + 1) % 2
            
    def start(self):
        self._is_running = True

        # self.cut_mango(False)
        # self.drop_mango(False)

        self.thread = Thread(target=self.loop)
        self.thread.start()

    def stop(self):
        self._is_running = False
