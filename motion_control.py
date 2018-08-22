import driver
import config
import time, hashlib, datetime, requests
import numpy as np
from threading import Thread, Lock
import pprint
import math

servo_template =  {
    "en": 1,
    'min_pos': 1000,
    'max_pos': 2000,
    'cur_pos': 1500,
}

motor_template = {
    'en' : 0,
    'cpr' : 8000,
    'cur_pos' : 0,
    'cur_velo': 0,
    'min_pos' : 0,
    'max_pos' : 0,
    'goal_pos' : 0,
    'goal_velo' : 0,
    'goto_pos' : 0,
    'goto_velo': 0,
}

secret_key = b'Eic981234'

def generate_otp():
    timestamp = int(time.time()*1000)
    timestamp = str(timestamp)
    h = hashlib.sha256()
    h.update(timestamp.encode() + secret_key)
    return h.digest()[:8].hex()+timestamp

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

        self.motor = {
            config.BASE_MOTOR_ID_L : dict(motor_template),
            config.BASE_MOTOR_ID_R : dict(motor_template),
            config.LIFT_MOTOR_ID_L : dict(motor_template),
            config.LIFT_MOTOR_ID_R : dict(motor_template),
            config.MIDDLE_MOTOR_ID : dict(motor_template),
            config.TURRET_MOTOR_ID : dict(motor_template),
            config.FORWARD_MOTOR_ID : dict(motor_template),
        }
        self.motor[config.BASE_MOTOR_ID_L]['cpr'] = config.encoder_pulse_base_l
        self.motor[config.BASE_MOTOR_ID_R]['cpr'] = config.encoder_pulse_base_r
        self.motor[config.LIFT_MOTOR_ID_L]['cpr'] = config.encoder_pulse_lift_l
        self.motor[config.LIFT_MOTOR_ID_R]['cpr'] = config.encoder_pulse_lift_r
        self.motor[config.MIDDLE_MOTOR_ID]['cpr'] = config.encoder_pulse_middle
        self.motor[config.TURRET_MOTOR_ID]['cpr'] = config.encoder_pulse_turret
        self.motor[config.FORWARD_MOTOR_ID]['cpr'] = config.encoder_pulse_forward

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

    def move_stereo_cam(self, x_pos, y_pos, z_pos, speed):
        self.move_to_stereo_cam(self.position[0] + x_pos, self.position[1] + y_pos, self.position[2] + z_pos, speed)

    def move_to_stereo_cam(self, x_pos, y_pos, z_pos, speed): # pos in mm, velo in mm/s
        # check input
        x_pos = min(max(x_pos, 0), config.workspace_x)
        y_pos = min(max(y_pos, 0), config.workspace_y)
        z_pos = min(max(z_pos, 0), config.workspace_z)

        x_diff = np.abs(x_pos - self.position[0])
        y_diff = np.abs(y_pos - self.position[1])
        z_diff = np.abs(z_pos - self.position[2])
        
        line_length = np.sqrt(x_diff**2 + y_diff**2 + z_diff**2)
        overall_time = line_length / speed # in seconds (mm / mm * s)
        #print (overall_time)

        x_velo = 0 if overall_time <= 0 else  x_diff / overall_time # mm / s
        y_velo = 0 if overall_time <= 0 else  y_diff / overall_time
        z_velo = 0 if overall_time <= 0 else  z_diff / overall_time
        #print (x_velo, y_velo, z_velo)

        position = [x_pos, y_pos, z_pos]
        velocity = [x_velo, y_velo, z_velo]

        # set output
        for i in range(0, 3):
            for motor_id in config.MOTOR_GROUP[i]:
                self.motor[motor_id]['en'] = 1
                self.motor[motor_id]['goal_pos'] = int(position[i] * self.motor[motor_id]['cpr'])
                self.motor[motor_id]['goal_velo'] = math.ceil(velocity[i] * self.motor[motor_id]['cpr'] / 1000) # pulse / s
                #print(motor_id, self.motor[motor_id]['goal_velo'])

    def move_single_cam(self, x_pos, y_pos, z_pos, speed, deg_to = 0):        
        # prepare
        z_length = self.position[4] + z_pos
        
        z_left = 0
        if z_length < config.arm_min_workspace:
            z_left = z_length - config.arm_min_workspace
        elif z_length > config.arm_max_workspace:
            z_left = z_length - config.arm_max_workspace
            
        z_remain_x = z_left * np.cos((self.position[3] + deg_to) * Planner.degToRad)
        z_remain_z = z_left * np.sin((self.position[3] + deg_to) * Planner.degToRad)
        x_pos_x = x_pos * np.cos((self.position[3] + deg_to) * Planner.degToRad - np.pi / 2)
        x_pos_z = x_pos * np.sin((self.position[3] + deg_to) * Planner.degToRad - np.pi / 2)

        z_pos = min(max(z_length, config.arm_min_workspace), config.arm_max_workspace)
        z_diff = np.abs(self.position[4] - z_pos)

        line_length = np.sqrt(x_pos**2 + y_pos**2 + z_pos**2)
        overall_time = line_length / speed
        z_velo = 0 if overall_time <= 0 else z_diff / overall_time

        self.move_stereo_cam(z_remain_x + x_pos_x, y_pos, z_remain_z + x_pos_z, speed)
        for motor_id in config.MOTOR_GROUP[4]:
            self.motor[motor_id]['en'] = 1
            self.motor[motor_id]['goal_pos'] = int(z_pos * self.motor[motor_id]['cpr'])
            self.motor[motor_id]['goal_velo'] = math.ceil(z_velo * self.motor[motor_id]['cpr'] / 1000)

    def turn_arm(self, deg, speed):
        self.turn_to_arm(self.position[3] + deg, speed)

    def turn_to_arm(self, deg, speed):
        deg = deg  % 360
        
#         check arm length that can move end effector to there
        workspace_min_x = 0 - config.workspace_arm_offet_x
        workspace_max_x = config.workspace_x + config.workspace_arm_offet_x
        arm_pos =  self.position[0] + self.position[4] * np.cos(deg * Planner.degToRad)
        arm_left = 0
        if arm_pos < workspace_min_x:
            arm_left = arm_pos - workspace_min_x
        elif arm_pos > workspace_max_x:
            arm_left = arm_pos - workspace_max_x
        self.move_single_cam(0, 0, arm_left, speed, deg - self.position[3])
            
        deg_diff = np.abs(self.position[3] - deg)
        linear = deg_diff * (self.position[4] - arm_pos)
        overall_time = linear / speed # mm / mm * s
        #print (deg_diff, overall_time, linear)
        velo = 0 if overall_time == 0 else deg_diff / overall_time # deg / s

        for motor_id in config.MOTOR_GROUP[3]:
            self.motor[motor_id]['en'] = 1
            self.motor[motor_id]['goal_pos'] = int((deg % 360) * self.motor[motor_id]['cpr'])
            self.motor[motor_id]['goal_velo'] = math.ceil(velo * self.motor[motor_id]['cpr'] / 1000) # pulse / ms

    def cut_mango(self, is_cut = False):
        cutter = self.servo[config.SERVO_CUTTER]
        
        cutter['en'] = 1
        cutter['cur_pos'] = cutter['min_pos'] if is_cut is True else cutter['max_pos']

    def drop_mango(self, is_drop, is_set = False):
        door = self.servo[config.SERVO_DOOR]

        door['en'] = 1
        door['cur_pos'] = door['min_pos'] if is_drop is True else door['max_pos']
    
    def stop_move(self):
        for i, item in enumerate(self.MOTOR_GROUP):
            for mtd in item:
                self.motor[mtd]['goal_pos'] = self.motor[item[0]]['cur_pos']
                self.motor[mtd]['goal_velo'] = 0

    def get_pos(self):
        return self.position[:3]

    def get_pos_arm(self):
        return [self.position[0] + self.position[5], 
                self.position[1], 
                self.position[2] + self.position[6]]

    def get_arm_length(self):
        return self.position[4]
    
    def get_arm_deg(self):
        return self.position[3]
    
    def get_depth(self):
        return self.depth

    def _update(self):
        result = {}

        url = "{}/get".format(config.url)
        data = {"token": generate_otp()}

        for motor_id in self.motor.keys():
            data[str(motor_id)] = ["cur_pos", "cur_velo"]

        servo = []
        for servo_id in self.servo.keys():
            servo.append('ch{}_pos'.format(servo_id))
        servo.append('range')
        data[str(config.END_EFFECTOR_ID)] = servo
        
        tmp = None
        try:
            tmp = requests.post(url, json=data).json()
        except Exception as e:
            print ("Motor Control", e)
            return 
        
        for motor_id in self.motor.keys():
            self.motor[motor_id]['cur_pos'] = tmp[str(motor_id)]['cur_pos']     # pulse
            self.motor[motor_id]['cur_velo'] = tmp[str(motor_id)]['cur_velo']   # pulse / ms
        for servo_id in self.servo.keys():
            self.servo[servo_id]['cur_pos'] = tmp[str(config.END_EFFECTOR_ID)]['ch{}_pos'.format(servo_id)]

        self.depth = tmp[str(config.END_EFFECTOR_ID)]['range']
        if self.depth == 65535:
            self.depth = 100

        for i in range(len(self.MOTOR_GROUP)):
            self.position[i] = self.motor[self.MOTOR_GROUP[i][0]]['cur_pos'] / self.motor[self.MOTOR_GROUP[i][0]]['cpr']  # mm, (pulse / ms)
            self.velocity[i] = self.motor[self.MOTOR_GROUP[i][0]]['cur_velo']

        self.position[4] += config.arm_min_workspace
        self.position[5] = self.position[4] * np.cos(self.position[3] * Planner.degToRad) # arm x
        self.position[6] = self.position[4] * np.sin(self.position[3] * Planner.degToRad) # arm z

    def _send_position(self):
        url = "{}/set".format(config.url)
        data = {"token": generate_otp()}

        motor = {}
        for motor_id in self.motor.keys():
            motor[motor_id] = {
                'mode': 1,
                'enable' : self.motor[motor_id]['en'],
                'goto_pos' : self.motor[motor_id]['goto_pos'],
                'goto_velo' : self.motor[motor_id]['goto_velo'],
            }
        #pprint.pprint(motor)        
        data.update(motor)
        #pprint.pprint (motor['forward'])
        #pprint.pprint (self.motor['middle'])

        servo = {}
        for servo_id in self.servo.keys():
            servo.update({
                'ch{}_enable'.format(servo_id) : self.servo[servo_id]['en'],
                'ch{}_pos'.format(servo_id) : self.servo[servo_id]['cur_pos'],
            })
        data[config.END_EFFECTOR_ID] = servo

        result = None
        try:
            if Planner.Debug:
                pprint.pprint (data)
            result = requests.post(url, json=data)
            #print (result)
        except Exception as e:
            print ("Motor_Control", e)
            return 
        if (result.status_code != 200):
            print ('HTTP Error: {}'.format(result.status_code))
    
    def loop(self):
        while self._is_running:
            time_diff = (datetime.datetime.now() - self.delay_time).microseconds / 1000
            if time_diff < config.planner_update_time: # ms
                sleep_more = config.planner_update_time - time_diff
                time.sleep(sleep_more / 1000)

            self._update()

            for i in self.motor.keys():
                self.motor[i]['goto_velo'] = self.motor[i]['goal_velo']

                length = self.motor[i]['goal_pos'] - self.motor[i]['cur_pos']
                length2 = self.motor[i]['goal_velo'] * config.planner_update_time * 1000
                length = max(min(length, length2), -length2)
                self.motor[i]['goto_pos'] = int(self.motor[i]['cur_pos'] + length)

            self._send_position()

            self.delay_time = datetime.datetime.now()
            
    def start(self):
        with self.lock:
            self._is_running = True

            self.thread = Thread(target=self.loop)
            self.thread.start()

    def stop(self):
        with self.lock:
            self._is_running = False
