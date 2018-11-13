import driver
import config
import time, hashlib, datetime, requests
import numpy as np
from threading import Thread, Lock
import pprint
import math

import sync_group

secret_key = b'Eic981234'

def generate_otp():
    timestamp = int(time.time()*1000)
    timestamp = str(timestamp)
    h = hashlib.sha256()
    h.update(timestamp.encode() + secret_key)
    return h.digest()[:8].hex()+timestamp

def inverse_kinematics(middle_pos, end_pos): # pos np.array([x, z])
    ck = 1
    while ck:
        dif_pos = end_pos - middle_pos
        length = np.linalg.norm(dif_pos)

        radius = config.arm_min_workspace if length > config.arm_max_workspace else length
        rad = np.arctan(dif_pos[1] / dif_pos[0]) - np.pi if dif_pos[0] != 0 else (np.pi / 2 if dif_pos[1] > 0 else -np.pi / 2)
        x = end_pos[0] - radius * np.cos(rad) # middle
        z = end_pos[1] - radius * np.sin(rad) # middle

        x -= config.offset_x_min
        if -1e-15 > x > -1e-1: x = 0
        if -1e-15 > z > -1e-1: z = 0
        if (0 <= x <= config.workspace_x) and 0 <= z:
            ck = 0
            break

        x_new = x - max(0, min(config.workspace_x, x))
        z_new = z - max(0, min(config.workspace_z, z))
        middle_pos -= np.array([x_new, z_new], dtype=np.float64)
        # print (rad * 180 / np.pi, x, x_new, z_new, middle_pos, dif_pos)

    return [x + config.offset_x_min, z, rad * 180 / np.pi, radius]

def plan_turn(cur, goto):
    # [-360, 360]
    a = ((cur % 360) + 360) % 360
    b = ((goto % 360) + 360) % 360

    c = ((goto - cur) + 360) % 360
    d = ((cur - goto) + 360) % 360

    # print (cur, goto, c, d)

    return cur + c if c < d else cur - d

import sys
if sys.version_info >= (3, 0):
    from queue import Queue
# otherwise, import the Queue class for Python 2.7
else:
    from Queue import Queue
class Planner:
    __instance = None

    MOVE = 0
    MOVE_TO = 1
    PLANE_MOVE = 2
    SET_PULSE = 3
    SET_POSITION = 4
    CUT = 5
    DROP = 6

    @staticmethod
    def getInstance():
        """ Static access method. """
        if Planner.__instance == None:
            Planner()
        return Planner.__instance 

    """docstring for Planner"""
    def __init__(self, queueSize = 64):
        if Planner.__instance != None:
            raise Exception("This class is a singleton!")
        else:
            Planner.__instance = self


        self.queue = Queue(maxsize=max(queueSize, 32))
        self.control = Control()
        self.cmd = [self.control.move, self.control.move_to, self.control.plane_move, self.control.set_pulse, self.control.set_position, self.control.cut_mango, self.control.drop_mango]

        self.thread = None
        self.start()

    def is_empty(self):
        return self.queue.empty()

    def is_moving(self):
        return self.control.is_moving()

    def get_control(self):
        return self.control

    def add(self, cmd, args):
        if not self.queue.full():
            return self.queue.put((cmd, tuple(args)))
        return False

    def start(self):
        if self.thread == None:
            self._is_running = 1
            self.thread = Thread(target=self.loop)
            self.thread.start()

    def loop(self):
        while self._is_running:
            block = self.queue.get()
            if block[0] < len(self.cmd):
                self.cmd[block[0]](*block[1])

            time.sleep(3)
            while self.control.is_moving():
                time.sleep(3)

    def stop(self):
        self._is_running = 0
        self.thread = None
        self.control.stop()

    def __str__(self):
        txt = ''
        while not self.is_empty():
            a = self.queue.get()
            txt += str(a) + '\n'
        return txt

class Control:
    __instance = None
    _en_servo = False

    @staticmethod
    def getInstance():
        """ Static access method. """
        if Control.__instance == None:
            Control()
        return Control.__instance 

    """docstring for Planner"""
    def __init__(self):
        if Control.__instance != None:
            raise Exception("This class is a singleton!")
        else:
            Control.__instance = self

        self.lock = Lock()

        self.position = [0 for i in range(7)] # middle, lift, base, turret, forward
        self.velocity = [0 for i in range(7)] # middle, lift, base, turret, forward
        self.goal_pos = [0 for i in range(7)]

        self.position[3] += config.arm_start_position
        self.position[4] += config.arm_min_workspace
        self.goal_pos[3] += config.arm_start_position
        self.goal_pos[4] += config.arm_min_workspace

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

        if Control._en_servo:
            self.motor[config.END_EFFECTOR_ID] = driver.DriverServo(config.END_EFFECTOR_ID)
            self.motor_group1.add_driver(self.motor[config.END_EFFECTOR_ID])

        self.delay_time = datetime.datetime(1970,1,1)
        self.is_update = False
        self.thread = None

        self.start()
        time.sleep(3)

    def plane_move(self, x_pos, y_pos, z_pos, speed = 0):
        rad = (self.position[3]) * np.pi / 180
        x = x_pos * np.cos(rad) - z_pos * np.sin(rad)
        z = x_pos * np.sin(rad) + z_pos * np.cos(rad)

        return self.move(x, y_pos, z, speed)

    def move(self, x_pos, y_pos, z_pos, speed = 0):
        return self.move_to(self.position[4] + x_pos, self.position[1] + y_pos, self.position[5] + z_pos)

    def move_to(self, x_pos, y_pos, z_pos, speed = 0): # pos in mm, velo in mm/s
        # check input
        workspace_x = config.offset_x_min + config.workspace_x + config.offset_x_max
        x_pos = min(max(x_pos, 0), workspace_x)
        y_pos = min(max(y_pos, 0), config.workspace_y)
        z_pos = min(max(z_pos, 0), config.workspace_z)

        end_pos = np.array([x_pos, z_pos], dtype=np.float64)
        middle_pos = np.array([self.position[0], self.position[2]], dtype=np.float64)

        x, z, rad, a = inverse_kinematics(middle_pos, end_pos)
        velo = list(config.default_spd)
        pos = [x, y_pos, z, rad, a]

        self.set_all_position(pos, velo)

    def cut_mango(self, is_cut):
        servo = self.motor[config.END_EFFECTOR_ID]
        
        servo.enable(1)
        servo.set(config.SERVO_CUTTER, config.servo_cutter_cut if is_cut is True else config.servo_cutter_open)

    def drop_mango(self, is_drop):
        servo = self.motor[config.END_EFFECTOR_ID]
        
        servo.enable(1)
        servo.set(config.SERVO_DOOR, config.servo_door_open if is_cut is True else config.servo_door_close)
    
    def stop_move(self):
        for i, item in enumerate(self.MOTOR_GROUP):
            for mtd in item:
                self.motor[mtd].set_goal(self.motor[item[0]].get_curr()[0], 0)

    def is_moving(self): # 
        if not self.is_update: return True

        if self.motor_group1.is_moving():
            return True

        for id in self.motor:
            if self.motor[id].is_moving():
                return True
        return False

    def get_pos(self):
        return np.array([self.position[0], self.position[1], self.position[2]], dtype=np.float64)

    def get_pos_arm(self):
        return np.array([self.position[5], self.position[1], self.position[6]], dtype=np.float64)
    
    def get_depth(self):
        return self.servo.get_range() if Control._en_servo else 0

    def set_all_position(self, pos, velo): # x, y, z, deg, arm
        for i in range(len(config.MOTOR_GROUP)):
            self.goal_pos[i] = pos[i]
            for id in config.MOTOR_GROUP[i]:
                self.motor[id].enable(1)
                if i == 0:
                    self.motor[id].set_goal(pos[i] - config.offset_x_min, velo[i])
                elif i == 3:
                    self.motor[id].set_goal(plan_turn(self.position[i] - config.arm_start_position, pos[i] - config.arm_start_position), velo[i])
                elif i == 4:
                    self.motor[id].set_goal(pos[i] - config.arm_min_workspace, velo[i])
                else:
                    self.motor[id].set_goal(pos[i], velo[i])
        self.is_update = False

    def set_position(self, motor_id, pulse, velo, error_wait = 0):
        for i in range(len(config.MOTOR_GROUP)):
            ck = 0
            for id in config.MOTOR_GROUP[i]:
                if id == motor_id:
                    self.goal_pos[i] = pulse
                    ck = 1
                    break
            if ck:
                for id in config.MOTOR_GROUP[i]:
                    self.motor[id].enable(1)
                    if i == 3:
                        pos = plan_turn(self.position[i] - config.arm_start_position, pulse - config.arm_start_position)
                        self.motor[id].set_goal(pulse, velo, False)
                    else:
                        self.motor[id].set_goal(pulse, velo, False)
        if error_wait:
            while abs(self.motor[motor_id].get_curr()[0] - pulse) >= error_wait:
                time.sleep(0.5)
        self.is_update = False

    def set_pulse(self, motor_id, pulse, velo, error_wait = 0):
        for i in range(len(config.MOTOR_GROUP)):
            ck = 0
            for id in config.MOTOR_GROUP[i]:
                if id == motor_id:
                    self.goal_pos[i] = pulse / self.motor[motor_id].ppmm
                    ck = 1
                    break
            if ck:
                for id in config.MOTOR_GROUP[i]:
                    self.motor[id].enable(1)
                    if i == 3:
                        pos = pulse / self.motor[motor_id].ppmm
                        # pos = plan_turn(self.position[i] - config.arm_start_position, pos - config.arm_start_position)
                        self.motor[id].set_goal(pos, velo, False)
                    else:
                        self.motor[id].set_goal(pulse, velo, True)
        if error_wait:
            while abs(self.motor[motor_id].get_curr()[0] * self.motor[motor_id].ppmm - pulse) >= error_wait:
                time.sleep(0.5)
        self.is_update = False

    def _update(self):
        result = {}

        url = "{}/get".format(config.url)
        data = {"token": generate_otp()}
        
        self.motor_group1.get()
        tmp = None
        try:
            tmp = requests.post(url, json=data).json()
        except Exception as e:
            print ("Motor Control", e)
            return 

        for i in range(len(config.MOTOR_GROUP)):
            self.position[i], self.velocity[i] = self.motor[config.MOTOR_GROUP[i][0]].get_curr()

        self.position[0] += config.offset_x_min
        self.position[3] += config.arm_start_position
        self.position[4] += config.arm_min_workspace
        self.position[5] = self.position[0] + self.position[4] * np.cos(self.position[3] * np.pi / 180) # arm x
        self.position[6] = self.position[2] + self.position[4] * np.sin(self.position[3] * np.pi / 180) # arm z

        self.is_update = True

    def _send_position(self):
        url = "{}/set".format(config.url)
        data = {"token": generate_otp()}

        result = None
        self.motor_group1.send()
        try:
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
        if self.thread == None:
            self.thread = Thread(target=self.loop)
            self.thread.start()

        return self

    def stop(self):
        self._is_running = False
        self.thread = None

    def __str__(self):
        txt = '\nnow position -> x: {}, y:{}, z:{}, deg:{}, arm:{}\n'.format(self.position[0], self.position[1], self.position[2], self.position[3], self.position[4])
        txt += 'goal position -> x: {}, y:{}, z:{}, deg:{}, arm:{}\n'.format(self.goal_pos[0], self.goal_pos[1], self.goal_pos[2], self.goal_pos[3], self.goal_pos[4])
        return txt

