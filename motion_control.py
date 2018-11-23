import pprint

import numpy as np
import datetime, time, sys
from threading import Thread, Lock

from Driver.motor import DriverMotor, DriverServo

import config
import sync_group

def inverse_kinematics(middle_pos, end_pos): # pos np.array([x, z])
    # print (middle_pos, end_pos)
    ck = 0
    while True:
        dif_pos = end_pos - middle_pos
        length = np.linalg.norm(dif_pos)
        print ("Diff z", dif_pos[1])

        if dif_pos[1] > config.arm_max_workspace or ck > 10:
            return [0, 0, 0, 0]
        
        # radius = max(dif_pos[1], config.arm_min_workspace) if length > config.arm_max_workspace else max(length, config.arm_min_workspace)
        radius = max(length, config.arm_min_workspace)
        rad = np.arcsin(dif_pos[1] / radius)
        rad = rad if dif_pos[0] >= 0 else (np.pi / 2 - rad) + np.pi / 2
        # rad = np.arctan(dif_pos[1] / dif_pos[0]) if dif_pos[0] != 0 else (np.pi / 2 if dif_pos[1] > 0 else -np.pi / 2)
        # print (dif_pos, np.sin(rad) * radius)

        x = end_pos[0] - radius * np.cos(rad) # middle
        z = end_pos[1] - radius * np.sin(rad) # middle

        x -= config.offset_x_min
        if abs(x) < 1e-1: x = 0
        if abs(z) < 1e-1: z = 0
        if (0 <= x <= config.workspace_x): # and middle_pos[1] == z:
            break

        x_new = x - max(0, min(config.workspace_x, x))
        z_new = 0 # z - max(0, min(config.workspace_z, z))
        middle_pos -= np.array([x_new, z_new], dtype=np.float64)
        ck += 1
        # print (rad * 180 / np.pi, x, x_new, z_new, middle_pos, dif_pos)

    return [x + config.offset_x_min, z, rad * 180 / np.pi, radius]

def plan_turn(cur, goto):
    # [-360, 360]

    # a = ((cur % 360) + 360) % 360
    b = ((goto % 360) + 360) % 360

    # c = ((goto - cur) + 360) % 360
    # d = ((cur - goto) + 360) % 360

    if b > 180:
        return (b - 360)
    return b
    # # print (cur, goto, c, d)

    # return cur + c if c < d else cur - d

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

        self.lock = Lock()
        self.queue = Queue(maxsize=max(queueSize, 32))
        self.control = Control()
        self.cmd = [self.control.move, self.control.move_to, self.control.plane_move, self.control.set_pulse, self.control.set_position, self.control.cut_mango]

        self.interrupt = False

        self.thread = None
        self.start()

    def is_empty(self):
        return self.queue.empty()

    def is_moving(self):
        return self.control.is_moving()

    def get_control(self):
        self.interrupt = True
        return self.control

    def add(self, cmd, args, wait = True):
        if not self.queue.full():
            return self.queue.put((cmd, tuple(args), wait))
        return False

    def start(self):
        if self.thread == None:
            self._is_running = 1
            self.thread = Thread(target=self.loop)
            self.thread.start()

    def loop(self):
        def check_exit():
            with self.lock:
                return not self._is_running
            return False

        while True:
            if check_exit(): return

            if self.interrupt:
                self.wait()
                self.interrupt = False

            try:
                block = self.queue.get(timeout=1)
            except Exception as e:
                pass
                # print ("Planer -", e) # timeout
            else:
                if block[0] < len(self.cmd):
                    self.cmd[block[0]](*block[1])

                    if block[2]:
                        time.sleep(2)
                        while self.is_moving():
                            if check_exit(): return
                            time.sleep(0.7)

    def wait(self, timeout = 0): # ms
        start = datetime.datetime.now()
        i = 1
        if self.interrupt:
            while self.is_moving():
                if timeout > 0 and ((datetime.datetime.now() - start).microseconds / 1000) > timeout:
                    sys.stdout.write("\n")
                    return 0
                sys.stdout.write("\rPlanner - Waiting " + "." * i)
                i += 1
                time.sleep(0.7)
        else:
            while not self.is_empty() or self.is_moving():
                if timeout > 0 and ((datetime.datetime.now() - start).microseconds / 1000) > timeout:
                    sys.stdout.write("\n")
                    return 0
                sys.stdout.write("\rPlanner - Waiting " + "." * i)
                i += 1
                time.sleep(0.7)

        sys.stdout.write("\n")
        return 1

    def stop(self):
        self.control.stop()
        with self.lock:
            self._is_running = 0
        
        self.thread = None

    def __str__(self):
        txt = ''
        while not self.is_empty():
            a = self.queue.get()
            txt += str(a) + '\n'
        return txt

class Control:
    __instance = None
    _en_servo = True

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
        self.motor[config.BASE_MOTOR_ID_L] = DriverMotor(config.BASE_MOTOR_ID_L, ppmm=config.encoder_pulse_base_l)
        self.motor[config.BASE_MOTOR_ID_R] = DriverMotor(config.BASE_MOTOR_ID_R, ppmm=config.encoder_pulse_base_r)
        self.motor[config.LIFT_MOTOR_ID_L] = DriverMotor(config.LIFT_MOTOR_ID_L, ppmm=config.encoder_pulse_lift_l)
        self.motor[config.LIFT_MOTOR_ID_R] = DriverMotor(config.LIFT_MOTOR_ID_R, ppmm=config.encoder_pulse_lift_r)
        self.motor[config.MIDDLE_MOTOR_ID] = DriverMotor(config.MIDDLE_MOTOR_ID, ppmm=config.encoder_pulse_middle)
        self.motor[config.TURRET_MOTOR_ID] = DriverMotor(config.TURRET_MOTOR_ID, ppmm=config.encoder_pulse_turret)
        self.motor[config.FORWARD_MOTOR_ID] = DriverMotor(config.FORWARD_MOTOR_ID, ppmm=config.encoder_pulse_forward)

        self.motor_group1 = sync_group.Group('overall')
        for motor_id in self.motor.keys(): self.motor_group1.add_driver(self.motor[motor_id])
        for motor_id in self.motor.keys(): self.motor[motor_id].set_moving_threshould(config._moving_threshold[motor_id])

        if Control._en_servo:
            self.motor[config.END_EFFECTOR_ID] = DriverServo(config.END_EFFECTOR_ID)
            self.motor_group1.add_driver(self.motor[config.END_EFFECTOR_ID])
            self.cut_mango(False)

        self.delay_time = datetime.datetime(1970,1,1)
        self.is_update = False
        self.thread = None

        self.start()
        time.sleep(3)

    def plane_move(self, x_pos, y_pos, z_pos, speed = 0):
        rad = (self.position[3]) * np.pi / 180
        # rad -= np.pi / 2
        # x = x_pos * np.cos(rad) - z_pos * np.sin(rad)
        # z = x_pos * np.sin(rad) + z_pos * np.cos(rad)
        x = x_pos * np.sin(rad) + z_pos * np.cos(rad)
        z = -x_pos * np.cos(rad) + z_pos * np.sin(rad)

        return self.move(x, y_pos, z, speed)

    def move(self, x_pos, y_pos, z_pos, speed = 0):
        return self.move_to(self.position[5] + x_pos, self.position[1] + y_pos, self.position[6] + z_pos)

    def move_to(self, x_pos, y_pos, z_pos, speed = 0): # pos in mm, velo in mm/s
        # check input
        workspace_x = config.offset_x_min + config.workspace_x + config.offset_x_max
        x_pos = min(max(x_pos, 0), workspace_x)
        y_pos = min(max(y_pos, 0), config.workspace_y)
        z_pos = min(max(z_pos, 0), config.workspace_z)

        end_pos = np.array([x_pos, z_pos], dtype=np.float64)
        middle_pos = np.array([self.position[0], self.position[2]], dtype=np.float64)

        klist = inverse_kinematics(middle_pos, end_pos)
        print (klist)
        if klist == [0, 0, 0, 0]:
            return False

        x, z, rad, a = klist
        velo = list(config.default_spd)
        pos = [x, y_pos, z, rad, a]


        # if self.goal[4] == config.arm_max_workspace and a == config.arm_max_workspace:
        #     return False

        self.set_all_position(pos, velo)
        return True

    def cut_mango(self, is_cut):
        servo = self.motor[config.END_EFFECTOR_ID]
        
        servo.enable(config.SERVO_CUTTER, 1)
        servo.set(config.SERVO_CUTTER, config.servo_cutter_cut if is_cut is True else config.servo_cutter_open)
    
    def stop_move(self):
        for i, item in enumerate(self.MOTOR_GROUP):
            for mtd in item:
                self.motor[mtd].set_goal(self.motor[item[0]].get_curr()[0], 0)

    def is_moving(self): # 
        if not self.is_update: return True

        # if self.motor_group1.is_moving():
        #     return True

        for id in self.motor:
            if isinstance(self.motor[id], DriverMotor):
                if self.motor[id].is_moving():
                    return True
        return False

    def get_pos(self):
        return np.array([self.position[0], self.position[1], self.position[2]], dtype=np.float64)

    def get_pos_arm(self):
        return np.array([self.position[5], self.position[1], self.position[6]], dtype=np.float64)

    def get_all_pos(self):
        return np.array(self.position)
    
    def get_depth(self):
        return self.motor[config.END_EFFECTOR_ID].get_range() if Control._en_servo else 0

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
                    ck = 1
                    break
            if ck:
                for id in config.MOTOR_GROUP[i]:
                    self.motor[id].enable(1)
                    if i == 3:
                        pulse = plan_turn(self.position[i] - config.arm_start_position, pulse - config.arm_start_position)
                        # pass
                    self.goal_pos[i] = pulse
                    self.motor[id].set_goal(pulse, velo, False)
                break
        if error_wait:
            while abs(self.motor[motor_id].get_curr()[0] - pulse) >= error_wait:
                time.sleep(0.5)
        self.is_update = False

    def set_pulse(self, motor_id, pulse, velo, error_wait = 0):
        for i in range(len(config.MOTOR_GROUP)):
            ck = 0
            for id in config.MOTOR_GROUP[i]:
                if id == motor_id:
                    ck = 1
                    break
            if ck:
                for id in config.MOTOR_GROUP[i]:
                    self.motor[id].enable(1)
                    if i == 3:
                        pulse = pulse / self.motor[motor_id].ppmm
                        pulse = plan_turn(self.position[i] - config.arm_start_position, pulse - config.arm_start_position)
                    self.goal_pos[i] = pulse
                    self.motor[id].set_goal(pulse, velo, True)
                break
        if error_wait:
            while abs(self.motor[motor_id].get_curr()[0] * self.motor[motor_id].ppmm - pulse) >= error_wait:
                time.sleep(0.5)
        self.is_update = False

    def _update(self):
        result = {}

        self.motor_group1.get()

        for i in range(len(config.MOTOR_GROUP)):
            self.position[i], self.velocity[i] = self.motor[config.MOTOR_GROUP[i][0]].get_curr()

        self.position[0] += config.offset_x_min
        self.position[3] += config.arm_start_position
        self.position[4] += config.arm_min_workspace
        self.position[5] = self.position[0] + self.position[4] * np.cos(self.position[3] * np.pi / 180) # arm x
        self.position[6] = self.position[2] + self.position[4] * np.sin(self.position[3] * np.pi / 180) # arm z

        self.is_update = True
    
    def loop(self):
        i = 0
        while True:
            with self.lock:
                if not self._is_running:
                    return

            time_diff = (datetime.datetime.now() - self.delay_time).microseconds / 1000
            if time_diff < config.planner_update_time: # ms
                sleep_more = config.planner_update_time - time_diff
                time.sleep(sleep_more / 1000)
                # print ("time", sleep_more)
                            
            if i % 2 == 0:
                self._update()
                self.motor_group1.update()
            else:                
                self.motor_group1.send()

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
        with self.lock:
            self._is_running = False
        self.thread = None

    def __str__(self):
        txt = '\nnow position -> x: {}, y:{}, z:{}, deg:{}, arm:{}\n'.format(self.position[0], self.position[1], self.position[2], self.position[3], self.position[4])
        txt += 'goal position -> x: {}, y:{}, z:{}, deg:{}, arm:{}\n'.format(self.goal_pos[0], self.goal_pos[1], self.goal_pos[2], self.goal_pos[3], self.goal_pos[4])
        return txt

