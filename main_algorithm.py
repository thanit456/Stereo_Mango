#!/usr/bin/python3

import cv2
import config
import visual_servo
import tensorflow as tf
from motion_control import Planner
from mango_detection import mango_detector as mod
import driver
import math
import time

camera = {
    'device' : driver.DriverCamera(config.CAMERA_END_EFFECTOR),
    'focus_length' : config.camera_focus_length,
    'offset' : [0, 0, 0], # x, y, z
}

base_position = [
    (500, 90),
    (1500, 270),
    (2500, 90),
    (3500, 270),
    (4500, 90),
    (5500, 270),
]

planner = Planner.getInstance()
mango_model = mod.load_model('mango_detection/model/', 'mango_1.1.pb')

def get_distance(pos1, pos2):
    def diff(a1, a2):
        return np.abs(a1 - a2)
    return math.sqrt(diff(pos1[0], pos2[0])**2 + diff(pos1[1], pos2[1])**2 + diff(pos1[2], pos2[2]))

def state_a1():
    cur_pos = planner.get_pos()
    
    x_diff = (config.workspace_x / 2) - cur_pos[0]
    y_diff = config.workspace_y - cur_pos[1]
      
    planner.force(config.LIFT_MOTOR_ID_L, 64000, 1)
    planner.force(config.LIFT_MOTOR_ID_R, 64000, 1, 300)
    return True
#     planner.move_stereo_cam(x_diff, y_diff, 0, config.default_speed)
#     return not planner.is_moving() and (math.sqrt(x_diff**2 + y_diff**2) <= config.accept_move)

deg_max_score = (0, 0) # deg, score
def state_find_deg_maxscore(tf_session, mango_model, camera):
    global deg_max_score
    frame = camera['device'].read()
    if not frame[0]:
        print ("Camera Failed")
        return False
        
    frame = frame[1]
    image_height, image_width, ch = frame.shape
        
    result = mod.detect(frame, mango_model, sess, 0.7, False)
    if 'center' in result.keys():
        if result['score'] > deg_max_score[1]:
            deg_max_score = (planner.get_arm_deg(), result['score'])
        
            deg_go = config.deg_find_score if result['center'][0] > (image_width / 2) else -1 * config.deg_find_score
            planner.move_single_cam(0, 0, 0, deg_go, config.default_speed)
            return False
        
        deg_go =  deg_max_score[0] - planner.get_arm_deg()
        planner.move_single_cam(0, 0, 0, deg_go, config.default_speed)
        time.sleep(2)
        return True
    return False

def turn_arm_drop_mango(color = 1):
    if color not in [1, 2]:
        color = 1
    
#     deg_go = config.basket[color] - planner.get_arm_deg()
#     forward = planner.get_arm_length()
#     planner.move_single_cam(0, 0, -1 * forward, deg_go, config.default_speed)
    
#     return True if planner.is_moving() and (deg_go * forward <= config.accept_move) else False
    
    if color == 1:
        planner.force(config.FORWARD_MOTOR_ID, 0, 500, 10000)
        print('A2')
        planner.force(config.TURRET_MOTOR_ID, 0, 500, 100)
        print('A3')
        planner.force(config.MIDDLE_MOTOR_ID, 4400000, 500, 10000)
    else:
        print('B1')
        planner.force(config.FORWARD_MOTOR_ID, 0, 500, 10000)
        print('B2')
        planner.force(config.TURRET_MOTOR_ID, 4050, 500, 100)
        print('B3')
        planner.force(config.MIDDLE_MOTOR_ID, 800000, 500, 10000)
    #drop_mango()
                     
    return True

def drop_mango():
    print('DROP')
    planner.cut_mango(False)
    planner.drop_mango(True)
    time.sleep(1.5)
    planner.drop_mango(False)
    time.sleep(1.5)
    
    return True

def cut_mango():
    for i in range(config.cut_count):
        planner.cut_mango(True)
        time.sleep(1)
        
        if i+1 < config.cut_count:
            planner.cut_mango(False)
            time.sleep(1)
    planner.cut_mango(True)
    
    return True

if __name__ == '__main__':
    detection_graph, tensor_dict, image_tensor, category_index = mango_model    
    current_pos = [0, 0, 0]
    mango_color = -1
    
    count_j = 10 # x
    count_i = 10 # y
    
    view_x = config.workspace_x / count_j
    view_y = config.workspace_y / count_i
    
    with detection_graph.as_default():
        with tf.Session() as sess:
            camera['device'].start()
#             for index, item in enumerate(base_position):
#                 while not state_a1():
#                     time.sleep(0.1)
            if 1:
                for i in range(2, count_i - 2):
                    for j in range(2, count_j - 1):
                        print('Coord',i,j)
                        state = 0
                        planner.force(config.FORWARD_MOTOR_ID, 0, 500, 10000)
                        planner.force(config.TURRET_MOTOR_ID, 2000, 500, 100)
                        while True:
                            print(state)
                            if state == 0:
                                x = (view_x / 2) + (view_x * j)
                                if i % 2 == 1:
                                    x = config.workspace_x - x
                                y = ((view_y / 2) + (view_y * i))
                                print ("POs x, y: ", x, y)
                                planner.force(config.MIDDLE_MOTOR_ID, x * config.encoder_pulse_middle, 1000)
                                planner.force(config.LIFT_MOTOR_ID_L, y * config.encoder_pulse_lift_l, 2)
                                planner.force(config.LIFT_MOTOR_ID_R, y * config.encoder_pulse_lift_l, 2, 300)
                                planner.force(config.MIDDLE_MOTOR_ID, x * config.encoder_pulse_middle, 1000, 1000)
                                state = 1
                                
                            elif state == 1:
                                frame = camera['device'].read()
                                if not frame[0]:
                                    print ("Camera Fail")
                                    continue
                                result = mod.detect(frame[1], mango_model[1:], sess, 0.9)
                                if 'center' in result.keys():
                                    state = 2
                                else:
                                    print ("Not Found Mango")
                                    break
                            elif state == 2:
                                mango_color = -1
                                if 1: # state_find_deg_maxscore(sess, mango_model[1:], camera):
                                    vs = visual_servo.VisualServo(True)
                                    vs.set_detector(sess, mango_model)
                                    vs.set_camera(camera['device'])
                                    try:
                                        vs.start()
                                    finally:
                                        vs.stop()
                                    
                                    mango_color = vs.get_color()
                                    planner.move_single_cam(0, 50, 0, 0, config.default_speed) # test z
                                    time.sleep(100 / config.default_speed)
                                    planner.move_single_cam(0, 0, 90, 0, config.default_speed) # test z
                                    time.sleep(70 / config.default_speed)
                                    
#                                     if planner.get_depth() > 500:
#                                         state = 0
                                    state = 3
                            
                            elif state == 3:
                                cut_mango()
                                state = 4
                            
                            elif state == 4:
                                if turn_arm_drop_mango(mango_color):
                                    state = 6
                            
                            elif state == 5:
                                cur_pos = planner.get_pos()
                                x_pos = config.middle_position[mango_color] - cur_pos[0]
                                
                                planner.move_stereo_cam(x_pos, 0, 0, config.default_speed)
                                
                                if planner.is_moving() and (x_pos <= config.accept_move):
                                    state = 6
                            
                            elif state == 6:
                                drop_mango()
                                planner.force(config.TURRET_MOTOR_ID, 2000, 500, 100)
                                state = 0
                                
                            elif state == 7:
                                #planner.turn_to_arm(item[1])
                                if np.abs(planner.get_arm_deg() - it) <= 2:
                                    state = 1
                            
                            print (state)
                
    camera['device'].stop()
    planner.stop()

