import visual_servo
import driver,time
from mango_detection import mango_detector as md
from motion_control import Planner
import tensorflow as tf
import config
import cv2

def turn_arm_drop_mango(color = 1):
    if color not in [1, 2]:
        color = 1
    #print (config.basket[color])
    deg_go = config.basket[color] - planner.get_arm_deg()
    forward = config.arm_min_workspace - planner.get_arm_length()
    planner.move_single_cam(0, 0, forward, deg_go, config.default_speed)
    
    return True if not planner.is_moving(4) else False


if __name__ == '__main__':
    planner = Planner.getInstance()
    mango_model = md.load_model('mango_detection/model/', 'mango_2.1.pb')
    detection_graph, tensor_dict, image_tensor, category_index = mango_model
    cam = driver.DriverCamera(config.CAMERA_END_EFFECTOR, 2)
    #cam = cv2.VideoCapture(0)
    
    time.sleep(1)
    color = 1
    with detection_graph.as_default():
        with tf.Session() as sess:
            vs = visual_servo.VisualServo(True)
            vs.set_detector(sess, mango_model)
            vs.set_camera(cam)
            try:
                cam.start()
                vs.start()
            finally:
                color = vs.get_color()
                vs.stop()
                
        print('Visual Servo, Stop')
        print('Lift up')
#         planner.move_single_cam(0, 70, 0, 0, config.default_speed) # test z
#         time.sleep(100 / config.default_speed)
#         planner.move_single_cam(0, 0, 70, 0, config.default_speed) # test z
#         time.sleep(70 / config.default_speed)
        
        
        
#         for i in range(config.cut_count):
#             planner.cut_mango(True)
#             time.sleep(1)
#             planner.cut_mango(False)
#             time.sleep(1)
        
#         state = 0
#         while True:
#             if state == 0:
#                 if turn_arm_drop_mango(color):
#                     state = 1
#             elif state == 1:
#                 cur_pos = planner.get_pos()
#                 x_pos = config.middle_position[color]
#                 #print (cur_pos[0], x_pos)
#                 planner.move_to_stereo_cam(x_pos, cur_pos[1], cur_pos[2], config.default_speed)
                
#                 time.sleep(1)
#                 if not planner.is_moving(0):
#                     break
            
            #print (state)
        
                        
        planner.drop_mango(True)
        time.sleep(1)
        planner.drop_mango(False)
        time.sleep(1)
        
        cam.stop()
        Planner.getInstance().stop()