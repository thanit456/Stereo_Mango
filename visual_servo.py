import cv2
import config
import math
import driver
import datetime, time
from motion_control import Planner
from mango_detection import mango_detector as md
import numpy as np

class VisualServo:
    """docstring for VisualServo"""
    def __init__(self, show_image = False):
        self.show_image = show_image
        self.color = -1

    def set_detector(self, session, model):
        self.detector = {
            'tf_session': session,
            'model': model,
        }

    def set_camera(self, camera):
        self.camera = camera

#     def rorate_about_point(self, origin, point, deg): # dict of x, y, z | deg in radian
#         x_diff = origin[0] - point[0]
#         z_diff = origin[2] - point[2]
#         return (origin[0] + (np.cos(deg) * x_diff - np.sin(deg) * z_diff),
#                 origin[1],
#                 origin[2] + (np.sin(deg) * x_diff + np.cos(deg) * z_diff))

    def get_color(self):
        return self.color

    def loop(self):
        planner = Planner.getInstance()
        time.sleep(1)
        error_count = 0
        z_length_sum = 0
        time_now = datetime.datetime.now()
        while self.__is_running and error_count < config.visual_failed_count:
            depth = planner.get_depth()
            if depth <= config.cut_length:
                break

            # move servo
            frame = self.camera.read()
            if not frame[0]:
                print ("Camera not ready.")
                error_count += 1
                planner.stop_move()
                continue

            frame_height, frame_width, ch = frame[1].shape
            frame_center = (int(frame_width/2), int(frame_height/2) + config.camera_K_offset_x)

            result = md.detect(frame[1], self.detector['model'][1:], self.detector['tf_session'], 0.85, self.show_image)
            if self.show_image:
                if 'center' in result.keys():
                    cv2.line(result['image'], frame_center, result['center'], (255, 127, 0), 8)
                cv2.imshow("Visual Servo", result['image'])
                cv2.waitKey(1)

            if not 'boxes' in result.keys():
                print ("Not Found mango")
                error_count += 1
                planner.stop_move()                
                continue

            diff_x = (result['center'][0] - frame_center[0]) # pixel
            diff_y = frame_center[1] - result['center'][1] # pixel
            diff_z = np.sqrt(diff_x**2 + diff_y**2) / 2

            z_length_sum += diff_z
            zm = 1000
            zf = zm  + z_length_sum
            
            diff_x = min(diff_x, config.visual_max_move) * (zf / (zm + z_length_sum)) * 2.5
            diff_y = min(diff_y, config.visual_max_move) * (zf / (zm + z_length_sum))
            diff_z = min(diff_z, config.visual_max_move) * (zf / (zm + z_length_sum))
#             diff_y=0
            planner.move_single_cam(diff_x, diff_y, diff_z, 0, config.default_speed)
            error_count = 0
            self.color = result['class']
            
            now = datetime.datetime.now()
            time.sleep(((now - time_now).microseconds / 1000000) + 0.2)
            time_now = now

            #print ("boxes", result['boxes'], 'center', result['center'])
            print ("move distance x:{}, y:{}, z:{}, d:{}".format(diff_x, diff_y, diff_z, depth))
        
        return True if error_count < config.visual_failed_count else False

    def start(self):
        self.__is_running = True
        self.run()

    def run(self):
        self.loop()
        self.__is_running = False

    def stop(self):
        if self.__is_running:
            cv2.destroyAllWindows()
        self.__is_running = False
