import cv2
import numpy as np
import datetime, time

from motion_control import Planner
from mango_detection import yolo

import config

class VisualServo:
    """docstring for VisualServo"""
    def __init__(self, net, cam, show_image = False):
        self.show_image = show_image
        self.color = -1

        self.net = net
        self.camera = cam
        
        self.tracker = None
        self.boxes = None

    def ob_tracking(self, tracker_name):
        OPENCV_OBJECT_TRACKERS = {
            "csrt": cv2.TrackerCSRT_create,
            "kcf": cv2.TrackerKCF_create,
            "boosting": cv2.TrackerBoosting_create,
            "mil": cv2.TrackerMIL_create,
            "tld": cv2.TrackerTLD_create,
            "medianflow": cv2.TrackerMedianFlow_create,
            "mosse": cv2.TrackerMOSSE_create
        }
        
        return OPENCV_OBJECT_TRACKERS[tracker_name]()


    def get_color(self):
        return self.color

    def loop(self):
        planner = Planner.getInstance()
        #time.sleep(1)
        
        error_count = 0
        track_count = 0
        z_length_sum = 0
        use_tracker = False
        result = None
        while self.__is_running and error_count < config.visual_failed_count:
            if not planner.is_empty() and planner.is_moving():
                time.sleep(0.5)
                continue

            # depth = planner.get_depth()
            # if depth <= config.cut_length:
            #     break
            
            # move servo
            frame = self.camera.read()
            if not frame[0]:
                print ("Camera not ready.")
                error_count += 1
                planner.stop_move()
                continue

            frame_height, frame_width, ch = frame[1].shape
            frame_center = (int(frame_width/2), int(frame_height/2) + config.camera_K_offset_x)
            
            if self.tracker and self.boxes:
                (use_tracker, box) = self.tracker.update(frame[1])
                result['boxes'] = box
                result['center'] = yolo.get_center(box)
                track_count += 1
            
            if not use_tracker:
                result = yolo.detect(frame[1], self.net, 0.85, self.show_image)
                self.tracker = None
                
            if self.show_image:
                if use_tracker:
                    bbox = result['boxes']
                    p1 = (int(bbox[0]), int(bbox[1]))
                    p2 = (int(bbox[0] + bbox[2]), int(bbox[1] + bbox[3]))
                    cv2.rectangle(frame[1], p1, p2, (255,0,0), 2, 1)
                    result['image'] = frame[1]
                else:
                    if 'center' in result.keys():
                        cv2.line(result['image'], frame_center, result['center'], (255, 127, 0), 8)
                cv2.imshow("Visual Servo", result['image'])
                cv2.waitKey(1)

            if not 'boxes' in result.keys():
                print ("Not Found mango")
                error_count += 1
                planner.stop_move()                
                continue    
                
            diff_x = result['center'][0] - frame_center[0] # pixel
            diff_y = frame_center[1] - result['center'][1] # pixel
            diff_z = 4

            z_length_sum += diff_z
            zm = 1000
            zf = zm  + z_length_sum
            
            diff_x = min(diff_x, config.visual_max_move) * (zf / (zm + z_length_sum)) * 2.5
            diff_y = min(diff_y, config.visual_max_move) * (zf / (zm + z_length_sum))
            diff_z = min(diff_z, config.visual_max_move) * (zf / (zm + z_length_sum))
#             diff_y=0
            planner.move(diff_x, diff_y, diff_z, 0, config.default_speed)
            # while planner.is_moving(): time.sleep(0.1)
            error_count = 0
        
            self.color = result['class']
            self.boxes = result['boxes']
            
            if self.tracker == None:
                self.tracker = self.ob_tracking(config.object_tracker)
                self.tracker.init(frame[1], result['boxes'])
                (use_tracker, box) = self.tracker.update(frame[1])
                
            if track_count >= config.object_track_count and config.object_track_count != 0:
                self.tracker = None
                track_count = 0
                use_tracker = False
            
#             time.sleep(math.sqrt(diff_x**2 + diff_y**2 + diff_z**2) / config.default_speed)

            #print ("boxes", result['boxes'], 'center', result['center'])
            print ("move distance x:{}, y:{}, z:{}, d:{}".format(diff_x, diff_y, diff_z, depth))
        
        return True if error_count < config.visual_failed_count else False

    def start(self):
        self.__is_running = True
        return self.run()

    def run(self):
        res = self.loop()
        self.__is_running = False
        return res

    def stop(self):
        if self.__is_running:
            cv2.destroyAllWindows()
        self.__is_running = False
