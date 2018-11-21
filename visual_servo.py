import cv2
import numpy as np
import datetime, time

import config
from Driver.stereo import DriverStereo
from motion_control import Planner
from mango_detection import yolo

def true_depth(pos, z):
    return pos * z / 750

def makeInt(box):
    return [int(i) for i in box]

def get_center(rect, num = 0):
    x, y, w, h = rect
    a = [int(x + w/2), int(y + h / 2)]
    if num:
        return np.array(a, dtype=np.float64)
    else:
        return a


def get_pos_from_stereo(frame, lbox, camera, scale=None):
    scale = 0.9
    if scale is not None:
        cen = lbox[0] + lbox[2]/2, lbox[1] + lbox[3]/2
        w, h = lbox[2:4]
        w *= scale
        h *= scale
        lbox = makeInt((cen[0] - w/2, cen[1] - h/2, w, h))
    else:
        lbox = makeInt(lbox)
    rbox, disparity = DriverStereo.find_lower_mean_r(frame[1], frame[2], lbox)
    # disparity = DriverStereo.find_disparity_from_box(lbox, rbox, frame[1].shape[:2])
    diff_z = camera.get_depth(disparity)

    # print (disparity, diff_z)
    
    clbox = get_center(lbox)
    crbox = get_center(rbox)

    # clbox[0] = clbox[0] * diff_z / config.CAMERA_FOCAL_LEFT[0] # mm
    # clbox[1] = clbox[1] * diff_z / config.CAMERA_FOCAL_LEFT[1] # mm
    # crbox[0] = crbox[0] * diff_z / config.CAMERA_FOCAL_RIGHT[0] # mm
    # crbox[1] = crbox[1] * diff_z / config.CAMERA_FOCAL_RIGHT[1] # mm

    return np.array([int((clbox[0] + crbox[0])/2), int((clbox[1] + crbox[1]) / 2), diff_z], dtype=np.float64), disparity
    # return np.array([int()])

class AvoidMango:
    """docstring for AvoidMango"""
    def __init__(self):
        self.list_mango = []

    def add(self, pos = None):
        # if pos == None:
        #     return
        self.list_mango.append(pos)

    def clear(self):
        self.list_mango.clear()

    def avoid_this(self, pos, avoid_range = 30): # pos is np.array, range has unit was millimeters;
        for a in self.list_mango:
            dif_pos = a - pos
            length = np.linalg.norm(dif_pos)
            if length < avoid_range:
                return True
        return False
        
class VisualServo:
    """docstring for VisualServo"""
    def __init__(self, net, cam, avoidMango, show_image = False):
        self.show_image = show_image
        self.color = -1

        self.net = net
        self.camera = cam
        self.avoidMango = avoidMango
        
        self.tracker = None
        self.boxes = None

        self.detect_pos = None
        self.last_z = None
        self.disparity = 0

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

    def loop(self, DEBUG = False):
        planner = Planner.getInstance()
        while not planner.is_empty() or planner.is_moving():
            time.sleep(0.5)
            continue
        
        error_count = 0
        track_count = 0
        low_pass_count = 0
        low_pass_pos = [0, 0, 0]
        z_length_sum = 0
        use_tracker = False
        result = dict()
        while True:
            if not self._is_running: return False
            if not DEBUG:
                if error_count > config.visual_failed_count: break

            cur_pos = planner.get_control().get_pos_arm()
            depth = planner.get_control().get_depth() if not DEBUG else 300
            if depth <= config.cut_length:
                break
            
            # move servo
            frame = self.camera.read()
            if not frame[0]:
                print ("Camera not ready.")
                error_count += 1
                continue

            if self.tracker and self.boxes:
                (use_tracker, box) = self.tracker.update(frame[1].copy())
                result['boxes'] = (box)
                track_count += 1
            
            if not use_tracker:
                results = yolo.detect(frame[1], self.net, 0.85, False, get_list=True)
                while 1:
                    (i, color) = yolo.find_max_scores(results[0], results[1], 0.85)
                    if i < 0:
                        result = {}
                        break

                    bounds = (results[1][i][2])
                    pos, self.disparity = get_pos_from_stereo(frame, bounds, self.camera)
                    pos[0] = true_depth(pos[0], pos[2]) + cur_pos[0]
                    pos[1] = true_depth(pos[1], pos[2]) + cur_pos[1]
                    self.detect_pos = pos
                    if not DEBUG:
                        pos[2] += cur_pos[2] + config.position_cam_from_end # x, y, z
                    
                    if not self.avoidMango.avoid_this(pos, 30):
                        result = {
                            'boxes': bounds,
                            'center': get_center(bounds, 1),
                            'score': results[1][i][1],
                            'class': color,
                        }
                        break
                    else:
                        results.pop(i)

                self.tracker = None

           
            if not 'boxes' in result.keys():
                print ("Not Found mango")
                error_count += 1
                if self.show_image:
                    cv2.imshow("Visual Servo", frame[1])
                    cv2.waitKey(1)
                while planner.is_moving():
                    time.sleep(0.5)
                continue
                
            result['center'], self.disparity = get_pos_from_stereo(frame, result['boxes'], self.camera, self.detect_pos[2] / self.last_z if use_tracker else None)
            diff_z = result['center'][2] if result['center'][2] < config.arm_max_workspace else depth
            self.last_z = diff_z

            if self.show_image:
                cv2.imshow("Visual Servo", self.makeRectangle(frame[1].copy(), frame[2], result['boxes'], diff_z))
                cv2.waitKey(1)

            self.color = result['class']
            self.boxes = result['boxes']
            
            if self.tracker == None:
                self.tracker = self.ob_tracking(config.object_tracker)
                self.tracker.init(frame[1], tuple(result['boxes']))
                (use_tracker, box) = self.tracker.update(frame[1])

            if track_count >= config.object_track_count and config.object_track_count != 0:
                self.tracker = None
                track_count = 0
                use_tracker = False
            
            fc = get_center([0, 0] + list(frame[1].shape[:2])[::-1])
            diff_y = (fc[1] - result['center'][1]) # mm
            diff_x = (result['center'][0] - fc[0]) # mm
            diff_y = true_depth(diff_y, diff_z)
            diff_x = true_depth(diff_x, diff_z)

            # print (fc[0], result['center'][0], fc[1], result['center'][1])
            print ("move distance x:{}, y:{}, z:{}, d:{}".format(diff_x, diff_y, diff_z, depth))
            alpha = 0.864
            low_pass_pos[0] = low_pass_pos[0] * alpha + diff_x * (1 - alpha)
            low_pass_pos[1] = low_pass_pos[1] * alpha + diff_y * (1 - alpha)
            low_pass_pos[2] = low_pass_pos[2] * alpha + diff_z * (1 - alpha)
            low_pass_count += 1
            if config.visual_low_pass_count < low_pass_count:
                low_pass_count = 0
                if not DEBUG and not planner.is_moving():
                    print ("low pass move distance x:{}, y:{}, z:{}, d:{}".format(low_pass_pos[0], low_pass_pos[1], low_pass_pos[2], depth))                    
                    # get current position
                    diff_z += config.position_cam_from_end # move z for length of end-effector to fruit
                    if not planner.get_control().plane_move(low_pass_pos[0], low_pass_pos[1], low_pass_pos[2] * 0.5a, 0):
                        pos = result['center']
                        pos[0] = true_depth(pos[0], pos[2]) + cur_pos[0]
                        pos[1] = true_depth(pos[1], pos[2]) + cur_pos[1]
                        pos[2] += cur_pos[2] + config.position_cam_from_end
                        self.avoidMango.add(pos)
                        print ("out of range.")
                        return False
                    print (planner.get_control())
                # while planner.is_moving(): time.sleep(0.1)
            error_count = 0

        return True if error_count < config.visual_failed_count else False

    def makeRectangle(self, frame, frame_alt, boxes, curr_z):
        cen = boxes[0] + boxes[2]/2, boxes[1] + boxes[3]/2
        w, h = boxes[2:4]
        scale = self.detect_pos[2] / curr_z
        w *= scale
        h *= scale
        #x, y, w, h = makeInt(boxes)
        x, y, w, h = makeInt((cen[0] - w/2, cen[1] - h/2, w, h))
        fh, fw = frame.shape[:2]
        p1 = (x, y)
        p2 = (x + w, y + h)
        # print (p1, p2)
        cv2.rectangle(frame, p1, p2, (255,0,0), 2, 1)
        pc = tuple(get_center(boxes))
        fc = tuple(get_center([0, 0, fw, fh]))
        cv2.line(frame, fc, pc, (255, 127, 0), 8)
        try:
            frame[y:y+h,x:x+w] = frame[y:y+h,x:x+w] // 2 + frame_alt[y:y+h,x-self.disparity:x+w-self.disparity] // 2
        except ValueError:
            pass
        return frame


    def start(self, DEBUG = False):
        self._is_running = True
        return self.run(DEBUG)

    def run(self, DEBUG):
        self.camera.start()
        res = self.loop(DEBUG)
        self.camera.stop()

        self._is_running = False
        return res

    def stop(self):
        if self._is_running:
            cv2.destroyAllWindows()
            self.camera.stop()
        self._is_running = False
