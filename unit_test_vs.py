import visual_servo
import driver,time
from mango_detection import mango_detector as md
from motion_control import Planner
import tensorflow as tf
import config
import cv2

if __name__ == '__main__':
    mango_model = md.load_model('mango_detection/model/', 'mango_2.0.pb')
    detection_graph, tensor_dict, image_tensor, category_index = mango_model
    cam = driver.DriverCamera(config.CAMERA_END_EFFECTOR, 2)
    #cam = cv2.VideoCapture(0)
    
    time.sleep(1)
    with detection_graph.as_default():
        with tf.Session() as sess:
            vs = visual_servo.VisualServo(True)
            vs.set_detector(sess, mango_model)
            vs.set_camera(cam)
            try:
                cam.start()
                vs.start()
            finally:
                vs.stop()
                cam.stop()
                Planner.getInstance().stop()
