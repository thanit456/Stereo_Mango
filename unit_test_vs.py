import visual_servo
import driver
from mango_detection import mango_detector as md
#from motion_control import Planner

if __name__ == '__main__':
    mango_model = md.load_model('mango_detection/model/')
    cam = driver.DriverCamera(0)
    cam.start()
    vs = visual_servo.VisualServo(cam, mango_model)
    
    vs.set_point_cloud(2000, 1000, 1000)
    vs.start()
