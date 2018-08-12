import visual_servo
import driver
from motion_control import planner

if __name__ == '__main__':
	cam = driver.DriverCamera(0)
	vs = VisualServo(cam)

	vs.set_point_cloud(2000, 1000, 1000)
	vs.start()