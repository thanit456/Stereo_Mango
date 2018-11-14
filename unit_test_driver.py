from Driver.motor import DriverMotor, DriverServo
from Driver.camera import DriverCamera
from Driver.stereo import DriverStereo

import datetime

def delay(ms = 1):
	time = datetime.datetime.now()
	while (datetime.datetime.now() - time).microseconds < ms:
		pass

# test_1 = driver.DriverMotor('http://localhost:8080/api', 1)

# print ("Set ppm: " + str(test_1.set_pulse_per_mm(8000)))

# print ("set min max movement")
# print (test_1.set_min_movement(0))
# print (test_1.set_max_movement(8000))

# print ("Get test")
# print ("Get goal: " + str(test_1.get_goal()))
# print ("Get current: " + str(test_1.get_current()))
# print ("Get gpio: " + str(test_1.get_gpio()))
# print ("Get hareware error: " + str(test_1.get_hw_error()))
# print ("Get adc: " + str(test_1.get_adc(1)))
# print ("Get adc: " + str(test_1.get_adc(2)))
# print ("Get adc: " + str(test_1.get_adc(3)))
# print ("Get adc: " + str(test_1.get_adc(4)))

# print ("set test")
# print ("set threshould: " + str(test_1.set_moving_threshould(300)))

# print ("Enable: " + str(test_1.enable(1)))

# print ("Change mode: " + str(test_1.set_mode(0)))
# print ("Set pwm: " + str(test_1.set_goal_pwm(200)))
# delay(500)
# print ("Set pwm: " + str(test_1.set_goal_pwm(0)))
# print ("Get position: " + str(test_1.get_current()))

# delay(500)

# print ("Change mode: " + str(test_1.set_mode(1)))
# print ("Set pos: " + str(test_1.set_goal_pos(1, 0)))
# delay(500)
# print ("Get position: " + str(test_1.get_current()))

# print ("Disable: " +str(test_1.enable(0)))
# delay (500)
# print ("Get position: " + str(test_1.get_current()))

# print ("cmd test")
# print ("Reset position: " + str(test_1.reset_position()))
# print ("Get position: " + str(test_1.get_current()))
