from motion_control import Planner
import config
import time
import pprint


if __name__ == '__main__':    
    planner = Planner.getInstance()
    time.sleep(1)
    
    #print ("Test turret")
    #planner.turn_to_arm(90, config.default_speed)
    #time.sleep(300 / config.default_speed)
        
#     print ("base x, y, z")
#     planner.move_stereo_cam(100, 0, 0, config.default_speed)
#     time.sleep(500 / config.default_speed)
#     """
#     print ("Test x cam")
#     planner.move_single_cam(100, 0, 100, 0, config.default_speed) # test x
#     time.sleep(300 / config.default_speed)
    print('A1')
    planner.force(config.FORWARD_MOTOR_ID, 0, 500, 10000)
    print('A2')
    planner.force(config.TURRET_MOTOR_ID, 0, 500, 100)
    print('A3')
    planner.force(config.MIDDLE_MOTOR_ID, 4400000, 500, 10000)
    
    
    print('B1')
    planner.force(config.FORWARD_MOTOR_ID, 0, 500, 10000)
    print('B2')
    planner.force(config.TURRET_MOTOR_ID, 4050, 500, 100)
    print('B3')
    planner.force(config.MIDDLE_MOTOR_ID, 800000, 500, 10000)
#     print ("Test y cam")
#     planner.move_single_cam(0, 500, 0, 0, config.default_speed) # test y
#     time.sleep(300 / config.default_speed)
#     planner.move_single_cam(0, -500, 0, 0, config.default_speed) # test y
#     time.sleep(300 / config.default_speed)
#     """
#     print ("Test z cam")
    #planner.move_single_cam(0, 0, 100, 0, config.default_speed) # test z
    #time.sleep(300 / config.default_speed)
    print('Mango Drop')
    planner.drop_mango(True)
    time.sleep(1)
    planner.drop_mango(False)
    time.sleep(1)
    
    planner.stop()
    
