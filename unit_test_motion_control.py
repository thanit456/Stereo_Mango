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
        
    print ("base x, y, z")
    #planner.move_stereo_cam(500, 500, 500, config.default_speed)
    #time.sleep(300 / config.default_speed)
    """
    print ("Test x cam")
    planner.move_single_cam(250, 0, 0, 0, config.default_speed) # test x
    time.sleep(300 / config.default_speed)
    

    print ("Test y cam")
    #planner.move_single_cam(0, 500, 0, 0, config.default_speed) # test y
    #time.sleep(300 / config.default_speed)
    """
    print ("Test z cam")
    planner.move_single_cam(0, 0, 100, 0, config.default_speed) # test z
    time.sleep(300 / config.default_speed)
    
    planner.stop()
    
