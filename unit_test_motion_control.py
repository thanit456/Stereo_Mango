from motion_control import Planner, Control
import motion_control as mc
import numpy as np

import config
import time
import pprint
import datetime

def wait(con, t = 5):
    st = datetime.datetime.now()
    while con.is_moving(): # or (datetime.datetime.now() - st).seconds < t:
        print (con)
        time.sleep(1)

def test_control():
    print ("Test Control")
    con = Control.getInstance()
    
    # print (con)
    con.set_position(config.BASE_MOTOR_ID_L, 0, config.default_spd[2], 0)
    wait(con)
    print (con)

    # con.move_to(500, 500, 1500)
    # wait(con)
    # print (con)
    
    # con.move_to(500, 500, 1000)
    # wait(con)
    # print (con)
    
    # con.move_to(1000, 500, 1000)
    # wait(con)
    # print (con)

    con.stop()

def test_planner():
    print ("Test Planner")

def test_ik(continues = 0):

    mid_pos = np.array([0 + config.arm_min_workspace, 0], dtype=np.float64)
    for z in range(0, 580,100):
        end_pos = np.array([0 + config.arm_min_workspace, z], dtype=np.float64)

        x, z, turret, forward = mc.inverse_kinematics(mid_pos, end_pos)
        print (x - config.arm_min_workspace, z, turret, forward)
        if continues:
            mid_pos[0] = x
            mid_pos[1] = z

def main():
    # test_control()
    # test_planner()
    test_ik()
    

if __name__ == '__main__':    
    main()