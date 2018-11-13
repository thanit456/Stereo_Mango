from motion_control import Planner, Control
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

def main():
    test_control()
    test_planner()
    

if __name__ == '__main__':    
    main()