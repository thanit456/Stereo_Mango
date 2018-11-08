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

def main():
    con = Control.getInstance()

    time.sleep(1)
    con.move_to(0, 500, 1000)
    wait(con)
    print (con)

    con.move_to(500, 500, 1500)
    wait(con)
    print (con)
    
    con.move_to(500, 500, 1000)
    wait(con)
    print (con)
    
    con.move_to(1000, 500, 1000)
    wait(con)
    print (con)

    con.stop()

if __name__ == '__main__':    
    main()