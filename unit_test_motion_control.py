from motion_control import Planner
import config
import time
import pprint
import datetime

def wait(plan, t = 5):
    st = datetime.datetime.now()
    while plan.is_moving(): # or (datetime.datetime.now() - st).seconds < t:
        print (plan)
        time.sleep(1)

def main():
    planner = Planner.getInstance()
    # planner.start()
    planner.move_to(0, 500, 1000)
    wait(planner)
    print (planner)

    planner.move_to(500, 500, 1500)
    wait(planner)
    print (planner)
    
    planner.move_to(500, 500, 1000)
    wait(planner)
    print (planner)
    
    planner.move_to(1000, 500, 1000)
    wait(planner)
    print (planner)

    planner.stop()

if __name__ == '__main__':    
    main()