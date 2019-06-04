#!/usr/bin/env python

import math
import numpy as np
from crazyflieParser import CrazyflieParser

# class of designed trajectory
class Design_trajectory:
    def __init__(self, shape, total_time, parameter, cf): # parameter means the size of the square
        self.yaw=0
        self.total_time=total_time
        self.parameter=parameter
        self.cf=cf
        if shape == "square":
            self.trajectory_square_wp()
        else:
            print "future work"

    def trajectory_square_wp(self):
        # record waypoints
        ini_Pos = self.cf.planner.lastKnownPosition
        s_len = self.parameter
        self.waypoints=np.array([[ini_Pos.x+s_len,ini_Pos.y,ini_Pos.z],[ini_Pos.x+s_len,ini_Pos.y+s_len,ini_Pos.z],[ini_Pos.x,ini_Pos.y+s_len,ini_Pos.z],[ini_Pos.x,ini_Pos.y,ini_Pos.z]])
        self.duration = self.total_time/4


if __name__ == '__main__':

    index = 1   # for cf1
    initialPosition = [0,0,0] # x,y,z coordinate for this crazyflie
    cfs = CrazyflieParser(index, initialPosition)
    cf = cfs.crazyflies[0]
    time = cfs.timeHelper

    cf.setParam("commander/enHighLevel", 1)
    cf.setParam("stabilizer/estimator", 2) # Use EKF
    cf.setParam("stabilizer/controller", 2) # Use mellinger controller
    #cf.setParam("ring/effect", 7)

    cf.takeoff(targetHeight = 0.5, duration = 3.0)
    time.sleep(3.0)

    # FILL IN YOUR CODE HERE
    # Please try both goTo and cmdPosition

    # initialization
    cf_dtraj=Design_trajectory("square",20.0,2.0,cf)
    n_wp=len(cf_dtraj.waypoints)
    for i in range(0,n_wp):
        cf.goTo(goal=cf_dtraj.waypoints[i],yaw=cf_dtraj.yaw,duration=cf_dtraj.duration)
        time.sleep(cf_dtraj.duration)

    cf.land(targetHeight = 0.0, duration = 5.0)
    time.sleep(5.0)
