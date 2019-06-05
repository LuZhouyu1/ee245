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
        elif shape == "circle":
            self.trajectory_circle_wp()
        elif shape == "eight":
            self.trajectory_eight_wp()
        else:
            print "not a defined shape"

    def trajectory_square_wp(self):
        # record waypoints
        ini_Pos = self.cf.planner.lastKnownPosition
        s_len = self.parameter
        self.waypoints=np.array([[ini_Pos.x+s_len,ini_Pos.y,ini_Pos.z],[ini_Pos.x+s_len,ini_Pos.y+s_len,ini_Pos.z],[ini_Pos.x,ini_Pos.y+s_len,ini_Pos.z],[ini_Pos.x,ini_Pos.y,ini_Pos.z]])
        self.duration = self.total_time/4

    def trajectory_circle_wp(self):
        # record waypoints
        self.duration=0.1
        ini_Pos = self.cf.planner.lastKnownPosition
        R = self.parameter
        center = np.array([R,0,ini_Pos.z])
        N_sample = int(self.total_time/self.duration)
        theta = np.linspace(-math.pi,math.pi,N_sample)
        self.waypoints=np.zeros((N_sample,3))
        for i in range (1,N_sample):
            self.waypoints[i-1,0] = center[0]+R*math.cos(theta[i])
            self.waypoints[i-1,1] = center[1]+R*math.sin(theta[i])
            self.waypoints[i-1,2] = ini_Pos.z
        self.waypoints[N_sample-1,2] = ini_Pos.z

    def trajectory_eight_wp(self):
        # record waypoints
        self.duration=0.1
        ini_Pos = self.cf.planner.lastKnownPosition
        R = self.parameter
        center_1 = np.array([R,0,ini_Pos.z])
        center_2 = np.array([R,2*R,ini_Pos.z])
        N_sample_1 = int((self.total_time/self.duration*3)/8)
        N_sample_2 = int((self.total_time/self.duration)/2)
        N_sample_3 = int((self.total_time/self.duration)/8)
        N_sample = N_sample_1+N_sample_2+N_sample_3
        print N_sample_1
        print N_sample_2
        print N_sample_3
        theta_1 = np.linspace(-math.pi,math.pi*1/2,N_sample_1)
        theta_2 = np.linspace(-math.pi*1/2,math.pi*3/2,N_sample_2)
        theta_2 = theta_2[::-1]
        theta_3 = np.linspace(math.pi*1/2,math.pi,N_sample_3)
        self.waypoints=np.zeros((N_sample-2,3))
        for i in range (1,N_sample_1):
            self.waypoints[i-1,0] = center_1[0]+R*math.cos(theta_1[i])
            self.waypoints[i-1,1] = center_1[1]+R*math.sin(theta_1[i])
            self.waypoints[i-1,2] = ini_Pos.z
        for i in range (1,N_sample_2):
            self.waypoints[i+N_sample_1-2,0] = center_2[0]+R*math.cos(theta_2[i])
            self.waypoints[i+N_sample_1-2,1] = center_2[1]+R*math.sin(theta_2[i])
            self.waypoints[i+N_sample_1-2,2] = ini_Pos.z
        for i in range (1,N_sample_3):
            self.waypoints[i+N_sample_1+N_sample_2-3,0] = center_1[0]+R*math.cos(theta_3[i])
            self.waypoints[i+N_sample_1+N_sample_2-3,1] = center_1[1]+R*math.sin(theta_3[i])
            self.waypoints[i+N_sample_1+N_sample_2-3,2] = ini_Pos.z
        self.waypoints[N_sample-3,2] = ini_Pos.z

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
    cf_dtraj=Design_trajectory("square",20.0,1.0,cf)
    n_wp=len(cf_dtraj.waypoints)
    for i in range(0,n_wp):
        cf.goTo(goal=cf_dtraj.waypoints[i],yaw=cf_dtraj.yaw,duration=cf_dtraj.duration)
        time.sleep(cf_dtraj.duration)

    cf_dtraj=Design_trajectory("circle",20.0,1.0,cf)
    n_wp=len(cf_dtraj.waypoints)
    for i in range(0,n_wp):
        cf.cmdPosition(pos=cf_dtraj.waypoints[i],yaw=cf_dtraj.yaw)
        time.sleep(cf_dtraj.duration)

    cf_dtraj=Design_trajectory("eight",30.0,1.0,cf)
    n_wp=len(cf_dtraj.waypoints)
    for i in range(0,n_wp):
        cf.cmdPosition(pos=cf_dtraj.waypoints[i],yaw=cf_dtraj.yaw)
        time.sleep(cf_dtraj.duration)

    cf.land(targetHeight = 0.0, duration = 5.0)
    time.sleep(5.0)
