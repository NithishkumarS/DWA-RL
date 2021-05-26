#!/usr/bin/env python

class Config():
    # simulation parameters

    def __init__(self, odom, goal):
        # robot parameter
        #NOTE good params:
        #NOTE 0.55,0.1,1.0,1.6,3.2,0.15,0.05,0.1,1.7,2.4,0.1,3.2,0.18
        self.max_speed = 0.65  # [m/s]
        self.min_speed = 0.0  # [m/s]
        self.max_yawrate = 1.0  # [rad/s]
        self.max_accel = 2.5  # [m/ss]
        self.max_dyawrate = 3.2  # [rad/ss]
        self.dt = 0.1  # [s]
        self.predict_time = 1.5  # [s]
        self.to_goal_cost_gain = 2.4 #lower = detour
        self.speed_cost_gain = 0.1 #lower = faster
        self.obs_cost_gain = 3.2 #lower z= fearless
        self.robot_radius = 0.15  # [m]
        self.goalX = goal["x"]
        self.goalY = goal["y"]
        self.x = odom["x"]
        self.y = odom["y"]
        self.th = odom["theta"]
        self.v = odom["u"]
        self.omega = odom["omega"]
        
        
      