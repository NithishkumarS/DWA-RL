#!/usr/bin/env python
import math
import numpy as np
class Obstacles():
    def __init__(self, ranges, config):
        # Set of coordinates of obstacles in view
        self.obst = set()
        self.ranges = ranges
        self.config = config
        self.assignObs()
        

    # Custom range implementation to loop over LaserScan degrees with
    # a step and include the final degree
    def myRange(self,start,end,step):
        i = start
        while i < end:
            yield i
            i += step
        yield end

    # Callback for LaserScan
    def assignObs(self):

        deg = len(self.ranges)   # Number of degrees - varies in Sim vs real world
        # print("Laser degree length {}".format(deg))
        self.obst = np.empty([0,2])   # reset the obstacle set to only keep visible objects

        maxAngle = 270
        scanSkip = 4
        anglePerSlot = (float(maxAngle) / deg) * scanSkip
        angleCount = 0
        angleValuePos = 0
        angleValueNeg = 0
        for angle in self.myRange(0,deg-1,scanSkip):
            distance = self.ranges[angle]
            
            if(angleCount < (deg / (2*scanSkip))):
                # print("In negative angle zone")
                angleValueNeg += (anglePerSlot)  
                scanTheta = (angleValueNeg - 135) * math.pi/180.0
                    

            elif(angleCount>(deg / (2*scanSkip))):
                # print("In positive angle zone")
                angleValuePos += anglePerSlot
                scanTheta = angleValuePos * math.pi/180.0
            # only record obstacles that are within 4 metres away

            else:
                scanTheta = 0

            angleCount += 1

            if (distance < 4):
                # angle of obstacle wrt robot
                # angle/2.844 is to normalise the 512 degrees in real world
                # for simulation in Gazebo, use angle/4.0
                # laser from 0 to 180
                # scanTheta = (angle/2.844 + deg*(-180.0/deg)+90.0) *math.pi/180.0
                
                # # angle of obstacle wrt global frame
                # if config.th < 0:
                #     objTheta = config.th + scanTheta
                # else:
                #     objTheta = config.th - scanTheta

                # print("The scan theta is {}".format(scanTheta * 180 / math.pi))
                # print("The angel count is {}".format(angleCount))
                # print("Angle per slot is {}".format(anglePerSlot))
                

                objTheta =  scanTheta + self.config.th
                # # back quadrant negative X negative Y
                # if (objTheta < -math.pi):
                #     # e.g -405 degrees >> 135 degrees
                #     objTheta = objTheta + 1.5*math.pi
                # # back quadrant negative X positve Y
                # elif (objTheta > math.pi):
                #     objTheta = objTheta - 1.5*math.pi

                #print("The scan theta is {}".format(objTheta))

                
                    

                # print("The angle is {}".format(objTheta * 180 / 3.14))





                # round coords to nearest 0.125m
                obsX = round((self.config.x + (distance * math.cos(abs(objTheta))))*8)/8
                # determine direction of Y coord
                # if (objTheta < 0): # uncomment and comment line below for Gazebo simulation
                if (objTheta < 0):
                    obsY = round((self.config.y - (distance * math.sin(abs(objTheta))))*8)/8
                else:
                    obsY = round((self.config.y + (distance * math.sin(abs(objTheta))))*8)/8

                # print("Robot's current location {} {}".format(config.x, config.y))
                # print("Obstacle's current location {} {}".format(obsX, obsY))
                # print("Current yaw of the robot {}".format(config.th))

                # add coords to set so as to only take unique obstacles
                obs_row = np.array([obsX, obsY])
                self.obst = np.vstack((self.obst,obs_row))
                # print("The obstacle space is {}".format(self.obst))
                #print self.obst
        # print("The total angle count is {}".format(angleCount  ))

            else:
                obsX = float("inf")
                obsY = float("inf")
                obs_row = np.array([obsX, obsY])
                self.obst = np.vstack((self.obst,obs_row))

        # print("The set size {}".format((self.obst.shape)))


