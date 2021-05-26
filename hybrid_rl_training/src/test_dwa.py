#!/usr/bin/env python

# Author: Connor McGuile
# Feel free to use in any way.

# A custom Dynamic Window Approach implementation for use with Turtlebot.
# Obstacles are registered by a front-mounted laser and stored in a set.
# If, for testing purposes or otherwise, you do not want the laser to be used,
# disable the laserscan subscriber and create your own obstacle set in main(),
# before beginning the loop. If you do not want obstacles, create an empty set.
# Implentation based off Fox et al.'s paper, The Dynamic Window Approach to 
# Collision Avoidance (1997).
import rospy
import math
import numpy as np
from std_msgs.msg import Float32
from geometry_msgs.msg import Twist, PointStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from tf.transformations import euler_from_quaternion
import time
from gazebo_msgs.srv import SetModelState, SetModelStateRequest
import sys
import csv
from odom_calculator import odom_calulator

class Config():
    # simulation parameters

    def __init__(self):
        # robot parameter
        #NOTE good params:
        #NOTE 0.55,0.1,1.0,1.6,3.2,0.15,0.05,0.1,1.7,2.4,0.1,3.2,0.18
        self.max_speed = 0.65  # [m/s]
        self.min_speed = 0.0  # [m/s]
        self.max_yawrate = 1.0  # [rad/s]
        self.max_accel = 2.5  # [m/ss]
        self.max_dyawrate = 3.2  # [rad/ss]
        self.v_reso = 0.15  # [m/s]
        self.yawrate_reso = 0.05  # [rad/s]
        self.dt = 0.5  # [s]
        self.predict_time = 1.5  # [s]
        self.to_goal_cost_gain = 2.4 #lower = detour
        self.speed_cost_gain = 0.1 #lower = faster
        self.obs_cost_gain = 3.2 #lower z= fearless
        self.robot_radius = 0.15  # [m]
        self.x = 0.0
        self.y = 0.0
        self.goalX = 0
        self.goalY = 0
        self.th = 0.0
        self.r = rospy.Rate(20)

        self.goalX = robot_goal["x"]
        self.goalY = robot_goal["y"]


    # Callback for Odometry
    def assignOdomCoords(self, msg):
        # X- and Y- coords and pose of robot fed back into the robot config
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y
        rot_q = msg.pose.pose.orientation
        (roll,pitch,theta) = \
            euler_from_quaternion ([rot_q.x,rot_q.y,rot_q.z,rot_q.w])
        self.th = theta

time_list = []


class Obstacles():
    def __init__(self):
        # Set of coordinates of obstacles in view
        self.obst = set()
        self.collision_status = False
        
    # Custom range implementation to loop over LaserScan degrees with
    # a step and include the final degree
    def myRange(self,start,end,step):
        i = start
        while i < end:
            yield i
            i += step
        yield end

    def return_time_taken(self):
        return self.time_list


    # Callback for LaserScan
    def assignObs(self, msg, config):

        deg = len(msg.ranges)   # Number of degrees - varies in Sim vs real world
        # print("Laser degree length {}".format(deg))
        self.obst = set()   # reset the obstacle set to only keep visible objects

        maxAngle = 270
        scanSkip = 4
        anglePerSlot = (float(maxAngle) / deg) * scanSkip
        angleCount = 0
        angleValuePos = 0
        angleValueNeg = 0
        self.collision_status = False
        for angle in self.myRange(0,deg-1,scanSkip):
            distance = msg.ranges[angle]

            if (distance < 0.3) and (not self.collision_status):
                self.collision_status = True
                # print("Collided")
                reached = False
                reset_robot(reached)
                
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
                

                objTheta =  scanTheta + config.th
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
                obsX = round((config.x + (distance * math.cos(abs(objTheta))))*8)/8
                # determine direction of Y coord
                # if (objTheta < 0): # uncomment and comment line below for Gazebo simulation
                if (objTheta < 0):
                    obsY = round((config.y - (distance * math.sin(abs(objTheta))))*8)/8
                else:
                    obsY = round((config.y + (distance * math.sin(abs(objTheta))))*8)/8

                # print("Robot's current location {} {}".format(config.x, config.y))
                # print("Obstacle's current location {} {}".format(obsX, obsY))
                # print("Current yaw of the robot {}".format(config.th))

                # add coords to set so as to only take unique obstacles
                self.obst.add((obsX,obsY))
                # print("The obstacle space is {}".format(self.obst))
                #print self.obst
        # print("The total angle count is {}".format(angleCount  ))

# Model to determine the expected position of the robot after moving along trajectory
def motion(x, u, dt):
    # motion model
    # x = [x(m), y(m), theta(rad), v(m/s), omega(rad/s)]
    x[2] += u[1] * dt
    x[0] += u[0] * math.cos(x[2]) * dt
    x[1] += u[0] * math.sin(x[2]) * dt
    
    x[3] = u[0]
    x[4] = u[1]

    return x

# Determine the dynamic window from robot configurations
def calc_dynamic_window(x, config):

    # Dynamic window from robot specification
    Vs = [config.min_speed, config.max_speed,
          -config.max_yawrate, config.max_yawrate]

    # Dynamic window from motion model
    Vd = [x[3] - config.max_accel * config.dt,
          x[3] + config.max_accel * config.dt,
          x[4] - config.max_dyawrate * config.dt,
          x[4] + config.max_dyawrate * config.dt]

    #  [vmin, vmax, yawrate min, yawrate max]
    dw = [max(Vs[0], Vd[0]), min(Vs[1], Vd[1]),
          max(Vs[2], Vd[2]), min(Vs[3], Vd[3])]

    return dw

# Calculate a trajectory sampled across a prediction time
def calc_trajectory(xinit, v, y, config):

    x = np.array(xinit)
    traj = np.array(x)  # many motion models stored per trajectory
    time = 0
    while time <= config.predict_time:
        # store each motion model along a trajectory
        x = motion(x, [v, y], config.dt)
        traj = np.vstack((traj, x))
        time += config.dt # next sample

    return traj

# Calculate trajectory, costings, and return velocities to apply to robot
def calc_final_input(x, u, dw, config, ob):

    xinit = x[:]
    min_cost = 10000.0
    min_u = u
    min_u[0] = 0.0

    # evaluate all trajectory with sampled input in dynamic window
    for v in np.arange(dw[0], dw[1], config.v_reso):
        for w in np.arange(dw[2], dw[3], config.yawrate_reso):
            traj = calc_trajectory(xinit, v, w, config)

            # calc costs with weighted gains
            to_goal_cost = calc_to_goal_cost(traj, config) * config.to_goal_cost_gain
            speed_cost = config.speed_cost_gain * \
                (config.max_speed - traj[-1, 3])

            ob_cost = calc_obstacle_cost(traj, ob, config) * config.obs_cost_gain

            final_cost = to_goal_cost + speed_cost + ob_cost

            # search minimum trajectory
            if min_cost >= final_cost:
                min_cost = final_cost
                min_u = [v, w]
    return min_u

# Calculate obstacle cost inf: collision, 0:free
def calc_obstacle_cost(traj, ob, config):
    skip_n = 2
    minr = float("inf")

    # Loop through every obstacle in set and calc Pythagorean distance
    # Use robot radius to determine if collision
    for ii in range(0, len(traj[:, 1]), skip_n):
        for i in ob.copy():
            ox = i[0]
            oy = i[1]
            dx = traj[ii, 0] - ox
            dy = traj[ii, 1] - oy

            r = math.sqrt(dx**2 + dy**2)

            if r <= config.robot_radius:
                return float("Inf")  # collision

            if minr >= r:
                minr = r

    return 1.0 / minr

# Calculate goal cost via Pythagorean distance to robot
def calc_to_goal_cost(traj, config):
    # If-Statements to determine negative vs positive goal/trajectory position
    # traj[-1,0] is the last predicted X coord position on the trajectory
    if (config.goalX >= 0 and traj[-1,0] < 0):
        dx = config.goalX - traj[-1,0]
    elif (config.goalX < 0 and traj[-1,0] >= 0):
        dx = traj[-1,0] - config.goalX
    else:
        dx = abs(config.goalX - traj[-1,0])
    # traj[-1,1] is the last predicted Y coord position on the trajectory
    if (config.goalY >= 0 and traj[-1,1] < 0):
        dy = config.goalY - traj[-1,1]
    elif (config.goalY < 0 and traj[-1,1] >= 0):
        dy = traj[-1,1] - config.goalY
    else:
        dy = abs(config.goalY - traj[-1,1])

    cost = math.sqrt(dx**2 + dy**2)
    return cost

# Begin DWA calculations
def dwa_control(x, u, config, ob):
    # Dynamic Window control

    dw = calc_dynamic_window(x, config)

    u = calc_final_input(x, u, dw, config, ob)

    return u

# Determine whether the robot has reached its goal
def atGoal(config, x):
    # check at goal
    if math.sqrt((x[0] - config.goalX)**2 + (x[1] - config.goalY)**2) \
        <= config.robot_radius + .4:
        return True
    return False

td = 0
def getDistance(msg):
    global td
    td = msg.data


def reset_robot(reached):
    global time_list, start_time ,td, start_d
    reset_robot = rospy.ServiceProxy('/gazebo/set_model_state', SetModelState)
    robot_reset_request = SetModelStateRequest()
    time_taken = rospy.get_time() - start_time
    distance_travelled = td - start_d

    if time_taken > 0.1:
        time_list.append([round(time_taken,2), round(distance_travelled,2), reached]) #, odom_obj.TotalDistance])     
    start_time = rospy.get_time()
    start_d = td
     
    robot_reset_request.model_state.model_name = 'turtlebot' + str(robot_number)
    robot_reset_request.model_state.pose.position.x = initial_pose["x_init"]
    robot_reset_request.model_state.pose.position.y = initial_pose["y_init"]
    robot_reset_request.model_state.pose.orientation.x = initial_pose["x_rot_init"]
    robot_reset_request.model_state.pose.orientation.y = initial_pose["y_rot_init"]
    robot_reset_request.model_state.pose.orientation.z = initial_pose["z_rot_init"]
    robot_reset_request.model_state.pose.orientation.w = initial_pose["w_rot_init"]

    rospy.wait_for_service('/gazebo/set_model_state')
    
    
    try:
        reset_robot(robot_reset_request)
        
    except rospy.ServiceException as e:
        print ("/gazebo/set_model_state service call failed")
    start_time = rospy.get_time()


def main():
    print(__file__ + " start!!")
    # robot specification

    config = Config()
    # position of obstacles
    global start_time, td, start_d
    
    obs = Obstacles()
    subOdom = rospy.Subscriber("/turtlebot"+str(robot_number)+"/ground_truth/state", Odometry, config.assignOdomCoords)
    subLaser = rospy.Subscriber("/turtlebot"+str(robot_number)+"/scan_filtered", LaserScan, obs.assignObs, config)
    TotalDistance = rospy.Subscriber("/total_distance",Float32, getDistance)
    start_d = td

    pub = rospy.Publisher("/turtlebot"+str(robot_number)+"/cmd_vel_mux/input/navi", Twist, queue_size=1)
    speed = Twist()
    # initial state [x(m), y(m), theta(rad), v(m/s), omega(rad/s)]
    x = np.array([config.x, config.y, config.th, 0.0, 0.0])
    # initial linear and angular velocities
    u = np.array([0.0, 0.0])

    max_test_episodes = 50
    reached = False
    count = 0
    total_collisions = 0
    reached_goal = 0

    start_time = rospy.get_time()


    # runs until terminated externally
    while not rospy.is_shutdown() and (count < max_test_episodes):
        if (atGoal(config,x) == False):
            # start_time = time.time()
            if (obs.collision_status):
                count += 1
                total_collisions += 1

            u = dwa_control(x, u, config, obs.obst)
            # print("Time to calculate a single pass through DWA {}".format(time.time() - start_time))

            x[0] = config.x
            x[1] = config.y
            x[2] = config.th
            x[3] = u[0]
            x[4] = u[1]
            speed.linear.x = x[3]
            speed.angular.z = x[4]
            reached = False
        else:
            # if at goal then stay there until new goal published
            speed.linear.x = 0.0
            speed.angular.z = 0.0
            if not reached:
                count += 1
                reached_goal += 1
            reached = True
            print("The count is {}".format(count))
            reset_robot(reached)
            x = np.array([config.x, config.y, config.th, 0.0, 0.0])
        # print(atGoal(config,x))
        pub.publish(speed)
        config.r.sleep()
    print("Total collisions and success {} ---- {}".format(total_collisions, reached_goal))
    global time_list
    for i in range(len(time_list)):
        print(time_list[i])
    file = open('dwa_'+world_name+'.csv', 'w') 

    # writing the data into the file 
    with file:     
      write = csv.writer(file) 
      write.writerows(time_list) 


if __name__ == '__main__':
    rospy.init_node('dwa')

    global start_time, world_name
    start_time = rospy.get_time()
    world_name = sys.argv[1]
    global robot_number
    robot_number = sys.argv[2]
    global initial_pose, robot_goal
    initial_pose = {}
    robot_goal = {}

    if world_name == "zigzag_3ped" :    

        initial_pose["x_init"] = 1
        initial_pose["y_init"] = 0
        initial_pose["x_rot_init"] = 0
        initial_pose["y_rot_init"] = 0
        initial_pose["z_rot_init"] = 0
        initial_pose["w_rot_init"] = 1
        robot_goal["x"] = 12.5
        robot_goal["y"] = 0
    

    elif world_name == "room":
        initial_pose["x_init"] = 0
        initial_pose["y_init"] = 0
        initial_pose["x_rot_init"] = 0
        initial_pose["y_rot_init"] = 0
        initial_pose["z_rot_init"] = 1
        initial_pose["w_rot_init"] = 0
        robot_goal["x"] = -9.5
        robot_goal["y"] = -1.5

    elif world_name == "wallped3_8":
        initial_pose["x_init"] = 11
        initial_pose["y_init"] = 0
        initial_pose["x_rot_init"] = 0
        initial_pose["y_rot_init"] = 0
        initial_pose["z_rot_init"] = 1
        initial_pose["w_rot_init"] = 0
        robot_goal["x"] = -9
        robot_goal["y"] = 0

    elif world_name == "zigzag_static":

        initial_pose["x_init"] = -11
        initial_pose["y_init"] = 7.5
        initial_pose["x_rot_init"] = 0
        initial_pose["y_rot_init"] = 0
        initial_pose["z_rot_init"] = 0
        initial_pose["w_rot_init"] = 1
        robot_goal["x"] = 4.4
        robot_goal["y"] = -8.4
   
    

    main()