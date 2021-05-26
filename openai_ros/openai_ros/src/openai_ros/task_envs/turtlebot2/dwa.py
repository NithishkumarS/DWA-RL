#!/usr/bin/env python

import math
import numpy as np
import time


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
def calc_final_input(x, u, dw, config, ob, num_stacked_frames):

    xinit = x[:]
    min_cost = 10000.0
    min_u = u
    min_u[0] = 0.0
    v_list = []
    w_list = []
    cost_list = []
    cost_matrix = np.empty((0,num_stacked_frames), int)
    obst_cost_matrix = np.empty((0,num_stacked_frames), int)
    to_goal_cost_matrix = np.empty((0,num_stacked_frames), int)
    v_reso = (dw[1] - dw[0]) / 12
    yawrate_reso = (dw[3] - dw[2]) / 12
    # evaluate all trajectory with sampled input in dynamic window
    for w in np.arange(dw[2], dw[3], yawrate_reso):
        for v in np.arange(dw[0], dw[1], v_reso):
            traj = calc_trajectory(xinit, v, w, config)

            # calc costs with weighted gains
            to_goal_cost = calc_to_goal_cost(traj, config) * config.to_goal_cost_gain
            speed_cost = config.speed_cost_gain * \
                (config.max_speed - traj[-1, 3])
            
            ob_cost = calc_obs_cost_bigO_efficient(traj, ob, config, num_stacked_frames) * config.obs_cost_gain

            
            final_cost = to_goal_cost + speed_cost + ob_cost
            to_goal_array = np.array([to_goal_cost])
            to_goal_array = np.repeat([to_goal_array],num_stacked_frames, axis = 1)
            v_list.append(v)
            w_list.append(w)
            cost_matrix = np.append(cost_matrix,final_cost, axis = 0)
            obst_cost_matrix = np.append(obst_cost_matrix, ob_cost, axis = 0)
            to_goal_cost_matrix = np.append(to_goal_cost_matrix, to_goal_array, axis = 0)    

    cost_list, v_list, w_list, cost_matrix, obst_cost_matrix, to_goal_cost_matrix = zip(*sorted(zip(cost_matrix[:,num_stacked_frames - 1], v_list, w_list, cost_matrix, obst_cost_matrix, to_goal_cost_matrix))) # Change 5 with a parameter

    cost_matrix = np.asarray(cost_matrix)
    max_cost = np.max(cost_matrix)
    cost_matrix_normalized = cost_matrix / max_cost

    obst_cost_matrix = np.asarray(obst_cost_matrix)
    max_obst_cost = np.max(obst_cost_matrix)
    obst_cost_matrix_normalized = obst_cost_matrix / max_obst_cost

    to_goal_cost_matrix = np.asarray(to_goal_cost_matrix)
    max_to_goal_cost = np.max(to_goal_cost_matrix)
    to_goal_cost_matrix_normalized = to_goal_cost_matrix / max_to_goal_cost

    np_v_list = np.asarray(v_list)
    np_v_list = np_v_list[:,np.newaxis]
    v_matrix = np.tile(np_v_list, num_stacked_frames)

    np_w_list = np.asarray(w_list)
    np_w_list = np_w_list[:,np.newaxis]
    w_matrix = np.tile(np_w_list, num_stacked_frames)
    
    return v_matrix, w_matrix, cost_matrix_normalized, obst_cost_matrix_normalized, to_goal_cost_matrix_normalized

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
                return 40  # collision

            if minr >= r:
                minr = r

    return 1.0 / minr

def calc_obs_cost_bigO_efficient(traj, ob, config, num_stacked_frames):
    skip_n = 2
    r_min_stacked = np.tile(np.array([np.array([float("inf")])]), num_stacked_frames)
    obs_cost = np.tile(np.array([np.array([float("inf")])]), num_stacked_frames)

    for ii in range(0, len(traj[:, 1]), skip_n):
        trajxy = np.array([traj[ii, 0], traj[ii, 1]])
        trajxy_stacked = np.tile(trajxy, num_stacked_frames)  # Repeat the trajxy to make it same in dimension as ob
        square_dxy = np.square(ob - trajxy_stacked)
        square_dxy_odd = square_dxy[:,1::2]
        square_dxy_even = square_dxy[:,0::2]
        sqaure_dxy_sum = np.add(square_dxy_odd, square_dxy_even)
        dist_stacked = np.sqrt(sqaure_dxy_sum)
        r_min_tmp = np.min(dist_stacked, axis = 0)
        r_min_stacked = np.vstack((r_min_stacked, r_min_tmp))

    r_min = np.min(r_min_stacked, axis = 0)
    
    for i in range(0, len(r_min)):
        if r_min[i] <= config.robot_radius:
            obs_cost[0,i] = 40

        else:
            obs_cost[0,i] = 1.0 / r_min[i]
    return obs_cost

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
def dwa_control(x, u, config, obstacles, num_stacked_frames):
    # Dynamic Window control

    dw = calc_dynamic_window(x, config)

    u = calc_final_input(x, u, dw, config, obstacles, num_stacked_frames)

    return u

# Determine whether the robot has reached its goal
def atGoal(config, x):
    # check at goal
    if math.sqrt((x[0] - config.goalX)**2 + (x[1] - config.goalY)**2) \
        <= config.robot_radius:
        return True
    return False


def DWA(config, obstacles, num_stacked_frames):
   
    # initial state [x(m), y(m), theta(rad), v(m/s), omega(rad/s)]
    x = np.array([config.x, config.y, config.th, config.v, config.omega])
    # initial linear and angular velocities
    u = np.array([0.0, 0.0])

    start_time = time.time()
    v_matrix, w_matrix, cost_matrix_normalized, obst_cost_matrix_normalized, to_goal_cost_matrix_normalized = dwa_control(x, u, config, obstacles, num_stacked_frames)
    
    
    return v_matrix, w_matrix, cost_matrix_normalized, obst_cost_matrix_normalized, to_goal_cost_matrix_normalized
       




