#!/usr/bin/env python

import rospy
import numpy
import time
import math
from gym import spaces
from openai_ros.robot_envs import turtlebot2_env
from gazebo_msgs.msg import ModelStates
from gym.envs.registration import register
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Header, Bool
from squaternion import Quaternion
from openai_ros.task_envs.turtlebot2.config import Config
from openai_ros.task_envs.turtlebot2.obstacles import Obstacles
from openai_ros.task_envs.turtlebot2.dwa import DWA
from openai_ros.gazebo_connection import GazeboConnection
import cv2
import os
import csv


# The path is __init__.py of openai_ros, where we import the TurtleBot2MazeEnv directly
timestep_limit_per_episode = 1000 # Can be any Value

register(
        id='TurtleBot2Maze-v0',
        entry_point='openai_ros.task_envs.turtlebot2.turtlebot2_maze:TurtleBot2MazeEnv',
        max_episode_steps=timestep_limit_per_episode,
    )

class TurtleBot2MazeEnv(turtlebot2_env.TurtleBot2Env):
    def __init__(self, world_file_name, robot_number=0):
        """
        This Task Env is designed for having the TurtleBot2 in some kind of maze.
        It will learn how to move around the maze without crashing.
        """
        
        # Only variable needed to be set here
        self.world_file_name = world_file_name
        number_actions = rospy.get_param('/turtlebot2/n_actions',144)
        self.action_space = spaces.Discrete(number_actions)
        
        # We set the reward range, which is not compulsory but here we do it.
        self.reward_range = (-numpy.inf, numpy.inf)
        
        
        #number_observations = rospy.get_param('/turtlebot2/n_observations')
        """
        We set the Observation space for the 6 observations
        cube_observations = [
            round(current_disk_roll_vel, 0),
            round(y_distance, 1),
            round(roll, 1),
            round(pitch, 1),
            round(y_linear_speed,1),
            round(yaw, 1),
        ]
        """
        
        # Actions and Observations
        self.dec_obs = rospy.get_param("/turtlebot2/number_decimals_precision_obs", 1)
        self.linear_forward_speed = rospy.get_param('/turtlebot2/linear_forward_speed',1)
        self.linear_turn_speed = rospy.get_param('/turtlebot2/linear_turn_speed',0.2)
        self.angular_speed = rospy.get_param('/turtlebot2/angular_speed',0.1)
        self.init_linear_forward_speed = rospy.get_param('/turtlebot2/init_linear_forward_speed',0.3)
        self.init_linear_turn_speed = rospy.get_param('/turtlebot2/init_linear_turn_speed',0.4)

        self.n_laser_discretization = rospy.get_param('/turtlebot2/n_laser_discretization',128)
        self.n_observations = rospy.get_param('/turtlebot2/n_observations',144)
        self.min_range = rospy.get_param('/turtlebot2/min_range',0.3)
        self.max_cost = rospy.get_param('/turtlebot2/max_cost',1)
        self.min_cost = rospy.get_param('/turtlebot2/min_cost',0)
        self.n_stacked_frames = rospy.get_param('/turtlebot2/n_stacked_frames',10)
        self.n_skipped_frames = rospy.get_param('/turtlebot2/n_skipped_frames',4)

        self.max_linear_speed = rospy.get_param('/turtlebot2/max_linear_speed',0.65)
        self.max_angular_speed = rospy.get_param('/turtlebot2/max_angular_speed',1)

        self.min_linear_speed = rospy.get_param('/turtlebot2/min_linear_speed',0)
        self.min_angular_speed = rospy.get_param('/turtlebot2/min_angular_speed',0)

        self.robot_number = robot_number
        self._get_goal_location()

        self.pedestrians_info = {}
        self.pedestrians_info["4_robot_3D1P"] = {}
        self.pedestrians_info["train2"] = {}
        self.pedestrians_info["zigzag_3ped"] = {}

        self.pedestrian_pose = {}
        # self.robot_pose = {}

        
        # Here we will add any init functions prior to starting the MyRobotEnv
        self._get_init_pose()
        super(TurtleBot2MazeEnv, self).__init__(robot_number=robot_number, initial_pose = self.initial_pose)

        self.gazebo = GazeboConnection(start_init_physics_parameters= True, robot_number = self.robot_number , initial_pose = self.initial_pose, reset_world_or_sim="ROBOT")
        
        # We create two arrays based on the binary values that will be assigned
        # In the discretization method.
        #laser_scan = self._check_laser_scan_ready()
        laser_scan = self.get_laser_scan()
        rospy.logdebug("laser_scan len===>"+str(len(laser_scan.ranges)))
        
        # Laser data
        self.laser_scan_frame = laser_scan.header.frame_id

        
        
        # Number of laser reading jumped
        self.new_ranges = int(math.ceil(float(len(laser_scan.ranges)) / float(self.n_laser_discretization)))

        
        rospy.logdebug("n_observations===>"+str(self.n_observations))
        rospy.logdebug("new_ranges, jumping laser readings===>"+str(self.new_ranges))
        
        
        high = numpy.full((self.n_observations), self.max_cost)
        low = numpy.full((self.n_observations), self.min_cost)
        
        # We only use two integers
        self.observation_space = spaces.Box(low, high)

        v_list_high = numpy.full((self.n_observations,self.n_stacked_frames),self.max_linear_speed)
        w_list_high = numpy.full((self.n_observations,self.n_stacked_frames),self.max_angular_speed)
        cost_list_high = numpy.full((self.n_observations,self.n_stacked_frames),1)
        obst_list_high = numpy.full((self.n_observations,self.n_stacked_frames),1)
        to_goal_list_high = numpy.full((self.n_observations,self.n_stacked_frames),1)

        v_list_low = numpy.full((self.n_observations,self.n_stacked_frames),self.min_linear_speed)
        w_list_low = numpy.full((self.n_observations,self.n_stacked_frames),self.min_angular_speed)
        cost_list_low = numpy.full((self.n_observations,self.n_stacked_frames),0)
        obst_list_low = numpy.full((self.n_observations,self.n_stacked_frames),0)
        to_goal_list_low = numpy.full((self.n_observations,self.n_stacked_frames),0)
        
        high = numpy.stack((v_list_high, w_list_high, obst_list_high, to_goal_list_high), axis=2)
        low = numpy.stack((v_list_low, w_list_low, obst_list_low, to_goal_list_low), axis=2)

        #MLP obs space
        # high = numpy.full((2*self.n_observations*self.n_stacked_frames, 1), 1)
        # low = numpy.full((2*self.n_observations*self.n_stacked_frames, 1), 0)

        self.observation_space = spaces.Box(low, high)
        
        rospy.logdebug("ACTION SPACES TYPE===>"+str(self.action_space))
        rospy.logdebug("OBSERVATION SPACES TYPE===>"+str(self.observation_space))
        
        # Rewards
        self.forwards_reward = rospy.get_param("/turtlebot2/forwards_reward",5)
        self.invalid_penalty = rospy.get_param("/turtlebot2/invalid_penalty",20)
        self.end_episode_points = rospy.get_param("/turtlebot2/end_episode_points",2000)
        self.goal_reaching_points = rospy.get_param("/turtlebot2/goal_reaching_points",500)

        self.cumulated_steps = 0.0

        # Collision danger reward ---- checks the lidar scan and penalizes accordingly
        self.select_collision_danger_cost = True
        self.collision_danger_cost = 0
        self.prox_penalty1 = -1 
        self.prox_penalty2 = -3.5
        self.closeness_threshold = 2.0

        self.laser_filtered_pub = rospy.Publisher('/turtlebot'+str(robot_number)+'/laser/scan_filtered', LaserScan, queue_size=1)
        self.goal_reaching_status_pub = rospy.Publisher('/turtlebot'+str(robot_number)+'/goal_reaching_status', Bool, queue_size=1)
        self.visualize_obs = False
        self.list_angular_vel = []
        self.list_vel = []

        rospy.Subscriber("/gazebo/model_states", ModelStates, self.callback_modelstates)

        if self.visualize_obs:
           os.chdir("../")
           if not os.path.isdir("observation_visualization"):
               os.mkdir("observation_visualization")
               os.chdir("observation_visualization")
               os.mkdir("v")
               os.mkdir("w")
               os.mkdir("cost")
               os.mkdir("obst")
               os.mkdir("toGoal")
               os.chdir("../")
               os.chdir("src")
           else:
               os.chdir("src")
        self.episode_num = 0
        self.total_collisions = 0
        self.episode_collisions = 0
        self.n_skipped_count = 0
        self.goal_reaching_status = Bool()
        self.goal_reaching_status.data = False

    def _set_init_pose(self):
        """Sets the Robot in its init pose
        """
        self.move_base( self.init_linear_forward_speed,
                        self.init_linear_turn_speed,
                        epsilon=0.05,
                        update_rate=10,
                        min_laser_distance=-1)

        return True


    def _get_init_pose(self):
        """ Gets the initial location of the robot to reset
        """
        self.initial_pose = {}
        if (self.world_file_name == "maze"):

            if (self.robot_number == 0):
                self.initial_pose["x_init"] = -2.12
                self.initial_pose["y_init"] = -0.11
                self.initial_pose["x_rot_init"] = 0
                self.initial_pose["y_rot_init"] = 0
                self.initial_pose["z_rot_init"] = 0
                self.initial_pose["w_rot_init"] = 1

            elif (self.robot_number == 1):
                self.initial_pose["x_init"] = 5.2737
                self.initial_pose["y_init"] = 4.55
                self.initial_pose["x_rot_init"] = 0
                self.initial_pose["y_rot_init"] = 0
                self.initial_pose["z_rot_init"] = 0.9999999
                self.initial_pose["w_rot_init"] = 0.00000000796

            elif(self.robot_number == 2):
                self.initial_pose["x_init"] = 7.64
                self.initial_pose["y_init"] = -12.22
                self.initial_pose["x_rot_init"] = 0
                self.initial_pose["y_rot_init"] = 0
                self.initial_pose["z_rot_init"] = 0.7068
                self.initial_pose["w_rot_init"] = 0.7073

            elif(self.robot_number == 3):
                self.initial_pose["x_init"] = -4.11
                self.initial_pose["y_init"] = -5.78
                self.initial_pose["x_rot_init"] = 0
                self.initial_pose["y_rot_init"] = 0
                self.initial_pose["z_rot_init"] = 0.7079
                self.initial_pose["w_rot_init"] = -0.7079
    

        elif (self.world_file_name == "zigzag_static"):
                self.initial_pose["x_init"] = -11
                self.initial_pose["y_init"] = 7.5
                self.initial_pose["x_rot_init"] = 0
                self.initial_pose["y_rot_init"] = 0
                self.initial_pose["z_rot_init"] = 0
                self.initial_pose["w_rot_init"] = 1

        elif(self.world_file_name == "zigzag_3ped"):

            self.initial_pose["x_init"] = 1
            self.initial_pose["y_init"] = 0
            self.initial_pose["x_rot_init"] = 0
            self.initial_pose["y_rot_init"] = 0
            self.initial_pose["z_rot_init"] = 0
            self.initial_pose["w_rot_init"] = 1
            self.pedestrians_info["zigzag_3ped"][0] = [0,1,2]

        elif self.world_file_name == "wallped3_8":
                self.initial_pose["x_init"] = 11
                self.initial_pose["y_init"] = 0
                self.initial_pose["x_rot_init"] = 0
                self.initial_pose["y_rot_init"] = 0
                self.initial_pose["z_rot_init"] = 1
                self.initial_pose["w_rot_init"] = 0

        elif (self.world_file_name == "4_robot_3D1P"):

            if (self.robot_number == 0):
                self.initial_pose["x_init"] = 0.8866
                self.initial_pose["y_init"] = 0.24
                self.initial_pose["x_rot_init"] = 0
                self.initial_pose["y_rot_init"] = 0
                self.initial_pose["z_rot_init"] = 0
                self.initial_pose["w_rot_init"] = 1
                self.pedestrians_info["4_robot_3D1P"][0] = [[0, "Right"],[1, "Left"]]
                print("robot 0: ",self.pedestrians_info["4_robot_3D1P"][0])
            elif (self.robot_number == 1):
                self.initial_pose["x_init"] = 1.18
                self.initial_pose["y_init"] = 12.13
                self.initial_pose["x_rot_init"] = 0
                self.initial_pose["y_rot_init"] = 0
                self.initial_pose["z_rot_init"] = 0
                self.initial_pose["w_rot_init"] = 1
                self.pedestrians_info["4_robot_3D1P"][1] = [[3, "Right"],[2, "Left"]]
                print("robot 1, ",self.pedestrians_info["4_robot_3D1P"][1])
            elif(self.robot_number == 2):
                self.initial_pose["x_init"] = -10.085
                self.initial_pose["y_init"] = 12.15
                self.initial_pose["x_rot_init"] = 0
                self.initial_pose["y_rot_init"] = 0
                self.initial_pose["z_rot_init"] = 1
                self.initial_pose["w_rot_init"] = 0.001
                self.pedestrians_info["4_robot_3D1P"][2] = []
            elif(self.robot_number == 3):
                self.initial_pose["x_init"] = -11.0
                self.initial_pose["y_init"] = -0.03
                self.initial_pose["x_rot_init"] = 0
                self.initial_pose["y_rot_init"] = 0
                self.initial_pose["z_rot_init"] = 1
                self.initial_pose["w_rot_init"] = 0.001
                self.pedestrians_info["4_robot_3D1P"][3] = [[0, "Straight"]]

        elif (self.world_file_name == "train2"):

            if (self.robot_number == 0):
                self.initial_pose["x_init"] = 0.8866
                self.initial_pose["y_init"] = 0.24
                self.initial_pose["x_rot_init"] = 0
                self.initial_pose["y_rot_init"] = 0
                self.initial_pose["z_rot_init"] = 0
                self.initial_pose["w_rot_init"] = 1
                self.pedestrians_info["train2"][0] = [[0, "Right"],[1, "Left"], [5, "Right"]]
              
            elif (self.robot_number == 1):
                self.initial_pose["x_init"] = 1.18
                self.initial_pose["y_init"] = 12.13
                self.initial_pose["x_rot_init"] = 0
                self.initial_pose["y_rot_init"] = 0
                self.initial_pose["z_rot_init"] = 0
                self.initial_pose["w_rot_init"] = 1
                self.pedestrians_info["train2"][1] = [[3, "Right"],[2, "Left"], [6, "Right"]]
                # print("robot 1, ",self.pedestrians_info["4_robot_3D1P"][1])
            elif(self.robot_number == 2):
                self.initial_pose["x_init"] = -10.085
                self.initial_pose["y_init"] = 12.15
                self.initial_pose["x_rot_init"] = 0
                self.initial_pose["y_rot_init"] = 0
                self.initial_pose["z_rot_init"] = 1
                self.initial_pose["w_rot_init"] = 0.001
                self.pedestrians_info["train2"][2] = []
            elif(self.robot_number == 3):
                self.initial_pose["x_init"] = -11.0
                self.initial_pose["y_init"] = -0.03
                self.initial_pose["x_rot_init"] = 0
                self.initial_pose["y_rot_init"] = 0
                self.initial_pose["z_rot_init"] = 1
                self.initial_pose["w_rot_init"] = 0.001
                self.pedestrians_info["train2"][3] = [[0, "Straight"]]

        elif(self.world_file_name == "zigzag_static"):

            self.initial_pose["x_init"] = -11
            self.initial_pose["y_init"] = 7.5
            self.initial_pose["x_rot_init"] = 0
            self.initial_pose["y_rot_init"] = 0
            self.initial_pose["z_rot_init"] = 0
            self.initial_pose["w_rot_init"] = 1



        return self.initial_pose


    def callback_modelstates(self, msg):

        for i in range(len(msg.name)):
            if (msg.name[i][0:5] == "actor"):
                actor_id = int(msg.name[i][5])

                posx = msg.pose[i].position.x
                posy = msg.pose[i].position.y
                posz = msg.pose[i].position.z

                x = msg.pose[i].orientation.x
                y = msg.pose[i].orientation.y
                z = msg.pose[i].orientation.z
                w = msg.pose[i].orientation.w
 
                q = Quaternion(w,x,y,z)
                e = q.to_euler(degrees=True)
                
                self.pedestrian_pose[actor_id] = [posx,posy, e[2]]


    def _get_goal_location(self):
            """ Gets the goal location for each robot
            """
            self.goal_pose = {}
            if(self.world_file_name == "maze"):
                if (self.robot_number == 0):
                   self.goal_pose["x"] = 8
                   self.goal_pose["y"] = 2.5
                

                elif (self.robot_number == 1):
                   self.goal_pose["x"] = -1
                   self.goal_pose["y"] = 7
                

                elif(self.robot_number == 2):
                   self.goal_pose["x"] = 5.5
                   self.goal_pose["y"] = -7.5
                

                elif(self.robot_number == 3):
                   self.goal_pose["x"] = -4
                   self.goal_pose["y"] = -11.5

            elif(self.world_file_name == "zigzag_3ped"):
               self.goal_pose["x"] = 12.5
               self.goal_pose["y"] = 0
        
            elif self.world_file_name == "wallped3_8":
                self.goal_pose["x"] = -9
                self.goal_pose["y"] = 0

            elif(self.world_file_name == "zigzag_static"):
               self.goal_pose["x"] = 5
               self.goal_pose["y"] = -9

            elif(self.world_file_name == "4_robot_3D1P" or self.world_file_name == "train2" ):
                if (self.robot_number == 0):
                   self.goal_pose["x"] = 14.81
                   self.goal_pose["y"] = 0.24
                

                elif (self.robot_number == 1):
                   self.goal_pose["x"] = 15.0
                   self.goal_pose["y"] = 12.13
                

                elif(self.robot_number == 2):
                   self.goal_pose["x"] = -24.23
                   self.goal_pose["y"] = 11.14
                

                elif(self.robot_number == 3):
                   self.goal_pose["x"] = -24.39
                   self.goal_pose["y"] = 1.021


    def _get_distance2goal(self):
        """ Gets the distance to the goal
        """
        return math.sqrt((self.goal_pose["x"] - self.odom_dict["x"])**2 + (self.goal_pose["y"] - self.odom_dict["y"])**2)


    def viz_obs(self):
        max_v_num = numpy.max(self.v_matrix)
        max_w_num = numpy.max(self.w_matrix)
        max_cost_num = numpy.max(self.cost_matrix)
        max_obst_cost = numpy.max(self.obst_cost_matrix)
        max_toGoal_cost = numpy.max(self.to_goal_cost_matrix)

        v_normalized = (self.v_matrix / max_v_num) * 255
        v_normalized = v_normalized.astype(numpy.uint8)
        w_normalized = (self.w_matrix / max_w_num) * 255
        w_normalized = w_normalized.astype(numpy.uint8)
        cost_normalized = (self.cost_matrix / max_cost_num) * 255
        cost_normalized = cost_normalized.astype(numpy.uint8)

        obst_normalized = (self.obst_cost_matrix / max_obst_cost) * 255
        obst_normalized = obst_normalized.astype(numpy.uint8)

        toGoal_normalized = (self.to_goal_cost_matrix / max_toGoal_cost) * 255
        toGoal_normalized = toGoal_normalized.astype(numpy.uint8)
        
        cv2.imwrite("../observation_visualization/v/v_matrix_"+str(self.episode_num)+"_"+str(self.counter)+".jpg", v_normalized)
        cv2.imwrite("../observation_visualization/w/w_matrix_"+str(self.episode_num)+"_"+str(self.counter)+".jpg", w_normalized)
        cv2.imwrite("../observation_visualization/cost/cost_matrix_"+str(self.episode_num)+"_"+str(self.counter)+".jpg", cost_normalized)
        cv2.imwrite("../observation_visualization/obst/obst_matrix_"+str(self.episode_num)+"_"+str(self.counter)+".jpg", obst_normalized)
        cv2.imwrite("../observation_visualization/toGoal/toGoal_matrix_"+str(self.episode_num)+"_"+str(self.counter)+".jpg", toGoal_normalized)
        self.counter = self.counter + 1

    def _init_env_variables(self):
        """
        Inits variables needed to be initialised each time we reset at the start
        of an episode.
        :return:
        """
        # For Info Purposes
        self.cumulated_reward = 0.0
        # Set to false Done, because its calculated asyncronously
        self._episode_done = False
        
        # We wait a small ammount of time to start everything because in very fast resets, laser scan values are sluggish
        # and sometimes still have values from the prior position that triguered the done.
        time.sleep(1.0)
        
        # TODO: Add reset of published filtered laser readings
        laser_scan = self.get_laser_scan()
        discretized_ranges = laser_scan.ranges

        odom_data_init = self.get_odom()
        odom_dict_init = {}
        q = Quaternion(odom_data_init.pose.pose.orientation.w,odom_data_init.pose.pose.orientation.x,odom_data_init.pose.pose.orientation.y,odom_data_init.pose.pose.orientation.z)
        e = q.to_euler(degrees=False)
        odom_dict_init["x"] = odom_data_init.pose.pose.position.x
        odom_dict_init["y"] = odom_data_init.pose.pose.position.y
        odom_dict_init["theta"] = e[2]
        odom_dict_init["u"] = odom_data_init.twist.twist.linear.x
        odom_dict_init["omega"] = odom_data_init.twist.twist.angular.z
        cnfg = Config(odom_dict_init, self.goal_pose)
        obs_init = Obstacles(laser_scan.ranges, cnfg)
        self.obs_list_stacked = numpy.column_stack((obs_init.obst for _ in range(0, self.n_stacked_frames)))

        self.counter = 1
        self.episode_num = self.episode_num + 1

        init_obs = self._get_obs()
        self.previous_distance2goal = self._get_distance2goal()
        self.publish_filtered_laser_scan(   laser_original_data=laser_scan,
                                         new_filtered_laser_range=discretized_ranges)
        self.total_collisions += self.episode_collisions
        print("Total number of collsions ------------ {}".format(self.total_collisions))
        self.episode_collisions = 0
        self.n_skipped_count = 0

        file = open('VelList.csv', 'w')
        file_ang = open('angularVelList.csv', 'w')
        with file:
            write = csv.writer(file)
            write.writerows(self.list_vel)
            write_ang = csv.writer(file_ang)
            write_ang.writerows(self.list_angular_vel)

        self.list_vel.append(["lower_limit", "upper_limit", "current_velocity", "Violation_Status"])
        self.list_angular_vel.append(["lower_limit", "upper_limit", "current_velocity", "Violation_Status"])
        

    def _set_action(self, action):
        """
        This set action will Set the linear and angular speed of the turtlebot2
        based on the action number given.
        :param action: The action integer that set s what movement to do next.
        """
        
        rospy.logdebug("Start Set Action ==>"+str(action))
        # We convert the actions to speed movements to send to the parent class CubeSingleDiskEnv
        
        linear_speed = self.v_matrix[:,self.n_stacked_frames - 1][action]
        angular_speed = self.w_matrix[:,self.n_stacked_frames - 1][action]
        self.last_action = linear_speed

        linear_acc_limit = 2.5
        angular_acc_limit = 3.2
        del_t = 0.1

        max_reachable_vel_x = (self.odom_dict["u"] + (linear_acc_limit * del_t))
        min_reachable_vel_x = (self.odom_dict["u"] - (linear_acc_limit * del_t))
        max_reachable_vel_w =  self.odom_dict["omega"] + (angular_acc_limit * del_t)
        min_reachable_vel_w =  self.odom_dict["omega"] - (angular_acc_limit * del_t)

        if (linear_speed > max_reachable_vel_x) or (linear_speed < min_reachable_vel_x):
            self.list_vel.append([min_reachable_vel_x,max_reachable_vel_x, linear_speed, False])
        if (linear_speed < max_reachable_vel_x) and (linear_speed > min_reachable_vel_x):
            self.list_vel.append([min_reachable_vel_x,max_reachable_vel_x, linear_speed, True])
        if (angular_speed > max_reachable_vel_w) or (angular_speed < min_reachable_vel_w):
            self.list_angular_vel.append([min_reachable_vel_w, max_reachable_vel_w, False])
        if (angular_speed < max_reachable_vel_w) and (angular_speed > min_reachable_vel_w):
            self.list_angular_vel.append([min_reachable_vel_w, max_reachable_vel_w, True])


        
        # We tell TurtleBot2 the linear and angular speed to set to execute
        self.move_base( linear_speed,
                        angular_speed,
                        epsilon=0.05,
                        update_rate=10,
                        min_laser_distance=self.min_range)
        
        rospy.logdebug("END Set Action ==>"+str(action)+", NAME="+str(self.last_action))

    def _get_obs(self):
        """
        Here we define what sensor data defines our robots observations
        To know which Variables we have acces to, we need to read the
        TurtleBot2Env API DOCS
        :return:
        """
        start_tt =time.time()
        rospy.logdebug("Start Get Observation ==>")
        # We get the laser scan data
        laser_scan = self.get_laser_scan()
        rospy.logdebug("BEFORE DISCRET _episode_done==>"+str(self._episode_done))
        
        discretized_observations = self.discretize_observation( laser_scan,
                                                                self.new_ranges
                                                                )


        rospy.logdebug("Observations==>"+str(discretized_observations))
        rospy.logdebug("AFTER DISCRET_episode_done==>"+str(self._episode_done))
        rospy.logdebug("END Get Observation ==>")
        discretized_observations = numpy.asarray(discretized_observations)

        # New code for getting observations based on DWA
        odom_data = self.get_odom()
        self.odom_dict = {}
        self._reached_goal = False
        q = Quaternion(odom_data.pose.pose.orientation.w,odom_data.pose.pose.orientation.x,odom_data.pose.pose.orientation.y,odom_data.pose.pose.orientation.z)
        e = q.to_euler(degrees=False)
        self.odom_dict["x"] = odom_data.pose.pose.position.x
        self.odom_dict["y"] = odom_data.pose.pose.position.y
        self.odom_dict["theta"] = e[2]
        self.odom_dict["u"] = odom_data.twist.twist.linear.x
        self.odom_dict["omega"] = odom_data.twist.twist.angular.z
        cnfg = Config(self.odom_dict, self.goal_pose)
        self.obs = Obstacles(laser_scan.ranges, cnfg)

        if(self.n_skipped_count == 0):
            self.obs_list_stacked = numpy.delete(self.obs_list_stacked, (0,1), 1)
            self.obs_list_stacked = numpy.append(self.obs_list_stacked, self.obs.obst, 1)
            self.n_skipped_count += 1

        elif(self.n_skipped_count < self.n_skipped_frames):
            self.obs_list_stacked[:, ((2*self.n_stacked_frames)-2):((2*self.n_stacked_frames))] = self.obs.obst
            self.n_skipped_count += 1

        elif(self.n_skipped_count == self.n_skipped_frames):
            self.obs_list_stacked[:, ((2*self.n_stacked_frames)-2):((2*self.n_stacked_frames))] = self.obs.obst
            self.n_skipped_count = 0
        

        # print("The stacked obs list {}".format(self.obs_list_stacked))
        # print("The stacked obs list part {}".format(self.obs_list_stacked[:5,:]))

        self.v_matrix, self.w_matrix, self.cost_matrix, self.obst_cost_matrix, self.to_goal_cost_matrix = DWA(cnfg, self.obs_list_stacked, self.n_stacked_frames)

        # print("The w_matrix after {}".format(self.w_matrix[:5,:]))
        # print("The w_matrix {}".format(self.w_matrix[:,self.n_stacked_frames - 1]))
        # print("The cost_matrix {}".format(self.cost_matrix[:,self.n_stacked_frames - 1]))

        if (self.visualize_obs == True) and (self.robot_number == 0):
        	self.viz_obs()

        # self.stacked_obs = numpy.stack((self.v_matrix, self.w_matrix, self.cost_matrix), axis=2)
       

        self.stacked_obs = numpy.stack((self.v_matrix, self.w_matrix, self.obst_cost_matrix, self.to_goal_cost_matrix), axis=2)
        # self.obst_cost_matrix = self.obst_cost_matrix.flatten('F')
        # self.to_goal_cost_matrix = self.to_goal_cost_matrix.flatten('F')

        # self.stacked_obs = numpy.concatenate((self.obst_cost_matrix, self.to_goal_cost_matrix), axis=0)
        # self.stacked_obs = numpy.expand_dims(self.stacked_obs, axis=1)
         

        self.current_distance2goal = self._get_distance2goal()
        if (self.current_distance2goal < 0.5):
            self._episode_done = True
            self._reached_goal = True


        return self.stacked_obs
        

    def _is_done(self, observations):
        
        if self._episode_done and (not self._reached_goal):
            rospy.logdebug("TurtleBot2 is Too Close to wall==>"+str(self._episode_done))
            

        elif self._episode_done and self._reached_goal:
            rospy.logdebug("Robot {} reached the goal".format(self.robot_number))
            
        else:
            rospy.logdebug("TurtleBot2 is Ok ==>")

        return self._episode_done

    def _compute_reward(self, observations, done):

        reward = 0


        reward += 200*(self.previous_distance2goal - self.current_distance2goal)

        self.previous_distance2goal = self.current_distance2goal

        if self.last_action != 0:
            reward += self.forwards_reward

        
        if self._episode_done and (not self._reached_goal):
            reward += -1*self.end_episode_points
            self.goal_reaching_status.data = False
            self.goal_reaching_status_pub.publish(self.goal_reaching_status)

        elif self._episode_done and self._reached_goal:
            reward += self.goal_reaching_points
            self.goal_reaching_status.data = True
            self.goal_reaching_status_pub.publish(self.goal_reaching_status)

        # Danger of collision cost
        if self.select_collision_danger_cost:
            reward += self.collision_danger_cost

        rospy.logdebug("reward=" + str(reward))
        self.cumulated_reward += reward
        rospy.logdebug("Cumulated_reward=" + str(self.cumulated_reward))
        self.cumulated_steps += 1
        rospy.logdebug("Cumulated_steps=" + str(self.cumulated_steps))
        
        return reward

    # Internal TaskEnv Methods
    
    def discretize_observation(self,data,new_ranges):
        """
        Discards all the laser readings that are not multiple in index of new_ranges
        value.
        """
        self._episode_done = False
        
        discretized_ranges = []
        filtered_range = []
        #mod = len(data.ranges)/new_ranges
        mod = new_ranges
        
        max_laser_value = data.range_max
        min_laser_value = data.range_min
        
        rospy.logdebug("data=" + str(data))
        rospy.logwarn("mod=" + str(mod))
        self.collision_danger_cost = 0

        for i, item in enumerate(data.ranges):
            if (i%mod==0):
                if item == float ('Inf') or numpy.isinf(item):
                    #discretized_ranges.append(self.max_laser_value)
                    discretized_ranges.append(round(max_laser_value,self.dec_obs))
                elif numpy.isnan(item):
                    #discretized_ranges.append(self.min_laser_value)
                    discretized_ranges.append(round(min_laser_value,self.dec_obs))
                else:
                    #discretized_ranges.append(int(item))
                    discretized_ranges.append(round(item,self.dec_obs))
                    if item > self.closeness_threshold:
                        self.collision_danger_cost += self.prox_penalty1 / round(item,self.dec_obs)
                    else:
                        self.collision_danger_cost += self.prox_penalty2 / round(item,self.dec_obs)

                if (self.min_range > item > 0):
                    rospy.logerr("done Validation >>> item=" + str(item)+"< "+str(self.min_range))

                    if not self._episode_done:
                        self.episode_collisions += 1
                    self._episode_done = True
                else:
                    rospy.logwarn("NOT done Validation >>> item=" + str(item)+"< "+str(self.min_range))
                # We add last value appended
                filtered_range.append(discretized_ranges[-1])
            else:
                # We add value zero
                filtered_range.append(0.1)
                    
        rospy.logdebug("Size of observations, discretized_ranges==>"+str(len(discretized_ranges)))
        
        
        self.publish_filtered_laser_scan(   laser_original_data=data,
                                            new_filtered_laser_range=discretized_ranges)
        
        return discretized_ranges
        
    
    def publish_filtered_laser_scan(self, laser_original_data, new_filtered_laser_range):
        
        rospy.logdebug("new_filtered_laser_range==>"+str(new_filtered_laser_range))
        
        laser_filtered_object = LaserScan()

        h = Header()
        h.stamp = rospy.Time.now() # Note you need to call rospy.init_node() before this will work
        h.frame_id = laser_original_data.header.frame_id
        
        laser_filtered_object.header = h
        laser_filtered_object.angle_min = laser_original_data.angle_min
        laser_filtered_object.angle_max = laser_original_data.angle_max
        
        new_angle_incr = abs(laser_original_data.angle_max - laser_original_data.angle_min) / len(new_filtered_laser_range)
        
        #laser_filtered_object.angle_increment = laser_original_data.angle_increment
        laser_filtered_object.angle_increment = new_angle_incr
        laser_filtered_object.time_increment = laser_original_data.time_increment
        laser_filtered_object.scan_time = laser_original_data.scan_time
        laser_filtered_object.range_min = laser_original_data.range_min
        laser_filtered_object.range_max = laser_original_data.range_max
        
        laser_filtered_object.ranges = []
        laser_filtered_object.intensities = []
        for item in new_filtered_laser_range:
            if item == 0.0:
                laser_distance = 0.1
            else:
                laser_distance = item
            laser_filtered_object.ranges.append(laser_distance)
            laser_filtered_object.intensities.append(item)
        
        
        self.laser_filtered_pub.publish(laser_filtered_object)

