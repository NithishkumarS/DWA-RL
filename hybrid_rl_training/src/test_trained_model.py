#!/usr/bin/env python

import sys
sys.path.remove('/opt/ros/melodic/lib/python2.7/dist-packages')
import gym
from stable_baselines.common.policies import FeedForwardPolicy, register_policy
from stable_baselines.common.vec_env import SubprocVecEnv
from stable_baselines import PPO2
sys.path.append('/opt/ros/melodic/lib/python2.7/dist-packages')
from openai_ros.task_envs.turtlebot2.turtlebot2_maze import TurtleBot2MazeEnv
import rospy
import os
from sensor_msgs.msg import LaserScan
import time
from std_msgs.msg import Float32, Bool
import csv
# Custom MLP policy of three layers of size 128 each
class CustomPolicy(FeedForwardPolicy):
    def __init__(self, *args, **kwargs):
        super(CustomPolicy, self).__init__(*args, **kwargs,
                                           net_arch=[dict(pi=[144, 144, 144],
                                                          vf=[144, 144, 144])],
                                           feature_extraction="mlp")

td = 0
def getDistance(msg):
    global td
    td = msg.data

gr = False
def getGoalReachingStatus(msg):
    global gr
    gr = msg.data

if __name__ == '__main__':
    world_file = sys.argv[1]
    number_of_robots = sys.argv[2]
    robot_number = sys.argv[3] # Provide robot number to subscribe to the correct topic  
    profiling = True
    max_steps = 900
    max_test_episodes = 50
    min_range = 0.5 # Refer Task environment to get the value of min range
    rospy.init_node('stable_training', anonymous=True, log_level=rospy.WARN)
    TotalDistance = rospy.Subscriber("/total_distance",Float32, getDistance)
    goal_reaching_status = rospy.Subscriber('/turtlebot'+str(robot_number)+'/goal_reaching_status', Bool, getGoalReachingStatus)
    env_temp = TurtleBot2MazeEnv
    env = SubprocVecEnv([lambda k=k:env_temp(world_file, k) for k in range(int(number_of_robots))])

    model = PPO2.load("ppo2_turtlebot_tr#3")

    counter = 0
    collisions = 0
    start_time = rospy.get_time()
    episode_time_dist_list = []
    start_td = td
    goal_reached = False
    while(counter < max_test_episodes):
        obs = env.reset()
        # Evaluate the agent
        episode_reward = 0
        for _ in range(max_steps):
            action, _ = model.predict(obs)
            td_before_reset = td
            obs, reward, done, info = env.step(action)
            episode_reward += reward
            goal_reached = gr
            if (done):
                print("Done")
                counter += 1
                if profiling:
                    total_time_episode = rospy.get_time() - start_time
                    total_distance_episode = td_before_reset - start_td
                    print("The total time is {}".format(total_time_episode))
                    print("The distance travelled {}".format(total_distance_episode))
                    episode_time_dist_list.append([total_time_episode, total_distance_episode, goal_reached])
                    start_time = rospy.get_time()
                    start_td = td
                break

    print("Total number of collisions {}".format(collisions))
    file = open('trained_model_data_'+str(world_file)+'.csv', 'w')
    with file:     
      write = csv.writer(file) 
      write.writerows(episode_time_dist_list)

        
