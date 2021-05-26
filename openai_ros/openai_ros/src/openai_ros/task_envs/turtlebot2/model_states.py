#!/usr/bin/env python

import rospy
import numpy as np

from gazebo_msgs.msg import ModelStates
from geometry_msgs.msg import Pose
from nav_msgs.msg import Odometry
import time
import math
import sys
sys.path.remove('/opt/ros/melodic/lib/python2.7/dist-packages')
from tf.transformations import euler_from_quaternion, quaternion_from_euler

# Global variables
ROB_POS_FLAG = False
rob_yaw = 0
ped_yaw = 0
rob_pos = np.array([])
ped_pos = np.array([])
rob_theta = 0


def check_relevancy(rel_x_rot, rel_y_rot, rel_ped_vec):
    # print(rel_x_rot, rel_y_rot)
    # print(rel_ped_vec)
    # The robot's vector is (1, 0)

    # Checking relevancy conditions
    # Check if obstacle is in front of the robot within sensing range
    if (rel_x_rot > 0.5 and rel_x_rot < 4.5 and rel_y_rot > -3 and rel_y_rot < 3):
        if (rel_y_rot > 0 and rel_ped_vec[1] < 0): # Obstacle approaching from left
            # print("Obstacle approaching from left")
            return True
        elif (rel_y_rot < 0 and rel_ped_vec[1] > 0): # Obstacle approaching from right
            # print("Obstacle approaching from right")
            return True
        elif (rel_ped_vec[0] < 0 and rel_ped_vec[1] >= -0.3 and rel_ped_vec[1] <= 0.3 and rel_y_rot < 0.3 and rel_y_rot > -0.3): # Head-on collision
            # print("Head on collision")
            return True
        elif (rel_ped_vec[0] > 0 and rel_ped_vec[1] >= -0.3 and rel_ped_vec[1] <= 0.3 and rel_y_rot < 0.3 and rel_y_rot > -0.3): # Straight ahead and moving away
            # print("Straight ahead. But beware")
            return True
        else:
            # print("Irrelevant")
            return False

    else:
        # print("Irrelevant")
        return False




def callback(msg):
    global ROB_POS_FLAG
    global rob_pos
    global rob_theta
    #  Relevant pedestrian positions and walking directions
    relevant_list = []
    rel_ped_poses = []
    rel_ped_vec = []

    for i in range(len(msg.name)):
        if (msg.name[i][0:6] == "turtle"):
            x = msg.pose[i].orientation.x
            y = msg.pose[i].orientation.y
            z = msg.pose[i].orientation.z
            w = msg.pose[i].orientation.w
            orientation_list = [x,y,z,w]

            (_, _, rob_theta) = euler_from_quaternion (orientation_list)
            # rob_vec = np.array([math.cos(rob_theta), math.sin(rob_theta)])

            x = msg.pose[i].position.x
            y = msg.pose[i].position.y
            rob_pos = np.array([x, y])

            if(ROB_POS_FLAG == False):
                ROB_POS_FLAG = True
                print ("Obtained Robot's position")


        # Get all pedestrian vectors and positions
        if(ROB_POS_FLAG == True):

            if (msg.name[i][0:5] == "actor"):

                x = msg.pose[i].orientation.x
                y = msg.pose[i].orientation.y
                z = msg.pose[i].orientation.z
                w = msg.pose[i].orientation.w
                orientation_list = [x,y,z,w]
                print("orientation list: ", orientation_list)

                # NOTE: Always check which side of the gazebo model is defined as front. For person_walking front of model is actually its backside
                (_, _, ped_theta) = euler_from_quaternion (orientation_list)
                ped_vec = np.array([math.cos(ped_theta), math.sin(ped_theta)])

                # Get pedestrian position wrt global coordinates
                x = msg.pose[i].position.x
                y = msg.pose[i].position.y
                ped_pos = np.array([x, y])

                # Compute relative position of ped wrt unrotated robot
                rel_x = ped_pos[0] - rob_pos[0]
                rel_y = ped_pos[1] - rob_pos[1]

                # Compute relative position of ped wrt rotated robot
                rel_x_rot = rel_x * math.cos(rob_theta) + rel_y * math.sin(rob_theta)
                rel_y_rot = -rel_x * math.sin(rob_theta) + rel_y * math.cos(rob_theta)

                # Compute relative yaw angle (orientation) of ped wrt robot
                rel_theta = ped_theta - rob_theta

                # compute unit vector of pedestrian relative to rotated robot
                rel_ped_vec = np.array([math.cos(rel_theta), math.sin(rel_theta)])

                # Checking relevancy
                relevant_result = check_relevancy(rel_x_rot, rel_y_rot, rel_ped_vec)

                if (relevant_result == True):
                    # Add current location to an array
                    relevant_list.append(msg.name[i])

    for j in range(len(relevant_list)):
        print(relevant_list[j])



if __name__ == '__main__':
    rospy.init_node('Model_State_Sub')
    rospy.Subscriber("/gazebo/model_states", ModelStates, callback)
    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()