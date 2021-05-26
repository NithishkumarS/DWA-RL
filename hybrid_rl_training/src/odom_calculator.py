#!/usr/bin/env python
import rospy
from tf.transformations import euler_from_quaternion
from std_msgs.msg import Float32
from std_msgs.msg import Int16
from geometry_msgs.msg import Pose
from nav_msgs.msg import Odometry
import time
import math
from tf.transformations import euler_from_quaternion, quaternion_from_euler


class odom_calulator():
    def __init__(self):

        rospy.init_node('OdomCalculator', anonymous=True)
        self.total_distance = rospy.Publisher("/total_distance",Float32, queue_size = 1)
        self.number_of_turns = rospy.Publisher("/number_of_turns",Int16, queue_size = 1)
        self.average_velocity = rospy.Publisher("/average_velocity",Float32, queue_size = 1)



        self.AverageVelocity = Float32()
        self.TotalDistance = Float32()


        self.current_location = Pose()
        self.previous_location = Pose()

        self.previous_angular_position = Float32()
        self.previous_angular_position = 0.0


        self.current_angular_position = Float32()
        self.current_angular_position = 0.0

        self.total_turns = 0
        self.previous_comparison = "neutral"
        self.current_comparison = "neutral"



        self.TotalDistance = 0

        self.current_time = 0
        self.previous_time = 0

        self.previous_location.position.x = 0.0
        self.previous_location.position.y = 0.0
        self.previous_location.position.z = 0.0
        odom_tmp = rospy.wait_for_message("/turtlebot0/ground_truth/state", Odometry, timeout=5.0)
        self.previous_location.position.x = odom_tmp.pose.pose.position.x
        self.previous_location.position.y = odom_tmp.pose.pose.position.y
        self.previous_location.position.z = odom_tmp.pose.pose.position.z
        self.current_location.position.x = 0.0
        self.current_location.position.y = 0.0
        self.current_location.position.z = 0.0

        self.previous_time = time.time()
        self.rate = rospy.Rate(10)

        self.location = rospy.Subscriber("/turtlebot0/ground_truth/state", Odometry, self.odometry_callback)
        rospy.spin()

    def odometry_callback(self,data):
        # print("-------------Start of Callback----------------")

        self.current_location.position.x = data.pose.pose.position.x
        self.current_location.position.y = data.pose.pose.position.y
        self.current_location.position.z = data.pose.pose.position.z
        self.current_time = time.time()
        time_dif = self.current_time - self.previous_time


        self.TotalDistance += math.sqrt(math.pow((self.current_location.position.x - self.previous_location.position.x),2) + math.pow((self.current_location.position.y - self.previous_location.position.y),2))
        self.total_distance.publish(self.TotalDistance)
        print("total distance is {}".format(self.TotalDistance))

        self.previous_location.position.x = self.current_location.position.x
        self.previous_location.position.y = self.current_location.position.y

        # x = data.pose.pose.orientation.x
        # y = data.pose.pose.orientation.y
        # z = data.pose.pose.orientation.z
        # w = data.pose.pose.orientation.w
        # orientation_list = [x,y,z,w]
        # (roll, pitch, self.current_angular_position) = euler_from_quaternion (orientation_list)
        # print("YAW {}".format(self.current_angular_position))
        # print("ROLL {}".format(roll))
        # print("PITCH".format(pitch))

        # if (self.current_angular_position - self.previous_angular_position  < -0.001 ):
        #     self.current_comparison = "negative"

        # elif (self.current_angular_position - self.previous_angular_position  > 0.001):
        #     self.current_comparison = "positive"

        # elif ((self.current_angular_position - self.previous_angular_position)  > -0.001 or ((self.current_angular_position - self.previous_angular_position)  < 0.001)):
        #     self.current_comparison = "neutral"

        # # print("current status {}".format(self.current_comparison))
        # # print("Previous status {}".format(self.previous_comparison))
        # if (self.current_comparison != self.previous_comparison ):
        #     print("current status {}".format(self.current_comparison))
        #     print("Previous status {}".format(self.previous_comparison))
        #     print("Truncated angular velocity values {}".format(truncate(self.current_angular_position)))

        #     self.total_turns += 1

        # else:
        #     self.total_turns = self.total_turns

        # self.previous_comparison = self.current_comparison
        # self.previous_angular_position = self.current_angular_position


        # print("Total turn {}".format(self.total_turns))


        # self.rate.sleep()
        # rospy.sleep(0.1)








            # self.rate.sleep()




if __name__ == '__main__':

    while not rospy.is_shutdown():
        d = odom_calulator()
