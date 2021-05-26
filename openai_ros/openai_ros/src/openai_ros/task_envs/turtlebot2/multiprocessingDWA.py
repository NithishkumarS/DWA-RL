import rospy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from squaternion import Quaternion
from config import Config
from obstacles import Obstacles
from dwa import DWA
import numpy as np
import concurrent.futures
import time



#[odom, goal, laser_scan, robot_number ]
def dwa_wrapper(final_list):

	odom_dec = {}
	q = Quaternion(final_list[0].pose.pose.orientation.w,final_list[0].pose.pose.orientation.x,final_list[0].pose.pose.orientation.y,final_list[0].pose.pose.orientation.z)
	e = q.to_euler(degrees=False)
	odom_dec["x"] = final_list[0].pose.pose.position.x
	odom_dec["y"] = final_list[0].pose.pose.position.y
	odom_dec["theta"] = e[2]
	odom_dec["u"] = final_list[0].twist.twist.linear.x
	odom_dec["omega"] = final_list[0].twist.twist.angular.z



	cnfg = Config(odom_dec, final_list[1])
	obs = Obstacles(final_list[2].ranges, cnfg)
	v_list, w_list, cost_list = DWA(cnfg, obs)


	

	return v_list, w_list, cost_list, final_list[3]
	




def main():
	r = rospy.Rate(20)
	while not rospy.is_shutdown():
		
		

		goal1 = {}
		goal1["x"] = -1
		goal1["y"] = 7

		goal2 = {}
		goal2["x"] = 5.5
		goal2["y"] = -7.5

		goal3 = {}
		goal3["x"] = 8
		goal3["y"] = 2.5

		goal4 = {}
		goal4["x"] = -4
		goal4["y"] = -11.5



		odom1 = rospy.wait_for_message("/turtlebot1/ground_truth/state", Odometry, timeout=5.0)
		laser_scan1 = rospy.wait_for_message("/turtlebot1/scan_filtered", LaserScan, timeout=5.0)
		
		odom2 = rospy.wait_for_message("/turtlebot2/ground_truth/state", Odometry, timeout=5.0)
		laser_scan2 = rospy.wait_for_message("/turtlebot2/scan_filtered", LaserScan, timeout=5.0)

		odom3 = rospy.wait_for_message("/turtlebot0/ground_truth/state", Odometry, timeout=5.0)
		laser_scan3 = rospy.wait_for_message("/turtlebot0/scan_filtered", LaserScan, timeout=5.0)

		odom4 = rospy.wait_for_message("/turtlebot3/ground_truth/state", Odometry, timeout=5.0)
		laser_scan4 = rospy.wait_for_message("/turtlebot3/scan_filtered", LaserScan, timeout=5.0)
		

		l1 = []
		l2 = []
		l3 = []
		l4 = []
		l1.append(odom1)
		l1.append(goal1)
		l1.append(laser_scan1)
		l1.append(1)
		

		l2.append(odom2)
		l2.append(goal2)
		l2.append(laser_scan2)
		l2.append(2)

		l3.append(odom3)
		l3.append(goal3)
		l3.append(laser_scan3)
		l3.append(0)

		l4.append(odom4)
		l4.append(goal4)
		l4.append(laser_scan4)
		l4.append(3)
		

		final_list = []
		final_list.append(l1)
		final_list.append(l2)
		final_list.append(l3)
		final_list.append(l4)

		start_t = time.time()

		with concurrent.futures.ProcessPoolExecutor() as executor:
			
			results = executor.map(dwa_wrapper, final_list)
		
				
		# 		v_list, w_list, cost_list, index = result

		# 	# results = [executor.submit(dwa_wrapper, list_list) for list_list in final_list]
		# 	# for f in concurrent.futures.as_completed(results):

		# 		# v_list, w_list, cost_list, index = f.result()

		end_t = time.time()
		print("total time taken {} ".format(end_t - start_t))

		for result in results:
			v_list, w_list, cost_list, index = result



			minpos = np.argmin(cost_list)
			pub = rospy.Publisher("/turtlebot"+str(index)+"/cmd_vel_mux/input/navi", Twist, queue_size=1)
			speed = Twist()
			speed.linear.x = v_list[minpos]
			speed.angular.z = w_list[minpos]
			pub.publish(speed)
			r.sleep()
		
		


if __name__ == '__main__':
    rospy.init_node('dwa')
    main()