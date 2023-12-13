#!/usr/bin/env python3
import rospy
import numpy as np
from sensor_msgs.msg import LaserScan
from math import floor
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import math

# Time in ROS
def time():
	return rospy.get_rostime().secs + (rospy.get_rostime().nsecs/1e9)

# Move robot
def move(v_x,rot):
	# Topic to move
	pub = rospy.Publisher('/robot/cmd_vel', Twist, queue_size=10)
	#rospy.init_node('movement_robot', anonymous=True)
	#rospy.sleep(1)
	t = Twist()
	t.linear.x = v_x
	t.linear.y = 0
	t.linear.z = 0
	t.angular.x = 0
	t.angular.y = 0
	t.angular.z = rot
	pub.publish(t)
	

def quaternion_to_euler_angle_vectorized1(w, x, y, z):
    ysqr = y * y

    t0 = +2.0 * (w * x + y * z)
    t1 = +1.0 - 2.0 * (x * x + ysqr)
    X = np.degrees(np.arctan2(t0, t1))

    t2 = +2.0 * (w * y - z * x)
    t2 = np.where(t2>+1.0,+1.0,t2)
    #t2 = +1.0 if t2 > +1.0 else t2

    t2 = np.where(t2<-1.0, -1.0, t2)
    #t2 = -1.0 if t2 < -1.0 else t2
    Y = np.degrees(np.arcsin(t2))

    t3 = +2.0 * (w * z + x * y)
    t4 = +1.0 - 2.0 * (ysqr + z * z)
    Z = np.degrees(np.arctan2(t3, t4))

    return X, Y, Z 
        
class Node:
	def __init__(self, n):
		self.x = n[0]
		self.y = n[1]

class Navigation:

	def __init__(self,p_start,p_goal,iter_max):
		self.s_start = Node(p_start)
		self.s_goal = Node(p_goal)
		self.laser_float = [0] * 720
		self.delta_theta = 5
		self.dist_wall = 5
		self.iter_max = iter_max
		self.v_x = 0.5 # forward velocity
		self.rot = 0.2 # rotation velocity
		self.s_now = Node(p_start)
		self.odom_position = 0
		self.odom_theta = 0
		self.dist_total = 0
		self.theta_total = 0
		self.theta_new = 0
		self.dist_local = 0
		self.planning_flag = True
		self.start_move_forward = True
		self.time_start_forward = 0
		self.time_current = 0
		
		
	def get_distance_and_angle(self,node_start, node_end):
        	dx = node_end.x - node_start.x
        	dy = node_end.y - node_start.y
        	return math.hypot(dx, dy), math.atan2(dy, dx)*180/math.pi + 180
		
	# get odometry messages
	def callback_odometry(self,msg):
		self.odom_position = (msg.pose.pose.position.x, msg.pose.pose.position.y)
		self.s_now = Node(self.odom_position)
		
		_,_,self.odom_theta = quaternion_to_euler_angle_vectorized1(msg.pose.pose.orientation.w,msg.pose.pose.orientation.x,msg.pose.pose.orientation.y,msg.pose.pose.orientation.z)
		
		self.odom_theta = self.odom_theta + 180
		
		
	# trajectory planning
	def planning(self):
		self.dist_total, self.theta_total = self.get_distance_and_angle(self.s_now,self.s_goal)
		
		self.dist_local = self.laser_float[int(self.theta_total)*2]
		
		
		
		
		if self.dist_local > self.dist_total:
			print("fuck you")
		
		else:
			self.theta_new = self.theta_total
			print("self.theta_new = " + str(self.theta_new))
			for i in range(self.iter_max):
				dist_local_minus = self.laser_float[int(self.theta_new - self.delta_theta)*2]
				dist_local_plus = self.laser_float[int(self.theta_new + self.delta_theta)*2]
				
				if dist_local_plus > dist_local_minus:
					self.theta_new = self.theta_new + self.delta_theta
					self.dist_local = dist_local_plus 
					print("self.theta_new = " + str(self.theta_new) + ", self.dist_local = " + str(self.dist_local))
					
				else:
					self.theta_new = self.theta_new - self.delta_theta
					self.dist_local = dist_local_minus
					print("self.theta_new = " + str(self.theta_new) + ", self.dist_local = " + str(self.dist_local))
					
				if self.dist_local > self.dist_wall:
					break
					
	# move robot
	def execute_movement(self):
		theta_error = self.odom_theta - self.theta_new
		if abs(theta_error) < 5:
			if self.start_move_forward:
				self.time_start_forward = time()
				self.start_move_forward = False
			move(self.v_x,0)
			self.time_current = time()
			if self.time_current > self.time_to_move_forward:
				self.planning_flag = True
				self.start_move_forward = True
				move(0,0)
		elif theta_error > 5:
			move(0,-self.rot)
		elif theta_error < -5:
			move(0,self.rot)
						
	# get the laser messages
	def callback_laser(self,msg):
		# 0 graus: behind the robot
		# 90 graus: right side of the robot
		# 180 graus: in front of the robo
		# 270 graus: left side of the robot
		dist_limit = 1.2 #minimum distance limit to move forward

		dist_check = 15
		difference_check = 30

		laser_raw = msg.ranges
		self.laser_float = [float(r) for r in laser_raw]
		min_dist = min(laser_raw) # minimum distance
		max_dist = max(laser_raw) # maximum distance
		angle_to_min = int((laser_raw.index(min(laser_raw)) + 1)/2) #angle to minimum distance
		angle_to_max = int((laser_raw.index(max(laser_raw)) + 1)/2) #angle to maximum distance
		angle_to_max_front = int((laser_raw.index(max(laser_raw[180:540])) + 1)/2) #angle to maximum distance
		angle_to_min_rear = int((laser_raw.index(min(min(laser_raw[0:180]),min(laser_raw[540:720]))) + 1)/2) #angle to maximum distance
	
		dist_front = laser_raw[360] #distance in front of robot
		dist_right = laser_raw[180] #distance right of the robot
		dist_left  = laser_raw[540] #distance left of the robot
	
		sum_dist_right = 0
		sum_dist_left = 0
		angle_med_right = 0
		angle_med_left = 0
		for i in range(180):
			if self.laser_float[i+180] > dist_check:
				self.laser_float[i+180] = dist_check
			if self.laser_float[i + 360] > dist_check:
				self.laser_float[i + 360] = dist_check
			
			sum_dist_right = sum_dist_right + self.laser_float[i+180]
			sum_dist_left  = sum_dist_left + self.laser_float[i+360]
			angle_med_right = angle_med_right + self.laser_float[i+180]*int((self.laser_float.index(self.laser_float[i+180]) + 1)/2)
			angle_med_left = angle_med_left + self.laser_float[i+360]*int((self.laser_float.index(self.laser_float[i+360]) + 1)/2)
		
		angle_med_right = angle_med_right/sum_dist_right - 90
		angle_med_left = 270 - angle_med_left/sum_dist_left
	
		sum_distance = sum_dist_right + sum_dist_left
		angle_distance = int((angle_med_right*sum_dist_right + angle_med_left*sum_dist_left)/sum_distance)
	
		difference = sum_dist_right - sum_dist_left
		
		if self.planning_flag:
			path = nav.planning()
			self.planning_flag = False
		
		
		nav.execute_movement()
		

		
		
	
if __name__ == '__main__':
	# Init ROS
	rospy.init_node("control_node", anonymous=True)
	
	p_start = (0, 0)  # Starting node
	p_goal = (10, 10)  # Goal node
	iter_max = 10
	
	nav = Navigation(p_start,p_goal,iter_max)
	
	rospy.Subscriber("/odometry/filtered", Odometry, nav.callback_odometry)
	rospy.Subscriber("/scan", LaserScan, nav.callback_laser)
	
	print("Entrei")
	rospy.spin() # this will block untill you hit Ctrl+C
   


