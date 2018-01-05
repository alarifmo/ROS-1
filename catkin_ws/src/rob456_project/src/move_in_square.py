#!/usr/bin/env python
#import 
import roslib
import rospy
import math
import tf
from nav_msgs.msg import Odometry
pi = math.pi
from tf.transformations import euler_from_quaternion

#the geometry mesgs Twist msg
from geometry_msgs.msg import Twist

#the move base result messages
from move_base_msgs.msg import MoveBaseActionResult
from nav_msgs.msg import OccupancyGrid
#Global Varibleles
Map = None
StateX = None
StateY = None
Resolution= None
X=None
Y=None
change=0

#Function to get the map from the node /map.data
def callback(occ_map):
	global Map
	global X
	global Y
	X = occ_map.info.origin.position.x
	Y = occ_map.info.origin.position.y
	Map = occ_map.data
#Odemetry function to get the location of the robot and use that information to convert between the map frame and global frame
def odometryCb(msg):
	global StateX
	global StateY
	global Angle
	global Resolution
	Resolution = 0.0500000007451
	StateX = msg.pose.pose.position.x/Resolution
	StateY = msg.pose.pose.position.y/Resolution
	C = msg.pose.pose.orientation
	Angle = [C.x, C.y, C.z, C.w]
#the functiion for sending waypoints to the robot
def mb_callback(msg):
	global Map
	global StateX
	global StateY
	global Angle
	global X
	global Y
	global change
	#check if the robot reaches its goal
	if msg.status.status == 2 or msg.status.status ==4 or msg.status.status==5 or msg.status.status==6:
		print "Robot failed to reach the waypoint"
	elif msg.status.status==3:
		print "Robot successfully reached the waypoint"

	#getting the location of the robot
	grid_x = int((StateX*Resolution - X) / Resolution)
	grid_y = int((StateY*Resolution - Y) / Resolution)

	#getting the location of the robot through a different method
	Current_Global_Coordinate_X = round(StateX + 2000)
	Current_Global_Coordinate_Y = round(StateY + 2000)
	#results match	
	print [Current_Global_Coordinate_X, Current_Global_Coordinate_Y]
	print ["True", grid_x, grid_y]
	

	#checking for obstacles
	#check in all direcation, if there is an obstacle return 1 and skip that way point
	def obs(i):
		global Map
		k=0
		
		for z in range (160):

			if Map[i+z]>20:
				k=1

			if Map[i-z]>20:
				k=1 
			if Map[i-(4000*z)]>20:
				k=1

			if  Map[i+(4000*z)]>20:
				k=1

		

		return k

	#check for Frontier points, if the point is not in the frontier then it will be skipped		
	def Frontier(i):
		global Map
		T = 0
		if Map[i] == 0:
			if Map[i - 1] == -1:
				T = 1
			elif Map[i - 4000] == -1:
				T = 1
			elif Map[i + 1] == -1:
				T = 1
			elif Map[i + 4000] == -1:
				T = 1
			else:
				T = 0
		return T
	#size of the frontier and check for obstacles too, you can control the size of the frontier when you call the function
	def Check_For_Obstacles_And_Size_Of_Frontier(L):
		global Map
		M = -36009
		T = 0
		while M < 36010:
			T = T + Frontier(L + M)
			if int(repr(M)[-1]) == 9 and M > 0:
				if Map[L + M] > 90:
					T = 0
					break
				else:
					M = M + 3994
			elif int(repr(M)[-1]) == 1 and M < 0:
				if Map[L + M] > 90:
					T = 0
					break
				else:
					M = M + 3994
			else:
				if Map[L + M] > 90:
					T = 0
					break
				else:
					M = M + 1
		return T				#  The bigger the T value the bigger the frontier

	#if the robot failed to reach goal, then move it a little bit; it mean its stuck 
	if  msg.status.status!=3:
		waypoint= Twist()
		(roll, pitch, yaw) = euler_from_quaternion(Angle)
		A = ((grid_x- 2) - Current_Global_Coordinate_X)*0.0500000007451
		B = ((grid_y -2) - Current_Global_Coordinate_Y)*0.0500000007451
		Xprime = round(A*math.cos(yaw) + B*math.sin(yaw))
		Yprime = round(B*math.cos(yaw) - A*math.sin(yaw)) 



	#each 5 waypoints the program will shift, Here it will start from (0,0)
	elif change<=5:

		waypoint= Twist()
		#Main Decision Making Program
		i = 0
		while i < len(Map):
			if Map[i] == 0:  #look for the first knowing empty location in the map
				if Map[i - 1] == -1:
					if Check_For_Obstacles_And_Size_Of_Frontier(i) > 15 and obs(i)==0:
						break
					else:
						i = i + 1
				elif Map[i - 4000] == -1:
					if Check_For_Obstacles_And_Size_Of_Frontier(i) > 15 and obs(i)==0:
						break
					else:
						i = i + 1
				elif Map[i + 1] == -1:
					if Check_For_Obstacles_And_Size_Of_Frontier(i) > 15 and obs(i)==0:
						break
					else:
						i = i + 1
				elif Map[i + 4000] == -1:
					if Check_For_Obstacles_And_Size_Of_Frontier(i) > 15 and obs(i)==0:
						break
					else:
						i = i + 1
				else:
					i = i + 1
			else:
				i = i + 1
		Global_Waypoint_Y = round(i/4000)
		Global_Waypoint_X = i - Global_Waypoint_Y*4000
			

		(roll, pitch, yaw) = euler_from_quaternion(Angle)

		A = (Global_Waypoint_X - Current_Global_Coordinate_X)*0.0500000007451
		B = (Global_Waypoint_Y - Current_Global_Coordinate_Y)*0.0500000007451
		Xprime = round(A*math.cos(yaw) + B*math.sin(yaw))
		Yprime = round(B*math.cos(yaw) - A*math.sin(yaw)) 
		print A
		print B
		print Global_Waypoint_X
		print Global_Waypoint_Y
		waypoint.linear.x = Xprime
		waypoint.linear.y = Yprime
		waypoint.angular.z = 0.0
		change=change+1
		

	#Here the search will start from (3999,3999)
	if change>5:
		waypoint= Twist()
		#Main Decision Making Program

		i = 15999999
		while i < len(Map):
			if Map[i] == 0: #find the first empty location in the map, start from bottom of the list.
				if Map[i - 1] == -1:
					if Check_For_Obstacles_And_Size_Of_Frontier(i) > 20 and obs(i)==0:
						break
					else:
						i = i - 1
				elif Map[i - 4000] == -1:
					if Check_For_Obstacles_And_Size_Of_Frontier(i) > 20 and obs(i)==0:
						break
					else:
						i = i - 1
				elif Map[i + 1] == -1:
					if Check_For_Obstacles_And_Size_Of_Frontier(i) > 20 and obs(i)==0:
						break
					else:
						i = i - 1
				elif Map[i + 4000] == -1:
					if Check_For_Obstacles_And_Size_Of_Frontier(i) > 20 and obs(i)==0 :
						break
					else:
						i = i - 1
				else:
					i = i - 1
			else:
				i = i - 1
		Global_Waypoint_Y = round(i/4000)
		Global_Waypoint_X = i - Global_Waypoint_Y*4000



		(roll, pitch, yaw) = euler_from_quaternion(Angle)
	
		A = (Global_Waypoint_X - Current_Global_Coordinate_X)*0.0500000007451
		B = (Global_Waypoint_Y - Current_Global_Coordinate_Y)*0.0500000007451
		Xprime = round(A*math.cos(yaw) + B*math.sin(yaw)) - 1
		Yprime = round(B*math.cos(yaw) - A*math.sin(yaw)) - 3
		print A
		print B
		print Global_Waypoint_X
		print Global_Waypoint_Y
		change=change+1
		if change==10:
			change=0

	


	waypoint.linear.x = Xprime
	waypoint.linear.y = Yprime
	waypoint.angular.z = 0.0

	pub.publish(waypoint)

if __name__=="__main__":
	#initilize the node
	rospy.init_node('move_in_squre')

	#publish waypoint sata to robot
	pub= rospy.Publisher('/base_link_goal',Twist,queue_size=10)

	#subscriber to move base result
	sub=rospy.Subscriber('/move_base/result', MoveBaseActionResult,mb_callback)
	
	sub2=rospy.Subscriber('/map', OccupancyGrid, callback)

	sub3=rospy.Subscriber('odom', Odometry, odometryCb)
	

	
	#turn control
	rospy.spin()
