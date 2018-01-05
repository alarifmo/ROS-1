#!/usr/bin/env python
import time
import rospy
import math
import tf
from tf.transformations import euler_from_quaternion
import message_filters

# The laser scan message
from sensor_msgs.msg import LaserScan

# The odometry message
from nav_msgs.msg import Odometry

# the velocity command message
from geometry_msgs.msg import Twist

# instantiate global variables "globalOdom"
globalOdom = Odometry()

# global pi - this may come in handy
pi = math.pi





# method to control the robot
def callback(scan,odom):
# the odometry parameter should be global
	global globalOdom
	globalOdom = odom
	global gotpostion
	global up
	global down
	global right
	global left
	global gotDirection
	global goalTheta
	global negative
	global ready
	global obs
	global i
	global go
# make a new twist message
	command = Twist()

# Fill in the fields.  Field values are unspecified 
# until they are actually assigned. The Twist message 
# holds linear and angular velocities.
	command.linear.x = 0.0
	command.linear.y = 0.0
	command.linear.z = 0.0
	command.angular.x = 0.0
	command.angular.y = 0.0
	command.angular.z = 0.0

# get goal x and y locations from the launch file
	goalX = rospy.get_param('hw2/goalX',0.0)
	goalY = rospy.get_param('hw2/goalY',0.0)

# find current (x,y) position of robot based on odometry
	currentX = globalOdom.pose.pose.position.x
	currentY = globalOdom.pose.pose.position.y

# find current orientation of robot based on odometry (quaternion coordinates)
	xOr = globalOdom.pose.pose.orientation.x
	yOr = globalOdom.pose.pose.orientation.y
	zOr = globalOdom.pose.pose.orientation.z
	wOr = globalOdom.pose.pose.orientation.w

# find orientation of robot (Euler coordinates)
	(roll, pitch, yaw) = euler_from_quaternion([xOr, yOr, zOr, wOr])

# find currentAngle of robot (equivalent to yaw)
# now that you have yaw, the robot's pose is completely defined by (currentX, currentY, currentAngle)
	currentAngle = yaw

# find laser scanner properties (min scan angle, max scan angle, scan angle increment)
	maxAngle = scan.angle_max
	minAngle = scan.angle_min
	angleIncrement = scan.angle_increment

# find current laser angle, max scan length, distance array for all scans, and number of laser scans
	currentLaserTheta = minAngle
	maxScanLength = scan.range_max 
	distanceArray = scan.ranges
	numScans = len(distanceArray)

# the code below (currently commented) shows how 
# you can print variables to the terminal (may 
# be useful for debugging)
#print 'x: {0}'.format(currentX)
#print 'y: {0}'.format(currentY)
#print 'theta: {0}'.format(currentAngle)

# for each laser scan
	
	for curScan in range(0, numScans):
		# curScan (current scan) loops from 0 to 
		# numScans (length of vector containing laser range data)
		# for each laser scan, the angle is currentLaserTheta,
		# and the range is distanceArray[curScan]
		# ............................................
		# ..... insert code here which uses...........
		# ..... distanceArray[curScan] and ...........
		


		#the avoiding obstecales part		
		if currentLaserTheta > (0-0.2) and currentLaserTheta < (+0.2) :			
			
			#print 'ditance array = {0} '.format(distanceArray[curScan])
			#print 'current Angle= {0}'.format(currentAngle)
			#print ' Current laser theta = {0}'.format(currentLaserTheta)
			
			#if obs==0:

			if distanceArray[curScan] < 2.0 :
				#obs=1			
				#goFix=1;	
				go=0				
				ready=0;					
				gotDirection=0;
				gotpostion=0;
				i=0
				#for a in range(0,10):				
				command.angular.z = 1.5708
				pub.publish(command)
					#time.sleep(5)
							
				
						
				for a in range(0,100):	
					command.linear.x=100.00
					pub.publish(command)	
							
				
		# ......... currentLaserTheta.................
		# ............................................
		# after you are done using one laser scan, update 
		# the current laser scan angle before the for loop
		# is incremented
		currentLaserTheta = currentLaserTheta + angleIncrement	

		# based on the motion you want (found using goal location,
		# current location, and obstacle info), set the robot
		# motion, e.g.:
		# command.linear.x = 0.0
		# command.angular.z = 0.0
		
		
	#This part to let the robot go stright after it turns to avoid the obstecle untill i=100 then it will relocate
	#and the elif statemnt is to make the robot relocate its self each 200 i even if it didnt face obstecles
	i=i+1;

	if i==100: 
		#goFix=0;
		gotDirection=0;
		gotpostion=0;
		ready=1;

	elif i==300:
		#command.linear.x = 0.0
		i=99;
		gotDirection=0;
		gotpostion=0;
		ready=0;	
		up=0;
		down=0;
		right=0
		left=0;
		go=0;

	#*******************************8The code to get the right direction ***************************************
	
	if gotDirection==0 :
		
		diX=abs(currentX-goalX)
		diY=abs(currentY-goalY)
		goalTheta= math.atan2(diX,diY) # invers tan to get Theta
		gotDirection=1


	#*****************getting the location of the robot regarding the GOal ***************************
	if gotpostion==0 :	
				
		if currentX >goalX:
		 	right=1;
			left=0;		
		else:
			right=0;
			left=1;
		
		if currentY > goalY:
			up=1;
			down=0;
			
		else:
			up=0;
			down=1;
	
		
		gotpostion=1;
		reay=1;
	#********* MOST IMPORTANT PART********************************************************************
	#**********After getting the posotion here we make the robot turn toward the goal**********************888
	

	obs=obs+1;
	print 'obs= {0}'.format(obs)
	print 'i= {0}'.format(i)
	
	#-1
	if ready==1:
		if up==1 and left==1 :
			
			if currentAngle < (goalTheta-0.1)-1.5708 or currentAngle >(goalTheta+0.1)-1.5708 :
								
							
				command.angular.z =- 0.1
				pub.publish(command)				
				
	
			else:
		
		
				go=1
			
		#-2
		elif up==1 and right ==1 :
			#ready=0
			if currentAngle < (1.5708-goalTheta-0.1)-3.1416 or currentAngle >(1.5708-goalTheta+0.1)-3.1416 :
								
				command.angular.z = -0.1
				pub.publish(command)				
				
	
			else:
	
			
				go=1
		#-3
		elif down==1 and right ==1 :
						
			if currentAngle < (goalTheta-0.1)+1.5708 or currentAngle >(goalTheta+0.1)+1.5708:
							
				command.angular.z = -0.1
				pub.publish(command)				
				
	
			else:
	
			
				go=1
	
	

	#-4
		elif down==1 and left ==1 :
			
			if currentAngle < (1.5708-goalTheta-0.1) or currentAngle >1.5708-goalTheta+0.1:
								
							
				command.angular.z = -0.1
				pub.publish(command)				
				
	
			else:
	
			
				go=1
	
	
	

	# ***************************8the MOVMENT TOWARD THE GOAL PART *************************8
	if go==1 :
			
		
		command.linear.x= 0.3
		pub.publish(command)				
	elif i<99:
		command.linear.x= 0.5
		pub.publish(command)


	print ' current X= {0}'.format(currentX)
	print ' current Y= {0}'.format(currentY)
	
	print 'currentAngle = {0}'.format(currentAngle)	
	print 'the goal theta is {0}'.format(goalTheta)
	print ' ready = {0}'.format(ready)
	print 'go = {0} '.format(go)
	print'up= {0}'.format(up)
	print'down= {0}'.format(down)
	print'right= {0}'.format(right)
	print'left= {0}'.format(left)
	print'*********************************************'
	#pub.publish(command)				

	if currentX > goalX-0.8 and currentX <goalX+0.8 and currentY > goalY-0.8 and currentY <goalY+0.8 :

		print'reached the goal'

		while True:
			currentX=0
		

	


	







	
# main function call
if __name__ == "__main__":
# Initialize the node
	rospy.init_node('lab2', log_level=rospy.DEBUG)
	gotpostion= 0; #if this set to 1 then we are in the riguth dirctin
	gotDirection=0;	
	up=0;
	down=0;
	right=0
	left=0;
	B=0;
	negative=0;
	ready=0;
	obs=0;
	i=99;
	go=0
	goFix=0;
# subscribe to laser scan message
	sub = message_filters.Subscriber('base_scan', LaserScan)

# subscribe to odometry message    
	sub2 = message_filters.Subscriber('odom', Odometry)
	
# synchronize laser scan and odometry data
	ts = message_filters.TimeSynchronizer([sub, sub2], 10)
	ts.registerCallback(callback)
	
# publish twist message
	pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)
	
# Turn control over to ROS
	rospy.spin()
	
