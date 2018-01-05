#!/usr/bin/env python

import rospy
import math
import tf
from tf.transformations import euler_from_quaternion
import message_filters

#The laser scan message
from sensor_msgs.msg import LaserScan

#The odemetry
from nav_msgs.msg import Odemetry

#The velocity 
from geometry_msgs.msg import Twist

class guidancesControl:

	#global pi	
	pi=math.pi

	#This class is the class constructer, it defines what varibles will be stored by the class member varibles and what functions are avilable to that class 

	def__init__(self):
		print "Creaating gudinece control object ..."
		rospy.init_node('guidance_control') #ros node house keeping

		#subscriber and puplisher
		self.scan_sub_sub_ = rospy.Subscriber('base_scan',LaserScan,self.scan_callback) #subscriber to base scan topic

		self.odom_sub_= rospy..Subscriber('odom',Odometry,self.odom_callback) #subscrive odom to topic
		
		self.cmd_vel_pub_ = rospy.Publisher ('cmd_vel', twist, queen_size=10) #publisher to cmd_vel topic 

		#initilaize member varible
		self.scan_received =False
		self.odom_received =False
		#RObot control varibles
		self.distThreshold =2.0 #obstacle avoidance threshold
		self.scaleVel= 1.0   #magnitude of obstecales avoidence and goal seeking velocity

		#get goal x and goal y location from the launch file
		self.goalX= rospy.get_param('robot_control/goalX',0.0)
		self.goalY= rospy.get_param('robot_control/goalY',0.0)

		#call back function triggrd when ever base scan messge is received
	def scan_callback(self,msg):
	#find a lser scan messge properities min max scan angle scan agnle incresament 
		self.maxAngle=msg.angle_max
		self.minAngle=msg.angle_min
		self.angleIncrement=msg.angle_increment
		#find current laser angle max scan length distance array for all scans and number of laser scans
		self.currentLaserTheta=msg.minAngle
		self.maxScanLength=msg.range_max
		self.distanceArray=msg.ranges
		self.numScans =len(self.distanceArray)

		#acknoledgment that a scan has been received and attempt to a compute a new control command
		self.scan_received=  True
		self.compute_cmd_vel()


		#call back function turggered wheneever and odom message is received
		def odom_callback(self,msg):
		self.currentX =msg.pose.pose.posotion.x
		self.currentY=msg.pose.pose.posotion.y

		#find current oriention of robos based on odometry *quaternion codinites)
		xOr = globalOdom.pose.pose.orientation.x
		yOr = globalOdom.pose.pose.orientation.y
		zOr = globalOdom.pose.pose.orientation.z
		wOr = globalOdom.pose.pose.orientation.w

		# find orientation of robot (Euler coordinates)
		(roll, pitch, yaw) = euler_from_quaternion([xOr, yOr, zOr, wOr])


		# find currentAngle of robot (equivlent to yaw), now that you have yaw the robot pose is completly defined by(currentX, currentY,currentAngle)

		self.currentAngle=yaw

		#acknowledge that an doometry has been received and attempt to compute a new control command
		self.odom_received= True
		self.compute_cmd_vel()



		#member function to compute a neew controll command given the latest sensor scan odometry

	def compute_cmd_vel(self):
		#only compute a new control if both base scan and odom have been received

		if(self.scan_received and self.odom_received):
			command=Twist()


			#fill in the field values are unspecified untill they are actually assignged. the twist message holds linead and angular


			command.linear.x = 0.0
			command.linear.y = 0.0
			command.linear.z = 0.0
			command.angular.x = 0.0
			command.angular.y = 0.0
			command.angular.z = 0.0



			#for each laser scan 

			TurnLeft=False 
			TurnRight=False

			obsAvoidingBearing =0.0 #holding change to avoid obstecale
			obsAvoidVel= 0.0 #velocity


			for curScan in range (0,self.numScans):
				if self.distanceArray[curScan] < self.distThreshold:

					if self.currentLaserTheta>= -self.pi/2.0 and self.currentLaserTheta <=0:

						# obstecale detected on the right side

						if not trunLeft: #has not pplied turn left yet

							obsAvoidBearing= 1.0 #turn left
							print'Left Turn manoveur applied'
							turnLeft= True


					elif self.currentLaserTheta >=0 and slef.currentLaserTheta <= self.pi/2:
						if not turnRight: #has not apllied turn right yet
							obsAvoidBearing +=1.0
							print' Right tuen manouvre applied'
							turnRight=True
					if self.currentLaserTheta >= -self.pi/6.0 and self.currentLaserTheta <self.pi/6.0:

						if self.distanceArray[curScan]/self.distanceThreshold < 1.0 - obsavoidVel:
							obsAvoidvel= self.scaleVel*(1.0-(self.sitanceArray[curScan])/self.distThreshold)
							print ' slowing down ' 

				self.currentLaserTheta +=self.angleIncrement

			#based on the motion you want (found using goal location, current location and pbstecale info

			headingToGoal= math.atan2(self.goalY-self.currentY, self.goalX-self.currentX)
			bearing= headingToGoal - self.currentAngle


			distToGoal= math.sqrt(math.pow(self.goalY - self.currentY,2) + math.pow(self.goalX -self.currentX,2))

			if distTogoal <5.0 :
			#slow down

			vel=vel&distToGoal/5.0
			if distTGoal <1.0 :
			#stop if arrive with in 1 unit from goal

			vel=0.0
			print'Arrived to goal'

			#commnand
			command.linear.x=2.5*(vel -obsAvoidvel)
			command.angular.z= 1.0 *(bearing +obsAvoidBearing)
			self.cmd_vel_pub_publish(command)



			# reset floags so that we only receive 

			self.scan_received=False
			self.odom_received=False
		
			
		else:
			if not self.scan_received:
				print"no scan received"
			if not self.odom_received:
				print "No ddometry received yet"


if __name__ =='__main__'=
	robot_control= GuidenceControl()
	rospy.spin()





