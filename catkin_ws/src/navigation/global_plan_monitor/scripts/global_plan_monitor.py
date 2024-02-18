#!/usr/bin/env python

#ros
import rospy

#python
import numpy as np

#ros messages
from msgs_navigation.msg import GlobalPlan
from geometry_msgs.msg import PoseWithCovarianceStamped, Point
from msgs_navigation.msg import SpeedConstraint
from std_msgs.msg import Bool

class GlobalPlanMonitor(object):


	def __init__(self):

		self.current_pose = None
		self.dist_ahead=30
		self.ddist=1
		self.npoints = 10
		self.stamp_global_plan = rospy.Time().now()
		self.global_plan_points = np.array([])
		self.global_plan_road_options = np.array([])
		self.road_options = {'lane_follow': 0, 'straight':1, 'right':2, 'left':3, 'changelane_left': 4, 'changelane_right':5}

		self.plan_sub = rospy.Subscriber('/carina/navigation/global_plan_raw', GlobalPlan, self.global_plan_cb, queue_size=1)
		self.pose_sub = rospy.Subscriber('/carina/localization/pose',PoseWithCovarianceStamped, self.pose_cb, queue_size=1)
		self.shutdown_sub = rospy.Subscriber("/carina/vehicle/shutdown", Bool, self.shutdown_cb, queue_size=1)
		self.speed_constraint_pub = rospy.Publisher('/carina/control/speed_constraint', SpeedConstraint, queue_size=1)
		self.near_to_intersection_pub = rospy.Publisher('/carina/control/near_to_intersection', Bool, queue_size=1)

		rospy.Timer(rospy.Duration(0.1), self.options_ahead)


	def global_plan_cb(self, msg):

		if msg.header.stamp > self.stamp_global_plan:
			print ("[Global Plan] plan received!")
			self.stamp_global_plan  = msg.header.stamp

			print('msg.points')

			# print(self.global_plan_points)
			print(np.array(msg.road_options))

			self.global_plan_points = np.array([[p.x, p.y, p.z] for p in msg.points])

			# if len(self.global_plan_points) == 0:
				# self.global_plan_points = np.array([[p.x, p.y, p.z] for p in msg.points])
			# else:
			# 	points = np.array([[p.x, p.y, p.z] for p in msg.points])
			# 	self.global_plan_points = np.concatenate((self.global_plan_points, points), axis=1)

			self.ddist = np.sqrt(np.power(self.global_plan_points[0,0:2] - self.global_plan_points[1, 0:2],2).sum())
			self.npoints = int(self.dist_ahead/self.ddist)

			print('self.npoints ',self.npoints)

			# print(self.global_plan_road_options)
			# print(np.array(msg.road_options))

			self.global_plan_road_options = np.array(msg.road_options)

			# if len(self.global_plan_road_options) == 0:
			# 	self.global_plan_road_options = np.array(msg.road_options)
			# else:
			# 	self.global_plan_road_options = np.concatenate((self.global_plan_road_options, np.array(msg.road_options)), axis=0)
			# 	print('cat',self.global_plan_road_options) 
			print ('road options', self.global_plan_road_options, self.global_plan_points)
			print (len(self.global_plan_road_options),len(self.global_plan_points))

			

	def pose_cb(self, msg):

		self.current_pose = msg
		self.stamp_global_plan = msg.header.stamp#rospy.Time().now()

	def shutdown_cb(self, msg):
		if msg.data:
			self.current_pose = None
			self.dist_ahead= None
			self.ddist= None
			self.npoints = None
			self.stamp_global_plan = None
			self.global_plan_points = None
			self.global_plan_road_options = None
			self.road_options = None

			del self.plan_sub 
			del self.pose_sub 
			del self.shutdown_sub 
			del self.speed_constraint_pub 
			del self.near_to_intersection_pub 



			print ("Bye!")
			rospy.signal_shutdown("finished route")


	def options_ahead(self, event):
		# print(self.global_plan_points)

		if self.current_pose is None:
			return

		if self.global_plan_points is None or len(self.global_plan_points)==0:
			return


		pose = np.array([self.current_pose.pose.pose.position.x, self.current_pose.pose.pose.position.y])

		dist = np.sqrt(np.power(pose - self.global_plan_points[:, 0:2],2).sum(axis=1))
		i_min = dist.argmin()

		path_segment = self.global_plan_points[i_min:i_min + self.npoints, :]
		
		road_options_segment = self.global_plan_road_options[i_min:i_min+self.npoints]
		print(road_options_segment)

		if (self.road_options['left'] in road_options_segment) or \
			(self.road_options['right'] in road_options_segment): #or \
			# (self.road_options['straight'] in road_options_segment):

			sp = SpeedConstraint()
			sp.header.stamp = self.stamp_global_plan#rospy.Time().now()
			# sp.speed=8.8
			#sp.speed=3.0
			# sp.speed=2.5
			# sp.speed=1.8
			sp.speed=1.0

			sp.reason="near to intersection"
			self.speed_constraint_pub.publish(sp)


			# self.near_to_intersection_pub.publish(True)
			if not(self.road_options['straight'] in road_options_segment):
				self.near_to_intersection_pub.publish(True)
				print ("near to intersection")

		elif(self.road_options['changelane_left'] in road_options_segment) or \
		   (self.road_options['changelane_right'] in road_options_segment) :

			sp = SpeedConstraint()
			sp.header.stamp = self.stamp_global_plan#rospy.Time().now()
			sp.speed=0.5
			# sp.speed=3.5
			#sp.speed=3.0
			# sp.speed=2.5
			# sp.speed=1.8
			# sp.speed=1.0

			sp.reason="near to changelane"
			# self.speed_constraint_pub.publish(sp)


			# # self.near_to_intersection_pub.publish(True)
			# if not(self.road_options['straight'] in road_options_segment):
			# 	self.near_to_intersection_pub.publish(True)
			# 	print ("near to changelane")

		else:
			# print ("road_options_segment[len(road_options_segment)/2:]")
			# print (road_options_segment[len(road_options_segment)/2:])
			self.near_to_intersection_pub.publish(False)



		




