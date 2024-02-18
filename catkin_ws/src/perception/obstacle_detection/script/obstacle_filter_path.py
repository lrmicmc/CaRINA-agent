#!/usr/bin/env python
import rospy
import numpy as np
import tf
import tf2_ros
from tf2_geometry_msgs import *
from tf.transformations import *

from msgs_navigation.msg import Path as NavPath
from nav_msgs.msg import Path as RosPath
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped, PointStamped
from std_msgs.msg import Bool
from msgs_perception.msg import ObstacleArray
from msgs_perception.msg import Obstacle


from visualization_msgs.msg import *


class ObstacleFilterPath(object):

	def __init__ (self):
		self.path = np.array([])
		self.path_ros = RosPath()
		self.curr_pose = None
		self.obstacles_array = []

		self.safe_dist = 45.0
		self.dangerous_dist = 8.5
		self.min_area_obj = 0.1
		self.range_dist_to_path = 2.0#1.3#1.2

		self.last_pose_track = None

		self.last_path_stamp=rospy.Time().now()
		self.pathReceived = False

		self.tf2_buffer_vel2map = tf2_ros.Buffer()
		self.tf2_listener_vel2map = tf2_ros.TransformListener(self.tf2_buffer_vel2map)

		self.pub = rospy.Publisher('/carina/transformed/lidar_map/obstacles_marker_array', MarkerArray, queue_size=1)


		self.path_sub = rospy.Subscriber('/carina/navigation/path_segment', NavPath, self.path_callback, queue_size=1)
		self.path_sub_cnn = rospy.Subscriber('/carina/navigation/cnn_path', NavPath, self.cnn_path_callback, queue_size=1)

		self.pose_sub = rospy.Subscriber('/carina/localization/pose', PoseWithCovarianceStamped, self.pose_callback, queue_size=1)
		self.obst_sub = rospy.Subscriber('/carina/perception/lidar/obstacles_array', ObstacleArray, self.obst_callback, queue_size=1)
		self.shutdown_sub = rospy.Subscriber('/carina/vehicle/shutdown', Bool, self.shutdown_cb, queue_size=1)

		self.nearest_obst_ahead_pub = rospy.Publisher('/carina/perception/lidar/nearest_obstacle_ahead', ObstacleArray, queue_size=1)

		

	def path_callback(self, msg):
		
		if msg.header.stamp>self.last_path_stamp:
			self.last_path_stamp = msg.header.stamp
			path_msg = msg.path

			arr =  np.array([p.point for p in path_msg])
			self.path = arr[:, 0:2] 		

			pathReceived = True


	def cnn_path_callback(self, msg):
		
		if msg.header.stamp>self.last_path_stamp:
			self.last_path_stamp = msg.header.stamp
			path_msg = msg.path

			arr =  np.array([p.point for p in path_msg])
			self.path = arr[:, 0:2] 		

			pathReceived = True

	def pose_callback(self, msg):
		self.curr_pose  = msg

			
		o_array = ObstacleArray()
		o_array.obstacle = []
		
		if len(self.path)>0 and len(self.obstacles_array)>0:
			if self.nearest_obst_ahead_pub.get_num_connections() > 0:

				pose = np.array([self.curr_pose.pose.pose.position.x, self.curr_pose.pose.pose.position.y])

				q = msg.pose.pose.orientation

				# Get yaw
				ori = (q.x,
					q.y, 
					q.z, 
					q.w)
				_, _, theta = tf.transformations.euler_from_quaternion(ori)

				dist, obst, _id = self.min_dist(pose, theta)
				# print(dist)


				if dist > 0:

					
					if self.last_pose_track is None:
						self.last_pose_track = np.array([obst.pose.position.x, obst.pose.position.y])

					dist_to_last_track = np.sqrt(np.power(self.last_pose_track - [obst.pose.position.x, obst.pose.position.y],2).sum())
					
				
					obst.color.r = 0
					obst.color.g = 0
					obst.color.b = 255
					obst.color.a = 255

					obst.scale.x = obst.scale.x + 0.3
					obst.scale.y = obst.scale.y + 0.3
					obst.scale.z = obst.scale.z + 0.3

					obst.header.stamp = msg.header.stamp#rospy.Time().now()
					o_array.obstacle.append(obst)
		self.nearest_obst_ahead_pub.publish(o_array)

		
		

	def obst_callback(self, msg):
		self.obstacles_array = msg.obstacle


	def min_dist(self, pose, theta):
		obst = self.obstacles_array
		obst_pose = []
		obst_area = []

		stamp = self.curr_pose.header.stamp 


		try:
			trans = self.tf2_buffer_vel2map.lookup_transform('map', 'velodyne', rospy.Time())
		except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
			print ('[Obstacle Detection] exception waiting for tf from frames velodyne to map')
			return -1, 0, 0


		markerArray = MarkerArray()
		markerArray.markers=[]

		for o in obst:

			old_pose = PoseStamped()
			old_pose.header.frame_id='velodyne'
			old_pose.header.stamp = stamp
			old_pose.pose = o.pose

			new_pose = tf2_geometry_msgs.do_transform_pose(old_pose, trans)

			obst_pose.append([new_pose.pose.position.x, new_pose.pose.position.y])

			obst_area.append(o.scale.x*o.scale.y)

			


		dist_pose2path = np.sqrt(np.power(self.path[:, 0:2] - pose, 2).sum(axis=1))
		i_min_dist2path = dist_pose2path.argmin()
		i_max_dist2path = np.where(dist_pose2path[i_min_dist2path:]<=self.safe_dist)[0].argmax() + i_min_dist2path

		path_segment = self.path[i_min_dist2path:i_max_dist2path, 0:2]


		obst_pose = np.asarray(obst_pose)
		dist_pose2obst = np.sqrt(np.power(pose - obst_pose, 2).sum(axis=1))		

		obst_area = np.asarray(obst_area)
		n_i_obst = np.where(np.logical_and(dist_pose2obst <= self.dangerous_dist, obst_area>=self.min_area_obj))[0]


		i_obst = np.where(dist_pose2obst <= self.safe_dist)[0]		
		dist_obst2path = np.array([np.sqrt(np.power(o - path_segment[:,0:2],2).sum(axis=1)) for o in obst_pose[i_obst]])
		obsts = []
		min_dist = np.inf
		i_min_dist = -1
		
		for i, dobst in zip(i_obst, dist_obst2path):
			o = obst[i]
			if (dobst <= self.range_dist_to_path).any():#and ((o.scale.x*o.scale.y) >= self.common_area_obst):
				obsts.append(i)
				if dist_pose2obst[i] < min_dist:
					min_dist = dist_pose2obst[i]
					i_min_dist = i

		obsts = np.asarray(obsts)
	
		if i_min_dist>=0:
			ob = obst[i_min_dist]
			ob.header.stamp = stamp#rospy.Time().now()
			ob.header.frame_id = 'map'
			ob.pose.position.x = obst_pose[i_min_dist, 0]
			ob.pose.position.y = obst_pose[i_min_dist, 1]
			ob.pose.orientation = obst[i_min_dist].pose.orientation
			return  min_dist, ob, i_min_dist
		else:
			return -1, None, -1

	def shutdown_cb(self, msg):
		if msg.data:
			self.path = None
			self.path_ros = None
			self.curr_pose = None
			self.obstacles_array = None
			self.safe_dist = None
			self.dangerous_dist = None
			self.min_area_obj = None
			self.range_dist_to_path = None

			self.last_pose_track = None
			self.last_path_stamp=None
			self.pathReceived = None
			self.tf2_buffer_vel2map = None
			self.tf2_listener_vel2map = None

			del self.path_sub 
			del self.path_sub_cnn 
			del self.pose_sub 
			del self.obst_sub
			del self.shutdown_sub 
			del self.nearest_obst_ahead_pub 
			# del self.emergency_stop_pub 
			# del self.pub 

			print ("Bye!")
			rospy.signal_shutdown("finished route")

