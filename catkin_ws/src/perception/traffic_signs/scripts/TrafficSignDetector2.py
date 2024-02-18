#!/usr/bin/env python

#ros
import rospy
import tf2_ros
from tf2_geometry_msgs import *
from tf.transformations import *
import tf


#python
import numpy as np 
import operator
from collections import Counter
import time

import math
#ros messages
# from geometry_msgs.msg import Polygon, PolygonStamped, PointStamped, PoseWithCovarianceStamped

from msgs_traffic.msg import TrafficSign, TrafficSignArray
from geometry_msgs.msg import PoseStamped, Point, PolygonStamped
from msgs_perception.msg import Obstacle, ObstacleArray
from geometry_msgs.msg import PoseWithCovarianceStamped
from std_msgs.msg import Bool
from msgs_navigation.msg import Path
from msgs_navigation.msg import GlobalPlan
from visualization_msgs.msg import Marker
from visualization_msgs.msg import MarkerArray

from msgs_perception.msg import ObstacleArray
from msgs_perception.msg import Obstacle

from shapely.geometry import LineString
from shapely.geometry import Point as Pointsh

class TrafficSignDetector(object):

	def __init__(self):

		self.path = np.array([])
		self.path_last_stamp = rospy.Time(0)
		self.current_pose = None
		self.warning_dist = 26.0

		self.LANEFOLLOW=0
		self.STRAIGHT=1
		self.RIGHT=2
		self.LEFT=3
		self.CHANGELANELEFT=4
		self.CHANGELANERIGHT=5
		self.UNKNOWN=6

		self.timeout_traffic_light = 120.0 #seg
		self.range_light_dist2path = 5.0

		##########################################
		# STOP
		self.range_stop_sign2path = 4.0

		self.global_plan=None
		self.markerArray = MarkerArray()
		self.markerArrayObs = MarkerArray()
		self.obstacle_Array = ObstacleArray()

		##########################################
		# TF
		self.tf2_buffer_odom2map = tf2_ros.Buffer()
		self.tf2_listener_odom2map = tf2_ros.TransformListener(self.tf2_buffer_odom2map)
		self.stamp=rospy.Time().now()
		##########################################

		# SUBS
		#self.signs_object_sub = rospy.Subscriber('/carina/perception/fusion/bbox_2Dto3D', ObstacleArray, self.signs_object_cb, queue_size=1)
		self.signs_object_sub = rospy.Subscriber('/carina/perception/stereo/traffic_sign_odom', ObstacleArray, self.signs_object_cb, queue_size=1)
		self.shutdown_sub = rospy.Subscriber('/carina/vehicle/shutdown', Bool, self.shutdown_cb, queue_size=1)
		self.path_sub = rospy.Subscriber("/carina/navigation/path_segment", Path, self.path_cb, queue_size=1)
		self.path_sub = rospy.Subscriber("/carina/navigation/cnn_path", Path, self.cnn_path_cb, queue_size=1)
		self.current_pose_sub = rospy.Subscriber('/carina/localization/pose', PoseWithCovarianceStamped, self.pose_cb, queue_size=1)
		self.global_plan_raw_sub = rospy.Subscriber('/carina/navigation/global_plan_raw', GlobalPlan, self.global_plan_cb, queue_size=1)

		self.pub = rospy.Publisher('/carina/perception/lidar/obstacles_marker_array_tfl', MarkerArray, queue_size=1)
		self.pub_obj = rospy.Publisher('/carina/perception/lidar/obstacles_array_tfl_all', ObstacleArray, queue_size=1)
		self.poly_path1_pub = rospy.Publisher('/carina/navigation/path_intersec_tf',PolygonStamped,queue_size=1)
		self.traffic_signs_pub = rospy.Publisher('/carina/perception/traffic_rules/traffic_signs', TrafficSignArray, queue_size=1)
		self.lines_to_traffic_signs_pub = rospy.Publisher('/carina/perception/traffic_rules/lines_to_traffic_signs', MarkerArray, queue_size=1, latch=True)


	def global_plan_cb(self,msg):
		self.global_plan=msg
		self.len_global_plan=len(self.global_plan.points)
		pose_back=None
		t = rospy.Duration(0.35) 

		self.markerArray.markers=[]

		for index, (pose, option) in enumerate(zip(self.global_plan.points, self.global_plan.road_options)):

			if self.STRAIGHT==option or self.RIGHT==option or self.LEFT==option:
				g=1.0

				angle = np.arctan2(pose.y-pose_back.y, pose.x-pose_back.x) #* 180 / np.pi

				pose_x=pose.x + math.cos(angle)*1. - math.sin(angle)*1. #ofset 1m in x and y 
				pose_y=pose.y + math.sin(angle)*1. + math.cos(angle)*1. #ofset 1m in x and y

				pose_xy1=Point()
				pose_xy1.x=pose_x
				pose_xy1.y=pose_y

				px = pose.x + math.cos(angle)*45 - math.sin(angle)*1.#ofset 45m in x and 1m in y
				py = pose.y + math.sin(angle)*45 + math.cos(angle)*1.#ofset 45m in x and 1m in y

				pose_xy2=Point()
				pose_xy2.x=px
				pose_xy2.y=py

				marker = Marker()
				marker.header.frame_id = "map"
				marker.type = marker.LINE_STRIP
				marker.action = marker.ADD
				marker.ns = "multi_tfl"
				marker.id = index
				# marker scale
				marker.scale.x = 6.
				marker.scale.y = 6.
				marker.scale.z = 6.

				# marker color
				marker.color.a = 1.0
				marker.color.r = 255
				marker.color.g = 0
				marker.color.b = 255

				# marker.points.append(pose_back)
				marker.points.append(pose_xy1)
				marker.points.append(pose_xy2)

				# marker orientaiton
				marker.pose.orientation.x = 0.0
				marker.pose.orientation.y = 0.0
				marker.pose.orientation.z = 0.0
				marker.pose.orientation.w = 1.0

				t = rospy.Duration(0) 
				marker.lifetime = t
				self.markerArray.markers.append(marker)

				pose_x1=pose.x+math.cos(angle)*1.6
				pose_y1=pose.y+math.sin(angle)*1.6

				pose_x2=pose.x+math.cos(angle)*5.
				pose_y2=pose.y+math.sin(angle)*5.

				px1 = (pose.x)+math.cos(math.pi/1.4)*(pose_x1-pose.x)+ math.sin(math.pi/1.4)*(pose_y1-pose.y)#rotating line -alpha
				py1 = (pose.y)-math.sin(math.pi/1.4)*(pose_x1-pose.x)+ math.cos(math.pi/1.4)*(pose_y1-pose.y)

				px2 = (pose.x)+math.cos(math.pi/1.4)*(pose_x2-pose.x)+ math.sin(math.pi/1.4)*(pose_y2-pose.y)
				py2 = (pose.y)-math.sin(math.pi/1.4)*(pose_x2-pose.x)+ math.cos(math.pi/1.4)*(pose_y2-pose.y)

				pxy1=Point()
				pxy1.x=px1
				pxy1.y=py1

				pxy2=Point()
				pxy2.x=px2
				pxy2.y=py2

				marker2 = Marker()
				marker2.header.frame_id = "map"
				marker2.type = marker2.LINE_STRIP
				marker2.action = marker2.ADD
				marker2.ns = "one_tfl"
				marker2.id = index+1000

				# marker scale
				marker2.scale.x = 7.
				marker2.scale.y = 7.
				marker2.scale.z = 7.

				# marker color
				marker2.color.a = 1.0
				marker2.color.r = 255
				marker2.color.g = 0
				marker2.color.b = 255

				# marker.points.append(pose_back)
				marker2.points.append(pxy1)
				marker2.points.append(pxy2)

				# marker orientaiton
				marker2.pose.orientation.x = 0.0
				marker2.pose.orientation.y = 0.0
				marker2.pose.orientation.z = 0.0
				marker2.pose.orientation.w = 1.0

				t = rospy.Duration(0) 
				marker2.lifetime = t
				self.markerArray.markers.append(marker2)
			pose_back=pose
		self.lines_to_traffic_signs_pub.publish(self.markerArray)


	def signs_object_cb(self, msg):
		if self.global_plan==None:
			return

		global_plan=self.global_plan		
		stamp = self.stamp

		traffic_sign_array = TrafficSignArray()
		traffic_sign_array.header.stamp = self.stamp#rospy.Time().now()
		traffic_sign_array.header.frame_id = 'map'
		traffic_sign_array.signs = []

		self.markerArrayObs.markers=[]
		self.obstacle_Array.obstacle=[]

		if (self.current_pose is not None) and (len(self.path) > 0) and len(msg.obstacle)>0: 
			curr_pose = np.array([self.current_pose.pose.pose.position.x, self.current_pose.pose.pose.position.y])

			#path segment from the current pose of the vehicle and an warnign safe distance
			dist_pose2path = np.sqrt(np.power(self.path[:, 0:2] - curr_pose, 2).sum(axis=1))
			i_min_dist_pose2path = dist_pose2path.argmin()
			i_max_dist_pose2path = np.where(dist_pose2path[i_min_dist_pose2path:] <= self.warning_dist)[0].argmax() + i_min_dist_pose2path

			path_segment = self.path[i_min_dist_pose2path:i_max_dist_pose2path, 0:2]
			
			indexes = np.arange(0, len(msg.obstacle))
			
			signs_array_light = []
			signs_array_light = np.array([i for i, o  in zip(indexes, msg.obstacle) if ('light' in o.classes[o.class_id])])

			signs_array_speed_limit = []
			signs_array_speed_limit = np.array([i for i, o in zip(indexes, msg.obstacle) \
				if (('30' in o.classes[o.class_id]) or ('60' in o.classes[o.class_id]) or ('90' in o.classes[o.class_id]))])

			signs_array_stop = []
			signs_array_stop = np.array([i for i, o in zip(indexes, msg.obstacle) if ('stop' in o.classes[o.class_id])])
						
			if (0 < len(signs_array_light) <= 12):
				
				signs_pose = []

				stamp = self.stamp
				for i in signs_array_light:

					if msg.obstacle[i].pose.position.z > 2.0:
						q = self.current_pose.pose.pose.orientation
						# Get yaw
						ori = (q.x,
								q.y, 
								q.z, 
								q.w)
						_, _, theta = tf.transformations.euler_from_quaternion(ori)

						msg.obstacle[i].pose.position.x = msg.obstacle[i].pose.position.x - 3.50*np.cos(theta)   
						msg.obstacle[i].pose.position.y = msg.obstacle[i].pose.position.y - 3.50*np.sin(theta)  

					signs_pose.append([msg.obstacle[i].pose.position.x, msg.obstacle[i].pose.position.y])

				# signs_pose = np.array([[obst.pose.position.x, obst.pose.position.y] for obst in msg.obstacle])
				signs_pose = np.asarray(signs_pose)
				
				#distante between the signs and the path segment
				dist_sign2path = np.array([np.sqrt(np.power(o - path_segment[:, 0:2], 2).sum(axis=1)) for o in signs_pose])

				signs_array = []
				signs_array = np.array([i for i, d in zip(signs_array_light, dist_sign2path) if (d<self.range_light_dist2path).any()])
				
				ego_pose_sh= Pointsh(self.current_pose.pose.pose.position.x, self.current_pose.pose.pose.position.y)
				ego_pose_sh_dilated = ego_pose_sh.buffer(0.1)###dilated pedestrian

				index=0

				for m in self.markerArray.markers:
					index=index+1
					
					area_intersect=-1
					area_intersect_ego=-1

					color_tfl=''

					back_l_ob_ego = LineString([(m.points[0].x, m.points[0].y), (m.points[1].x, m.points[1].y)])
					back_l_ob_ego_dilated = back_l_ob_ego.buffer(m.scale.x/2)

					if(ego_pose_sh_dilated.intersects(back_l_ob_ego_dilated)):### ignore all traffic ligths when ego is running into the marker area
						continue

					poligon =None
					for obj in msg.obstacle:
						if len(msg.obstacle)> 4 and m.ns =="one_tfl":
							continue

						if obj.pose.position.z > 3.0 and m.ns =="one_tfl":
							continue

						l_ob = Pointsh(obj.pose.position.x, obj.pose.position.y)
						l_ob = l_ob.buffer(0.20)###dilated 
						if m.ns =="one_tfl":
							l_ob = l_ob.buffer(.40)###dilated
						poligon =None

						if(l_ob.intersects(back_l_ob_ego_dilated)):
							if l_ob.intersection(back_l_ob_ego_dilated).area > area_intersect:
								poligon=back_l_ob_ego_dilated
								area_intersect = l_ob.intersection(back_l_ob_ego_dilated).area
								color_tfl = obj.classes[0]
								xp=m.points[0].x
								yp=m.points[0].y

					if poligon is not None:
						pol_viz = PolygonStamped()
						pol_viz.header.frame_id = "map"
						for x, y in poligon.exterior.coords:
							p = PointStamped()
							p.point.x=x
							p.point.y=y
							p.point.z=1.0
							# print("x={}, y={}, z={}".format(p.point.x, p.point.y, p.point.z))
							pol_viz.polygon.points.append(p.point)
						self.poly_path1_pub.publish(pol_viz)

					# if obj.classes[0]=='traffic_light_red'  or  obj.classes[0]=='traffic_light_yellow':
					if color_tfl=='traffic_light_red'  or  color_tfl=='traffic_light_yellow':

						scale_x = 4.2
						scale_y = 4.2
						scale_z = 1.5

						marker2 = Marker()
						marker2.header.frame_id = "map"
						marker2.type = marker2.CUBE
						marker2.action = marker2.ADD
						marker2.ns = "my_namespace";
						marker2.id = index
		                    # marker scale
						marker2.scale.x = scale_x
						marker2.scale.y = scale_y
						marker2.scale.z = scale_z
		                    # marker color
						marker2.color.a = 1.0
						marker2.color.r = 1.0
						marker2.color.g = 0.0
						marker2.color.b = 0.0
		                    # marker orientaiton
						marker2.pose.orientation.x = 0.0
						marker2.pose.orientation.y = 0.0
						marker2.pose.orientation.z = 0.0
						marker2.pose.orientation.w = 1.0
		                    # marker position
						marker2.pose.position.x = xp
						marker2.pose.position.y = yp
						marker2.pose.position.z = 0.0
		                    
						t = rospy.Duration(0.50) 
						marker2.lifetime = t
						self.markerArrayObs.markers.append(marker2)

						obj = Obstacle()
						obj.header.frame_id = "map"
						obj.header.stamp = self.stamp#rospy.Time.now()
						obj.ns = "my_namespace";
						obj.classes=['blocked_point',color_tfl]
		                    # object orientaiton
						obj.pose.orientation.x = 0
						obj.pose.orientation.y = 0
						obj.pose.orientation.z = 0
						obj.pose.orientation.w = 1
		                    # object position
						obj.pose.position.x =  m.points[0].x
						obj.pose.position.y =  m.points[0].y
						obj.pose.position.z = -1.0

						obj.scale.x = scale_x
						obj.scale.y = scale_y
						obj.scale.z = scale_z

						obj.id = index

						obj.color.r = 255
						obj.color.g = 0
						obj.color.b = 0
						obj.color.a = 255

						obj.track_status = 1   ############## important 1

						obj.lifetime = t
						obj.type = -1
						obj.animation_speed = 0.5;

						self.obstacle_Array.obstacle.append(obj)


			if len(signs_array_stop) > 0:
				signs_pose = []
				# try:
				# trans = self.tf2_buffer_odom2map.lookup_transform('map', 'odom', rospy.Time())
				stamp = self.stamp
				for i in signs_array_stop:

					# old_pose = PoseStamped()
					# old_pose.header.frame_id='odom'
					# old_pose.header.stamp = self.stamp
					# old_pose.pose = msg.obstacle[i].pose

					# new_pose = tf2_geometry_msgs.do_transform_pose(old_pose, trans)
					# msg.obstacle[i].pose.position.x = new_pose.pose.position.x
					# msg.obstacle[i].pose.position.y = new_pose.pose.position.y

					signs_pose.append([msg.obstacle[i].pose.position.x, msg.obstacle[i].pose.position.y])
					# signs_pose.append([new_pose.pose.position.x, new_pose.pose.position.y])
					
				# except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
				# 	print ('[Traffic Sign Detection] exception waiting for tf from frames odom to map')
				# 	return
				signs_pose = np.asarray(signs_pose)
				dist_pose2sign = np.sqrt(np.power(curr_pose - signs_pose[:, 0:2], 2).sum(axis=1))

				#stop sign within a threshold?
				if (dist_pose2sign <= self.warning_dist).any():
					#dist to path
					dist_signs2path = np.array([np.sqrt(np.power(o - path_segment[:, 0:2], 2).sum(axis=1)) for o in signs_pose])

					i_min = -1
					dist_min = np.inf

					for i, d in zip(signs_array_stop, dist_signs2path):
						if ((d.min() < dist_min) and (d.min() < self.range_stop_sign2path)):
							dist_min = d.min()
							i_min = i 

					if i_min >= 0:
						obj = msg.obstacle[i_min]

						s = TrafficSign()
						s.name = obj.classes[obj.class_id]
						s.pose.pose = obj.pose
						s.type = s.list.UNKNOWN

						if s.name == 'stop':
							s.type = s.list.STOP
							s.value = 0.0

						if s.type != s.list.UNKNOWN:
							index = np.where(signs_array_stop==i_min)[0]
							print ("[Stop Sign Detector] detected stop sign at ({}/{})".format(dist_pose2sign[index], str(dist_min)) )
						traffic_sign_array.signs.append(s)

		self.traffic_signs_pub.publish(traffic_sign_array)
		self.pub.publish(self.markerArrayObs)
		self.pub_obj.publish(self.obstacle_Array)
		self.markerArrayObs.markers=[]
		self.obstacle_Array.obstacle=[]


	def pose_cb(self, msg):
		self.current_pose = msg
		self.stamp=msg.header.stamp

	def path_cb(self, msg):
		if msg.header.stamp > self.path_last_stamp:
			self.path_last_stamp = msg.header.stamp
			self.path =  np.array([p.point for p in msg.path])

	def cnn_path_cb(self, msg):
		if msg.header.stamp > self.path_last_stamp:
			self.path_last_stamp = msg.header.stamp
			self.path =  np.array([p.point for p in msg.path])

	def shutdown_cb(self, msg):
		if msg.data:
			del self.signs_object_sub
			del self.path_sub
			del self.shutdown_sub
			del self.traffic_signs_pub

			print ("Bye!")

			rospy.signal_shutdown("finished route")

