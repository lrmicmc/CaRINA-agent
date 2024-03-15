#!/usr/bin/env python3
import rospy

from sensor_msgs.msg import PointCloud2
# import sensor_msgs.point_cloud2 as pc2
from std_msgs.msg import Bool

import ros_numpy

from msgs_navigation.msg import Path as NavPath
from shapely.geometry import LineString,Point
from geometry_msgs.msg import Polygon, PolygonStamped, PointStamped, PoseWithCovarianceStamped

import numpy as np

import tf2_ros
from tf2_geometry_msgs import *
from tf.transformations import *
from tf2_sensor_msgs.tf2_sensor_msgs import do_transform_cloud

from matplotlib import pyplot as plt
from msgs_perception.msg import ObstacleArray
from msgs_perception.msg import Obstacle
from msgs_traffic.msg import TrafficSign, TrafficSignArray
from msgs_navigation.msg import GlobalPlan
from visualization_msgs.msg import Marker
from visualization_msgs.msg import MarkerArray

from msgs_navigation.msg import SpeedConstraint
import os
import math 

class CollisionDetection(object):

	def __init__ (self):

		self.last_path_stamp = rospy.Time().now()
		self.path_shapely_dilated1 = None
		self.path_shapely_dilated2 = None
		self.back_l_ob_ego_dilated=None
		self.curr_pose = None
		self.ocup_map = []
		self.ocup_map_f = []

		self.obst_det3d_arr=None

		self.stops=[]

		self.tf2_buffer_blink2odom = tf2_ros.Buffer()
		self.tf2_listener_blink2odom = tf2_ros.TransformListener(self.tf2_buffer_blink2odom)
		self.obstacle_tfl=[]
		self.obstacles_dataset_gt=[]
		self.obstacles_3d=[]
		self.obstacles_mot=[]
		self.obstacles_mot_stereo=[]


		self.LANEFOLLOW=0
		self.STRAIGHT=1
		self.RIGHT=2
		self.LEFT=3
		self.CHANGELANELEFT=4
		self.CHANGELANERIGHT=5
		self.UNKNOWN=6

		self.markerArray = MarkerArray()
		self.markerArray.markers=[]
		self.global_plan_received=False
		self.path = np.array([])

		self.global_plan_processed=None
		
		self.track_env = os.environ.get('CHALLENGE_TRACK_CODENAME')


		self.poly_path1_pub = rospy.Publisher('/carina/navigation/path_dilated1',PolygonStamped,queue_size=1)
		self.poly_path2_pub = rospy.Publisher('/carina/navigation/path_dilated2',PolygonStamped,queue_size=1)
		self.poly_path3_pub = rospy.Publisher('/carina/navigation/path_dilated3',PolygonStamped,queue_size=1)
		self.ego_dilated_area = rospy.Publisher('/carina/navigation/ego_dilated_area',PolygonStamped,queue_size=1)
		self.tfl_red_area     = rospy.Publisher('/carina/navigation/tfl_red_trigger_area',PolygonStamped,queue_size=1)
		self.risk_of_collision = rospy.Publisher('/carina/sensor/risk_of_collision', Bool, queue_size=1)
		self.obstacles_array_tfl_pub = rospy.Publisher('/carina/perception/lidar/obstacles_array_tfl', ObstacleArray, queue_size=1)
		self.marker_tf_red_array_pub = rospy.Publisher('/carina/perception/lidar/tfl_marker_array', MarkerArray, queue_size=1)
		self.marker_intersection_warning_pub = rospy.Publisher('/carina/route/points/route_points_warning', MarkerArray, queue_size=1)
		self.lines_ori_obst_pub = rospy.Publisher('/carina/objects/lines_orientation_obstacles', MarkerArray, queue_size=1)
		self.pub = rospy.Publisher('/carina/perception/lidar/obstacles_marker_array', MarkerArray, queue_size=1)
		self.pub_obj = rospy.Publisher('/carina/perception/lidar/obstacles_array', ObstacleArray, queue_size=1)
		self.speed_constraint_pub = rospy.Publisher('/carina/control/speed_constraint', SpeedConstraint, queue_size=1)

		# self.odom_objs_tfsign_pub = rospy.Subscriber("/carina/perception/stereo/traffic_sign_odom", ObstacleArray, self.stop_callback,queue_size = 1)
		self.traffic_signs_sub= rospy.Subscriber('/carina/perception/traffic_rules/traffic_signs', TrafficSignArray, self.stop_callback, queue_size=1)
		self.pose_sub = rospy.Subscriber('/carina/localization/pose', PoseWithCovarianceStamped, self.pose_callback, queue_size=1)
		self.cnn_path_sub = rospy.Subscriber('/carina/navigation/cnn_path', NavPath, self.cnn_path_callback, queue_size=1)
		self.path_sub     = rospy.Subscriber('/carina/navigation/path_segment', NavPath, self.path_callback, queue_size=1)
		self.lidar_sub = rospy.Subscriber("/carina/perception/lidar/velodyne_obstacles", PointCloud2, self.ocupation_point_cloud)
		# self.all_obstacles_sub =   rospy.Subscriber('/carina/perception/lidar/obstacles_array'        ,ObstacleArray, self.all_obstacles_coll_avoidance_callback,queue_size=1)
		self.gt_obstacles_sub = rospy.Subscriber('/carina/perception/dataset/obstacles_array', ObstacleArray, self.obstacles_dataset_gt_cb, queue_size=1)
		self.obstacles_3d_sub = rospy.Subscriber('/carina/sensor/lidar/obst_3d_array', ObstacleArray, self.obstacles_3d_cb, queue_size=1)
		self.obstacles_mot_sub = rospy.Subscriber('/carina/perception/mot/lidar/obstacles', ObstacleArray, self.obstacles_mot_cb, queue_size=1)
		self.obstacles_mot_stereo_sub = rospy.Subscriber('/carina/perception/mot/stereo/obstacles', ObstacleArray, self.obstacles_mot_stereo_cb, queue_size=1)
		self.shutdown_sub = rospy.Subscriber('/carina/vehicle/shutdown', Bool, self.shutdown_cb, queue_size=1)
		self.global_plan_raw_sub = rospy.Subscriber('/carina/navigation/global_plan_raw', GlobalPlan, self.global_plan_cb, queue_size=1)
		self.obstacles_array_tfl_all_sub = rospy.Subscriber('/carina/perception/lidar/obstacles_array_tfl_all', ObstacleArray, self.obstacle_tfl_cb, queue_size=1)


	def normalize_angle(self, angle):
		return math.atan2(np.sin(angle), np.cos(angle))


	def obstacle_tfl_cb(self,msg):
		self.obstacle_tfl=msg.obstacle
		# print('len ',len(self.global_plan.points))


	def obstacles_dataset_gt_cb(self,msg):
		self.obstacles_dataset_gt=msg.obstacle
		# print('len ',len(self.global_plan.points))
	def obstacles_3d_cb(self,msg):
		self.obstacles_3d=msg.obstacle
		# print('len ',len(self.obstacles_3d))
	def obstacles_mot_cb(self,msg):
		self.obstacles_mot=msg.obstacle
		# print('len ',len(self.obstacles_3d))
	def obstacles_mot_stereo_cb(self,msg):
		obstacles_mot_stereo_temp=msg.obstacle

		# print('obstacles_mot_stereo_temp', obstacles_mot_stereo_temp)
		if len(msg.obstacle)>0:
			for obs in obstacles_mot_stereo_temp:
				obs.pose.orientation.x = 0.0
				obs.pose.orientation.y = 0.0
				obs.pose.orientation.z = 0.0
				obs.pose.orientation.w = 1.0

		self.obstacles_mot_stereo=obstacles_mot_stereo_temp
		# print('self.obstacles_mot_stereo', self.obstacles_mot_stereo)



	def cnn_path_callback(self, msg):
		if msg.header.stamp>self.last_path_stamp:
			# print("[CollisionDetection Node] path received!!!")

			self.last_path_stamp = msg.header.stamp
			path_msg = msg.path
			# print(path_msg)
			arr = np.array([p.point for p in path_msg])
			# print(arr)
			self.path = arr[:, 0:2]

	def stop_callback(self, msg):
		signs=msg.signs
		stops=[]
		for s in signs:
			# print(s.name)
			# print(s.pose)
			stops.append(s)
		self.stops=stops


	def global_plan_cb(self,msg):
		if msg.points==[]:
			return
		if self.global_plan_received == True:
			return

		self.global_plan_received=True

		self.global_plan=msg
		# print('len ',len(self.global_plan.points))
		self.len_global_plan=len(self.global_plan.points)



		
		self.global_plan_processed = GlobalPlan()
		self.global_plan_processed.header.stamp = msg.header.stamp#rospy.Time().now()
		self.global_plan_processed.header.frame_id = 'map'
		self.global_plan_processed.points = []
		self.global_plan_processed.road_options = []

		pose_plan_old=None

		for index, (pose_plan, option) in enumerate(zip(self.global_plan.points, self.global_plan.road_options)):



			# if self.LANEFOLLOW==option:

			x = pose_plan.x
			y = pose_plan.y

			pose_plan_arr=np.array([x,y])
			# dist_to_point_instruction = np.linalg.norm(obj_pose-pose_actual)
			# if dist_to_plan_point>40:
			# 	continue
			# l_ob = Point(x, y)
			# l_ob = l_ob.buffer(0.5)###dilated obstacle

			# if( l_ob.intersects(self.path_shapely_dilated1)):# or l_ob.intersects(self.path_shapely_dilated2) ):
			if (self.STRAIGHT==option or self.RIGHT==option or self.LEFT==option or self.CHANGELANELEFT==option or self.CHANGELANERIGHT==option):

				if pose_plan_old is None:
					self.global_plan_processed.points.append(pose_plan)
					self.global_plan_processed.road_options.append(option)	
					pose_plan_old=pose_plan_arr
				
				else: 
					dist_to_plan_point = np.linalg.norm(pose_plan_old-pose_plan_arr)

					if dist_to_plan_point > 26:
						self.global_plan_processed.points.append(pose_plan)
						self.global_plan_processed.road_options.append(option)
						pose_plan_old=pose_plan_arr	

				# if index+20 < len(self.global_plan.points):
				# 	print(self.global_plan.points[index])


	def path_callback(self, msg):
		#Path callback
		if msg.header.stamp>self.last_path_stamp:
			# print ("[CollisionDetection Node] path received!!!")
			self.last_path_stamp = msg.header.stamp
			path_msg = msg.path

			arr =  np.array([p.point for p in path_msg])
			self.path = arr[:, 0:2] 		

	def pose_callback(self,msg):
		self.obstacles= self.obstacles_dataset_gt + self.obstacles_3d + self.obstacles_mot + self.obstacles_mot_stereo

		collision=False



		markerArray_follow = MarkerArray()
		obstacle_Array = ObstacleArray()
		markerArray_follow.markers=[]
		obstacle_Array.obstacle=[]


		dist=1000
		xo=100
		yo=100
		
		markerArray_tfl= MarkerArray()
		obstacle_Array_tfl = ObstacleArray()
		markerArray_tfl.markers=[]
		obstacle_Array_tfl.obstacle=[]
		# lines_obs = []

		marker_array_lines = MarkerArray()
		marker_array_lines.markers = []

		obst_det3d=[]


		self.markerArray.markers=[]


		self.curr_pose = msg

		if len(self.path)==0 or self.curr_pose == None:


			self.marker_tf_red_array_pub.publish(markerArray_tfl)
			self.obstacles_array_tfl_pub.publish(obstacle_Array_tfl)

			self.marker_intersection_warning_pub.publish(self.markerArray)
			self.pub.publish(markerArray_follow)
			self.pub_obj.publish(obstacle_Array)
			self.lines_ori_obst_pub.publish(marker_array_lines)
			# self.risk_of_collision.publish(collision)
			self.risk_of_collision.publish(True)

			return



		pose =  np.array([self.curr_pose.pose.pose.position.x, self.curr_pose.pose.pose.position.y])
		# print(pose)


		ori = self.curr_pose.pose.pose.orientation
		# pose =  np.array([self.curr_pose.pose.pose.position.x, self.curr_pose.pose.pose.position.y])

		dist_pose2path = np.sqrt(np.power(self.path[:,0:2]-pose,2).sum(axis=1))
		i_min_dist2path = dist_pose2path.argmin()

		i_monitor_path=i_min_dist2path


		if(i_monitor_path+35>=len(self.path)):
			path_ahead_1=self.path[  i_monitor_path:min(i_monitor_path,len(self.path))     , 0:2]
			path_ahead_2=self.path[len(self.path)-1:len(self.path), 0:2]
		else:
			path_ahead_1=self.path[i_monitor_path+14:i_monitor_path+35, 0:2]
			path_ahead_2=self.path[i_monitor_path+35:len(self.path), 0:2]

		# print ('len path', len(path_ahead_1))
		# print ('len path2', len(path_ahead_2))

		if len(path_ahead_1)<7 :  #or  len(path_ahead_2)<1  or  i_monitor_path>len(self.path)-5:
				collision_constraint = SpeedConstraint()
				collision_constraint.header.stamp = rospy.Time().now()
				collision_constraint.speed = 0.20
				collision_constraint.reason = "\033[31m[From collision_node] Hight risk of collision path ended: \033[0m" 
				print (collision_constraint.reason)
				self.speed_constraint_pub.publish(collision_constraint)
				# return

		# if len(path_ahead_1)<3:  #or  len(path_ahead_2)<1  or  i_monitor_path>len(self.path)-5:
		# 	print ('len path', len(path_ahead_1))
		# 	print ('len path2', len(path_ahead_2))
		# 	self.marker_tf_red_array_pub.publish(markerArray_tfl)
		# 	self.obstacles_array_tfl_pub.publish(obstacle_Array_tfl)

		# 	self.marker_intersection_warning_pub.publish(self.markerArray)
		# 	self.pub.publish(markerArray_follow)
		# 	self.pub_obj.publish(obstacle_Array)
		# 	self.lines_ori_obst_pub.publish(marker_array_lines)
		# 	# self.risk_of_collision.publish(collision)
		# 	self.risk_of_collision.publish(True)

		# 	return

		# self.risk_of_collision.publish(collision)

		# elif len(path_ahead_2)<2:
		# 	return

		path_shapely_1 = LineString(path_ahead_1)
		path_shapely_2 = LineString(path_ahead_2)

		self.path_shapely_dilated1 = path_shapely_1.buffer(1.0)
		self.path_shapely_dilated2 = path_shapely_2.buffer(1.0)
		# self.path_shapely_dilated3 = path_shapely_1.buffer(1.7)




		angle_ori_hero = euler_from_quaternion([ori.x, ori.y, ori.z, ori.w])[2]



			# elif ob.classes[0]== "walker":
			# 	orig_line_obx  = ob.pose.position.x
			# 	orig_line_oby =  ob.pose.position.y
			# 	fin_line_obx  = ob.pose.position.x### can be comment (ingnored) -->only for viz
			# 	fin_line_oby =  ob.pose.position.y### can be comment (ingnored) -->only for viz
			# 	l_ob = Point(orig_line_obx, orig_line_oby)
			# 	l_ob = l_ob.buffer(10.1)###dilated pedestrian

						# back line_pose ego car
		center_line_egox  = self.curr_pose.pose.pose.position.x
		center_line_egoy  = self.curr_pose.pose.pose.position.y

		orig_line_egox  = center_line_egox+math.cos(angle_ori_hero)*2.25
		orig_line_egoy  = center_line_egoy+math.sin(angle_ori_hero)*2.25

		fin_line_egox = center_line_egox-math.cos(angle_ori_hero)*2.25
		fin_line_egoy = center_line_egoy-math.sin(angle_ori_hero)*2.25

		back_l_ob_ego = LineString([(orig_line_egox, orig_line_egoy), (fin_line_egox, fin_line_egoy)])

		self.back_l_ob_ego_dilated = back_l_ob_ego.buffer(1.21)



		#########publish marker###############
		if self.ego_dilated_area.get_num_connections() > 0:
			pol_viz = PolygonStamped()
			pol_viz.header.frame_id = msg.header.frame_id
			for x, y in self.back_l_ob_ego_dilated.exterior.coords:
				p = PointStamped()
				p.point.x=x
				p.point.y=y
				p.point.z=self.curr_pose.pose.pose.position.z
				# print("x={}, y={}, z={}".format(p.point.x, p.point.y, p.point.z))
				pol_viz.polygon.points.append(p.point)
			self.ego_dilated_area.publish(pol_viz)




		if self.poly_path1_pub.get_num_connections() > 0:
			pol_viz = PolygonStamped()
			pol_viz.header.frame_id = msg.header.frame_id
			for x, y in self.path_shapely_dilated1.exterior.coords:
				p = PointStamped()
				p.point.x=x
				p.point.y=y
				p.point.z=self.curr_pose.pose.pose.position.z
				# print("x={}, y={}, z={}".format(p.point.x, p.point.y, p.point.z))
				pol_viz.polygon.points.append(p.point)
			self.poly_path1_pub.publish(pol_viz)

		# if self.poly_path3_pub.get_num_connections() > 0:
		# 	pol_viz = PolygonStamped()
		# 	pol_viz.header.frame_id = msg.header.frame_id
		# 	for x, y in self.path_shapely_dilated3.exterior.coords:
		# 		p = PointStamped()
		# 		p.point.x=x
		# 		p.point.y=y
		# 		p.point.z=1.0
		# 		# print("x={}, y={}, z={}".format(p.point.x, p.point.y, p.point.z))
		# 		pol_viz.polygon.points.append(p.point)
		# 	self.poly_path3_pub.publish(pol_viz)

		if self.poly_path2_pub.get_num_connections() > 0:
			pol_viz = PolygonStamped()
			pol_viz.header.frame_id = msg.header.frame_id
			for x, y in self.path_shapely_dilated2.exterior.coords:
				p = PointStamped()
				p.point.x=x
				p.point.y=y
				p.point.z=self.curr_pose.pose.pose.position.z
				# print("x={}, y={}, z={}".format(p.point.x, p.point.y, p.point.z))
				pol_viz.polygon.points.append(p.point)
			self.poly_path2_pub.publish(pol_viz)


		# print(path_ahead_1)
		# print(path_ahead_2)
		ocup_map_raw=self.ocup_map.copy()
		# print('ocup_map_raw',ocup_map_raw)
		# print('obst_det3d_arr',self.obst_det3d_arr)
		if self.obst_det3d_arr is not None:
			ocup_map_cat=np.concatenate((ocup_map_raw,self.obst_det3d_arr))
		else:
			ocup_map_cat=ocup_map_raw

		# print('ocup_map cat',ocup_map_cat)
		if len(self.ocup_map)>0:

			dist_pose_obstacle = np.sqrt(np.power(ocup_map_cat[:,0:2]-pose,2).sum(axis=1))

		# print(dist_pose_obstacle)

		# plt.clf() 
		# plt.plot(path_ahead_1[:,0], path_ahead_1[:,1], 'ro')
		# plt.plot(path_ahead_2[:,0], path_ahead_2[:,1], 'bo')
		# plt.plot(ocup_map[:,0], ocup_map[:,1], 'go', markersize=6)
			ocup_map_40=ocup_map_cat[dist_pose_obstacle<40.]
			ocup_map=ocup_map_cat[dist_pose_obstacle<6.5]

		else:
			ocup_map_40=[]
			ocup_map=[]

		# plt.plot(ocup_map[:,0], ocup_map[:,1], 'yo', markersize=4)

		# plt.plot(ocup_map_40[:,0], ocup_map_40[:,1], 'yo', markersize=4)



		# for s in self.stops:
		# 	l_ob = Point(s.pose.pose.position.x, s.pose.pose.position.y)
		# 	l_ob = l_ob.buffer(1.05)###dilated obstacle

		# 	if(l_ob.intersects(self.path_shapely_dilated1)) and stop_ignore==False:
		# 		if self.firts_time_stop is None:
		# 			self.firts_time_stop=msg.stamp
		# 		print('stop warning obstacle')
		# 		print(msg.stamp)
		# 		# plt.plot(x,y, 'ko', markersize=2)
		# 		collision=True

		# 		if msg.stamp>self.firts_time_stop+5:
		# 			self.firts_time_stop=None
		# 			print('ignoring stop')
		# 			stop_ignore=True
		# 			# collision=False


		for x,y in ocup_map:

			l_ob = Point(x, y)
			l_ob = l_ob.buffer(0.05)###dilated obstacle

			if(l_ob.intersects(self.path_shapely_dilated1)):
				# print('warning obstacle')
				# plt.plot(x,y, 'ko', markersize=2)
				collision=True


				dist_o=np.sqrt(np.power(np.array([x,y])-pose,2).sum())
				if dist_o<dist:
					xo=x
					yo=y
					# print('warning follow obstacle')
					# plt.plot(x,y, 'ko', markersize=2)


					try:
						trans = self.tf2_buffer_blink2odom.lookup_transform('velodyne', 'map', rospy.Time())
						# stamp = rospy.Time().now()
						# stamp = self.curr_pose.header.stamp 
						# da = ''
						# for o in obst:

						old_pose = PoseStamped()
						old_pose.header.frame_id='map'
						old_pose.header.stamp = msg.header.stamp
						old_pose.pose.position.x = x
						old_pose.pose.position.y = y
						old_pose.pose.position.z = self.curr_pose.pose.pose.position.z
						new_pose = tf2_geometry_msgs.do_transform_pose(old_pose, trans)


						marker = Marker()
						marker.header.frame_id = "velodyne"
						marker.type = marker.CUBE
						marker.action = marker.ADD
						marker.ns = "my_namespace";

						# marker scale
						marker.scale.x = .3
						marker.scale.y = .3
						marker.scale.z = .3

						# marker color
						marker.color.a = 1.0
						marker.color.r = 1.0
						marker.color.g = 0.0
						marker.color.b = 0.0

						# marker orientaiton
						marker.pose.orientation.x = 0.0
						marker.pose.orientation.y = 0.0
						marker.pose.orientation.z = 0.0
						marker.pose.orientation.w = 1.0

						# marker position
						marker.pose.position.x = new_pose.pose.position.x
						marker.pose.position.y = new_pose.pose.position.y
						marker.pose.position.z = new_pose.pose.position.z


						t = rospy.Duration(0.35) 
						marker.lifetime = t

						# obj.lifetime = t
						# obj.type = -1
						# obj.animation_speed = 0.5;

						markerArray_follow.markers.append(marker)
						# print(markerArray)
						# obstacle_Array.obstacle.append(obj)


						obj = Obstacle()
						obj.header.frame_id = "velodyne"
						obj.header.stamp = msg.header.stamp#rospy.Time.now()
						obj.ns = "my_namespace";


						# object orientaiton
						obj.pose.orientation.x = 0
						obj.pose.orientation.y = 0
						obj.pose.orientation.z = 0
						obj.pose.orientation.w = 1

						# object position
						obj.pose.position.x = new_pose.pose.position.x
						obj.pose.position.y = new_pose.pose.position.y
						obj.pose.position.z = new_pose.pose.position.z


						obj.scale.x = 1.2
						obj.scale.y = 1.2
						obj.scale.z = 1.0

						obj.id = 1#id_obj

						obj.color.r = 255
						obj.color.g = 0
						obj.color.b = 0
						obj.color.a = 255

						obj.track_status = 1   ############## important 1


						obj.lifetime = t
						obj.type = -1
						obj.animation_speed = 0.5;

						obstacle_Array.obstacle.append(obj)

						# id_obj=1


						
					except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
						print ('[Obstacle Detection] exception waiting for tf from frames velodyne to map')

					id = 0
					for m in markerArray_follow.markers:
					   m.id = id
					   id += 1

				break

			# 	collision_constraint.reason = "\033[31m[From collision_basic_node] Hight risk of collision: \033[0m" + ob.classes[0]

		# plt.axis('equal')
		# plt.draw()
		# plt.pause(0.0001)



		for obs in self.obstacle_tfl:
			x=obs.pose.position.x
			y=obs.pose.position.y
			z=obs.pose.position.z

			l_ob = Point(x, y)
			if self.track_env=='SENSORS' or self.track_env=='MAP':
				l_ob = l_ob.buffer(min([obs.scale.x, obs.scale.y])/2)###dilated obstacle
			else:
				l_ob = l_ob.buffer(.1)###dilated obstacle dataset gt


			if(l_ob.intersects(self.path_shapely_dilated2)):
				# print('warning traffic light red')
				# plt.plot(x,y, 'ko', markersize=2)
				# print ('obs traffic light')
				# collision=True
				# break
				#########publish marker###############
				if self.tfl_red_area.get_num_connections() > 0:
					pol_viz = PolygonStamped()
					pol_viz.header.frame_id = msg.header.frame_id
					for x, y in l_ob.exterior.coords:
						p = PointStamped()
						p.point.x=x
						p.point.y=y
						p.point.z=z#self.curr_pose.pose.pose.position.z
						# print("x={}, y={}, z={}".format(p.point.x, p.point.y, p.point.z))
						pol_viz.polygon.points.append(p.point)
					self.tfl_red_area.publish(pol_viz)

			if(l_ob.intersects(self.path_shapely_dilated1) or (l_ob.intersects(self.back_l_ob_ego_dilated) and 'traffic_light_red' in obs.classes) ):
			# if(l_ob.intersects(self.path_shapely_dilated1)):# or l_ob.intersects(self.back_l_ob_ego_dilated)):
				print('stoping traffic light red')
				# plt.plot(x,y, 'ko', markersize=2)
				# print ('obs traffic light')
				collision=True
				# break
				#########publish marker###############
				if self.tfl_red_area.get_num_connections() > 0:
					pol_viz = PolygonStamped()
					pol_viz.header.frame_id = msg.header.frame_id
					for x, y in l_ob.exterior.coords:
						p = PointStamped()
						p.point.x=x
						p.point.y=y
						p.point.z=z#self.curr_pose.pose.pose.position.z
						# print("x={}, y={}, z={}".format(p.point.x, p.point.y, p.point.z))
						pol_viz.polygon.points.append(p.point)
					self.tfl_red_area.publish(pol_viz)

			if(l_ob.intersects(self.path_shapely_dilated1)) or (l_ob.intersects(self.path_shapely_dilated2)) or (l_ob.intersects(self.back_l_ob_ego_dilated) and 'traffic_light_red' in obs.classes):
				# try:
				# 	trans = self.tf2_buffer_blink2odom.lookup_transform('velodyne', 'map', rospy.Time())
				# 	# stamp = rospy.Time().now()
				# 	# stamp = self.curr_pose.header.stamp 
				# 	# da = ''
				# 	# for o in obst:

				# 	old_pose = PoseStamped()
				# 	old_pose.header.frame_id='map'
				# 	old_pose.header.stamp = msg.header.stamp
				# 	old_pose.pose.position.x = x
				# 	old_pose.pose.position.y = y
				# 	old_pose.pose.position.z = z#self.curr_pose.pose.pose.position.z
				# 	new_pose = tf2_geometry_msgs.do_transform_pose(old_pose, trans)
					
				# except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
				# 	print ('[Obstacle Detection] exception waiting for tf from frames map to velodyne')

				marker = Marker()
				marker.header.frame_id = "map"
				marker.type = marker.CUBE
				marker.action = marker.ADD
				marker.ns = "my_namespace";

				# marker scale
				marker.scale.x = 1.3
				marker.scale.y = 1.3
				marker.scale.z = 1.3

				# marker color
				marker.color.a = 1
				marker.color.r = 0#r
				marker.color.g = 0#g
				marker.color.b = 1#b

				# marker orientaiton
				marker.pose.orientation.x = 0.0
				marker.pose.orientation.y = 0.0
				marker.pose.orientation.z = 0.0
				marker.pose.orientation.w = 1.0

				# marker position
				marker.pose.position.x = x#new_pose.pose.position.x
				marker.pose.position.y = y#new_pose.pose.position.y
				marker.pose.position.z = z#new_pose.pose.position.z


				t = rospy.Duration(0.75) 
				marker.lifetime = t

				# obj.lifetime = t
				# obj.type = -1
				# obj.animation_speed = 0.5;

				markerArray_tfl.markers.append(marker)
				# print(markerArray)
				# obstacle_Array.obstacle.append(obj)


				obj = Obstacle()
				obj.header.frame_id = "map"
				obj.header.stamp = msg.header.stamp#rospy.Time.now()
				obj.ns = "my_namespace";


				# object orientaiton
				obj.pose.orientation.x = obs.pose.orientation.x
				obj.pose.orientation.y = obs.pose.orientation.y
				obj.pose.orientation.z = obs.pose.orientation.z
				obj.pose.orientation.w = obs.pose.orientation.w

				# object position
				obj.pose.position.x = obs.pose.position.x
				obj.pose.position.y = obs.pose.position.y
				obj.pose.position.z = obs.pose.position.z


				obj.scale.x = 1.2
				obj.scale.y = 1.2
				obj.scale.z = 1.0

				obj.id = 1#id_obj

				obj.color.r = 0#r
				obj.color.g = 0#g
				obj.color.b = 1#b
				obj.color.a = 1

				obj.track_status = 1   ############## important 1


				obj.lifetime = t
				obj.type = -1
				obj.animation_speed = 0.5;

				obstacle_Array_tfl.obstacle.append(obj)

		id = 0
		for m in markerArray_tfl.markers:
		   m.id = id
		   id += 1

		id = 0
		for m in obstacle_Array_tfl.obstacle:
		   m.id = id
		   id += 1







		for obs in self.obstacles:
			# print('obj:')
			# print(obs.twist.linear.x)
			# print(obs.twist.linear.y)
			angle_vel_ob=0.0
			try:
				trans = self.tf2_buffer_blink2odom.lookup_transform('map', 'velodyne',  rospy.Time())

				old_obs = PoseStamped()
				old_obs.header.frame_id='map'
				old_obs.header.stamp = msg.header.stamp
				old_obs.pose.position.x = obs.pose.position.x
				old_obs.pose.position.y = obs.pose.position.y
				old_obs.pose.position.z = obs.pose.position.z

				old_obs.pose.orientation.x = obs.pose.orientation.x
				old_obs.pose.orientation.y = obs.pose.orientation.y
				old_obs.pose.orientation.z = obs.pose.orientation.z
				old_obs.pose.orientation.w = obs.pose.orientation.w

				new_obs = tf2_geometry_msgs.do_transform_pose(old_obs, trans)

				if not(obs.twist.linear.x==0 and obs.twist.linear.y==0 and obs.twist.linear.z==0):

					old_vel = PoseStamped()
					old_vel.header.frame_id='map'
					old_vel.header.stamp = msg.header.stamp
					old_vel.pose.position.x = obs.twist.linear.x
					old_vel.pose.position.y = obs.twist.linear.y
					old_vel.pose.position.z = obs.twist.linear.z
					
					# print('old_vel :', old_vel)

					transf_vel=trans
					transf_vel.transform.translation.x=0.
					transf_vel.transform.translation.y=0.
					transf_vel.transform.translation.z=0.

					# print('new transform print:', transf_vel)
					new_vel = tf2_geometry_msgs.do_transform_pose(old_vel, transf_vel)

					# print('new_vel: ', new_vel)
					angle_vel_ob = -np.arctan2(new_vel.pose.position.x, new_vel.pose.position.y) + np.pi/2
				else:
					angle_vel_ob=0.0

				
			except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
				print ('[Obstacle Detection] exception waiting for tf from frames velodyne to map')


			# print('obs.scale.x', obs.scale.x, 'obs.scale.y', obs.scale.y)
			
			angle_ob =  euler_from_quaternion([new_obs.pose.orientation.x, new_obs.pose.orientation.y, new_obs.pose.orientation.z, new_obs.pose.orientation.w])[2]
			
			# print('angle_vel_ob, angle_ob ', angle_vel_ob, angle_ob)

			obst_det3d.append([new_obs.pose.position.x, new_obs.pose.position.y])  ## detections to points in height map
			obst_det3d.append([new_obs.pose.position.x + math.cos(angle_ob)*obs.scale.x/2 , new_obs.pose.position.y + math.sin(angle_ob)*obs.scale.y/2])
			obst_det3d.append([new_obs.pose.position.x - math.cos(angle_ob)*obs.scale.x/2 , new_obs.pose.position.y - math.sin(angle_ob)*obs.scale.y/2])

			# print('obst_det3d',obst_det3d)
			self.obst_det3d_arr=np.array(obst_det3d)
			# print('obst_det3d',self.obst_det3d_arr)


			# print ('obs dataset gt')
			# x=obs.pose.position.x
			# y=obs.pose.position.y

			# l_ob = Point(x, y)
			# l_ob = l_ob.buffer(2.5)###dilated obstacle

			# if(l_ob.intersects(self.path_shapely_dilated1)):
			# 	print('warning obstacle dataset ')
			# 	# plt.plot(x,y, 'ko', markersize=2)
			# 	collision=True
			# 	break


			# l_ob=None
			# if ob.classes[0]== "vehicle":
			# if math.fabs(angle_ori_hero-angle_ob)<0.5:
			# 	print('continue')
			# 	continue
			orig_line_obx  = new_obs.pose.position.x+math.cos(angle_ob)*obs.scale.y/2
			orig_line_oby =  new_obs.pose.position.y+math.sin(angle_ob)*obs.scale.y/2

			vel_norm=np.linalg.norm((obs.twist.linear.x,obs.twist.linear.y))

			angle_ob_corrected = angle_ob

			if obs.pose.orientation.x==0 and obs.pose.orientation.y==0 and obs.pose.orientation.z==0 and obs.pose.orientation.w==1:

				fin_line_obx = orig_line_obx+0.01
				fin_line_oby = orig_line_oby+0.01

				fin_line_obx_vel = orig_line_obx+0.01#2.5
				fin_line_oby_vel = orig_line_oby+0.01#2.5

			else:
				if vel_norm > 2. : # velocity of tracking > than 2m/s

					angle_diff=np.rad2deg(self.normalize_angle(self.normalize_angle(angle_vel_ob) - self.normalize_angle(angle_ob)))

					if abs(angle_diff)>140: #correcting predicted angle using angle of the velocity for objects with vel > 3m/s
						angle_ob = -angle_ob


				fin_line_obx = orig_line_obx+math.cos(angle_ob)*5.
				fin_line_oby = orig_line_oby+math.sin(angle_ob)*5.


				fin_line_obx_vel = orig_line_obx+math.cos(angle_ob_corrected)*vel_norm*2.5  #seconds to collision
				fin_line_oby_vel = orig_line_oby+math.sin(angle_ob_corrected)*vel_norm*2.5


			fin_line_obx_tracking = orig_line_obx+math.cos(angle_vel_ob)*vel_norm*2.5 #seconds to collision
			fin_line_oby_tracking = orig_line_oby+math.sin(angle_vel_ob)*vel_norm*2.5


			l_ob = LineString([(orig_line_obx, orig_line_oby), (fin_line_obx, fin_line_oby)])
			l_ob_vel_norm=LineString([(orig_line_obx, orig_line_oby), (fin_line_obx_vel, fin_line_oby_vel)])
			l_ob_tracking = LineString([(orig_line_obx, orig_line_oby), (fin_line_obx_tracking, fin_line_oby_tracking)])

			# l_ob_vel_norm_angle_corrected=LineString([(orig_line_obx, orig_line_oby), (fin_line_obx_vel, fin_line_oby_vel)])

				# print('angle_vel_ob ', angle_vel_ob,'angle_ob ', angle_ob)
				# print(self.normalize_angle(angle_vel_ob), self.normalize_angle(angle_ob))
				# print(self.normalize_angle(self.normalize_angle(angle_vel_ob) - self.normalize_angle(angle_ob)))
				# angle_diff=np.rad2deg(self.normalize_angle(self.normalize_angle(angle_vel_ob) - self.normalize_angle(angle_ob)))
				# print('angle_diff ', angle_diff)


				# l_ob_vel_norm_angle_corrected=LineString([(orig_line_obx, orig_line_oby), (fin_line_obx_vel, fin_line_oby_vel)])

			# ori = self.curr_pose.pose.pose.orientation
			# pose =  np.array([self.curr_pose.pose.pose.position.x, self.curr_pose.pose.pose.position.y])



			r=0
			g=1
			b=0

			if(l_ob.intersects(self.back_l_ob_ego_dilated) or l_ob_tracking.intersects(self.back_l_ob_ego_dilated) or  l_ob_vel_norm.intersects(self.back_l_ob_ego_dilated)):
				# print("\033[93m[From collision_basic_node] Persuivant npc\033[0m")
				continue

			# if (self.path_shapely_dilated1 == None or  self.path_shapely_dilated2 == None):
			# 	print('continue 2')
			# 	continue

			elif(l_ob.intersects(self.path_shapely_dilated1) or l_ob_tracking.intersects(self.path_shapely_dilated1) or l_ob_vel_norm.intersects(self.path_shapely_dilated1)):
				collision_constraint = SpeedConstraint()
				collision_constraint.header.stamp = rospy.Time().now()
				collision_constraint.speed = 0.20
				collision_constraint.reason = "\033[31m[From collision_node] Hight risk of collision incoming obstacle: \033[0m" 
				# print (collision_constraint.reason)
				self.speed_constraint_pub.publish(collision_constraint)

				if l_ob.intersects(self.path_shapely_dilated1):
					poly = l_ob.intersection(self.path_shapely_dilated1)
				else:
					poly = l_ob_tracking.intersection(self.path_shapely_dilated1)

				try:
					xx, yy = poly.coords.xy
				except:
					# print(poly)
					# print(type(poly))
					continue


				try:
					trans_map_vel = self.tf2_buffer_blink2odom.lookup_transform('velodyne', 'map',  rospy.Time())
						# stamp = rospy.Time().now()
						# stamp = self.curr_pose.header.stamp 
						# da = ''
						# for o in obst:

					old_obs = PoseStamped()
					old_obs.header.frame_id='map'
					old_obs.header.stamp = msg.header.stamp
					old_obs.pose.position.x = xx[0]
					old_obs.pose.position.y = yy[0]
					old_obs.pose.position.z = self.curr_pose.pose.pose.position.z

					# old_obs.pose.orientation.x = obs.pose.orientation.x
					# old_obs.pose.orientation.y = obs.pose.orientation.y
					# old_obs.pose.orientation.z = obs.pose.orientation.z
					# old_obs.pose.orientation.w = obs.pose.orientation.w

					new_obs_intersect = tf2_geometry_msgs.do_transform_pose(old_obs, trans_map_vel)
				except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
					print ('[Obstacle Detection] exception waiting for tf from frames velodyne to map')


				# collision=True
				r=1
				g=0
				b=1

				marker = Marker()
				marker.header.frame_id = "velodyne"
				marker.type = marker.CUBE
				marker.action = marker.ADD
				marker.ns = "my_namespace";

				# marker scale
				marker.scale.x = 0.3
				marker.scale.y = 0.3
				marker.scale.z = 0.3

				# marker color
				marker.color.a = 1
				marker.color.r = r
				marker.color.g = g
				marker.color.b = b

				# marker orientaiton
				marker.pose.orientation.x = 0.0
				marker.pose.orientation.y = 0.0
				marker.pose.orientation.z = 0.0
				marker.pose.orientation.w = 1.0

				# marker position
				marker.pose.position.x = new_obs_intersect.pose.position.x
				marker.pose.position.y = new_obs_intersect.pose.position.y
				marker.pose.position.z = new_obs_intersect.pose.position.z


				t = rospy.Duration(0.35) 
				marker.lifetime = t

				# obj.lifetime = t
				# obj.type = -1
				# obj.animation_speed = 0.5;

				markerArray_follow.markers.append(marker)
				# print(markerArray)
				# obstacle_Array.obstacle.append(obj)


				obj = Obstacle()
				obj.header.frame_id = "velodyne"
				obj.header.stamp = msg.header.stamp#rospy.Time.now()
				obj.ns = "my_namespace";


				# object orientaiton
				obj.pose.orientation.x = new_obs_intersect.pose.orientation.x
				obj.pose.orientation.y = new_obs_intersect.pose.orientation.y
				obj.pose.orientation.z = new_obs_intersect.pose.orientation.z
				obj.pose.orientation.w = new_obs_intersect.pose.orientation.w

				# object position
				obj.pose.position.x = new_obs_intersect.pose.position.x
				obj.pose.position.y = new_obs_intersect.pose.position.y
				obj.pose.position.z = new_obs_intersect.pose.position.z


				obj.scale.x = 1.2
				obj.scale.y = 1.2
				obj.scale.z = 1.0

				obj.id = 1#id_obj

				obj.color.r = r
				obj.color.g = g
				obj.color.b = b
				obj.color.a = 1

				obj.track_status = 1   ############## important 1


				obj.lifetime = t
				obj.type = -1
				obj.animation_speed = 0.5;

				# obstacle_Array.obstacle.append(obj)

			elif(l_ob.intersects(self.path_shapely_dilated2) or l_ob_tracking.intersects(self.path_shapely_dilated2)):
				# collision_constraint = SpeedConstraint()
				# collision_constraint.header.stamp = rospy.Time().now()
				# collision_constraint.speed = 2.0
				# collision_constraint.reason = "\033[93m[From collision_basic_node] Medium risk of collision: \033[0m"+ obs.classes[0]
				# print ("\033[93m[From collision_node] Medium risk of collision: \033[0m")#+ obs.classes)
				# self.speed_constraint_pub.publish(collision_constraint)
				
				# print(type(poly))
				if l_ob.intersects(self.path_shapely_dilated2):
					poly = l_ob.intersection(self.path_shapely_dilated2)
				else:
					poly = l_ob_tracking.intersection(self.path_shapely_dilated2)


				try:
					xx, yy = poly.coords.xy
				except:
					# print(poly)
					# print(type(poly))
					continue

				# print(xx,yy)

				try:
					trans_map_vel = self.tf2_buffer_blink2odom.lookup_transform('velodyne', 'map',  rospy.Time())
						# stamp = rospy.Time().now()
						# stamp = self.curr_pose.header.stamp 
						# da = ''
						# for o in obst:

					old_obs = PoseStamped()
					old_obs.header.frame_id='map'
					old_obs.header.stamp = msg.header.stamp
					old_obs.pose.position.x = xx[0]
					old_obs.pose.position.y = yy[0]
					old_obs.pose.position.z = self.curr_pose.pose.pose.position.z

					# old_obs.pose.orientation.x = obs.pose.orientation.x
					# old_obs.pose.orientation.y = obs.pose.orientation.y
					# old_obs.pose.orientation.z = obs.pose.orientation.z
					# old_obs.pose.orientation.w = obs.pose.orientation.w

					new_obs_intersect = tf2_geometry_msgs.do_transform_pose(old_obs, trans_map_vel)
				except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
					print ('[Obstacle Detection] exception waiting for tf from frames velodyne to map')

				r=1
				g=1
				b=1

				marker = Marker()
				marker.header.frame_id = "velodyne"
				marker.type = marker.CUBE
				marker.action = marker.ADD
				marker.ns = "my_namespace";

				# marker scale
				marker.scale.x = 1.3
				marker.scale.y = 1.3
				marker.scale.z = 1.3

				# marker color
				marker.color.a = 1
				marker.color.r = r
				marker.color.g = g
				marker.color.b = b

				# marker orientaiton
				marker.pose.orientation.x = 0.0
				marker.pose.orientation.y = 0.0
				marker.pose.orientation.z = 0.0
				marker.pose.orientation.w = 1.0

				# marker position
				marker.pose.position.x = new_obs_intersect.pose.position.x
				marker.pose.position.y = new_obs_intersect.pose.position.y
				marker.pose.position.z = new_obs_intersect.pose.position.z


				t = rospy.Duration(0.35) 
				marker.lifetime = t

				# obj.lifetime = t
				# obj.type = -1
				# obj.animation_speed = 0.5;

				markerArray_follow.markers.append(marker)
				# print(markerArray)
				# obstacle_Array.obstacle.append(obj)


				obj = Obstacle()
				obj.header.frame_id = "velodyne"
				obj.header.stamp = msg.header.stamp#rospy.Time.now()
				obj.ns = "my_namespace";


				# object orientaiton
				obj.pose.orientation.x = new_obs_intersect.pose.orientation.x
				obj.pose.orientation.y = new_obs_intersect.pose.orientation.y
				obj.pose.orientation.z = new_obs_intersect.pose.orientation.z
				obj.pose.orientation.w = new_obs_intersect.pose.orientation.w

				# object position
				obj.pose.position.x = new_obs_intersect.pose.position.x
				obj.pose.position.y = new_obs_intersect.pose.position.y
				obj.pose.position.z = new_obs_intersect.pose.position.z

				obj.scale.x = 1.2
				obj.scale.y = 1.2
				obj.scale.z = 1.0

				obj.id = 1#id_obj

				obj.color.r = r
				obj.color.g = g
				obj.color.b = b
				obj.color.a = 1

				obj.track_status = 1   ############## important 1

				obj.lifetime = t
				obj.type = -1
				obj.animation_speed = 0.5;

				obstacle_Array.obstacle.append(obj)



			l_marker = Marker()
			l_marker.header.frame_id = "map"#obs.header.frame_id
			l_marker.header.stamp = rospy.Time().now()
			l_marker.ns ="lines ob"
			l_marker.id = obs.id+1000
			l_marker.type=l_marker.ARROW
			l_marker.action = l_marker.ADD
			l_marker.scale.x =0.1
			l_marker.scale.y =0.1
			l_marker.scale.z =0.1

			l_marker.color.r=r
			l_marker.color.g=g
			l_marker.color.b=0
			l_marker.color.a=0.2
			l_marker.lifetime=rospy.Duration(0.35)

			p = PointStamped()

			p.point.x=orig_line_obx
			p.point.y=orig_line_oby
			p.point.z=new_obs.pose.position.z
			l_marker.points.append(p.point)

			p1 = PointStamped()

			p1.point.x=fin_line_obx
			p1.point.y=fin_line_oby
			p1.point.z=new_obs.pose.position.z
			l_marker.points.append(p1.point)
			marker_array_lines.markers.append(l_marker)

			l_marker = Marker()
			l_marker.header.frame_id = "map"#obs.header.frame_id
			l_marker.header.stamp = rospy.Time().now()
			l_marker.ns ="lines ob"
			l_marker.id = obs.id+2000
			l_marker.type=l_marker.ARROW
			l_marker.action = l_marker.ADD
			l_marker.scale.x =2.
			l_marker.scale.y =2.
			l_marker.scale.z =0.2

			l_marker.color.r=r
			l_marker.color.g=.8
			l_marker.color.b=.3
			l_marker.color.a=0.2
			l_marker.lifetime=rospy.Duration(0.35)

			p = PointStamped()

			p.point.x=orig_line_obx
			p.point.y=orig_line_oby
			p.point.z=new_obs.pose.position.z
			l_marker.points.append(p.point)

			p1 = PointStamped()

			p1.point.x=fin_line_obx_tracking
			p1.point.y=fin_line_oby_tracking
			p1.point.z=new_obs.pose.position.z
			l_marker.points.append(p1.point)
			marker_array_lines.markers.append(l_marker)
			# print('l_marker',l_marker)
			l_marker = Marker()
			l_marker.header.frame_id = "map"#obs.header.frame_id
			l_marker.header.stamp = rospy.Time().now()
			l_marker.ns ="lines ob"
			l_marker.id = obs.id+3000
			l_marker.type=l_marker.ARROW
			l_marker.action = l_marker.ADD
			l_marker.scale.x =2.
			l_marker.scale.y =2.
			l_marker.scale.z =0.1

			l_marker.color.r=r
			l_marker.color.g=1.
			l_marker.color.b=.5
			l_marker.color.a=0.2
			l_marker.lifetime=rospy.Duration(0.35)

			p = PointStamped()

			p.point.x=orig_line_obx
			p.point.y=orig_line_oby
			p.point.z=new_obs.pose.position.z
			l_marker.points.append(p.point)

			p1 = PointStamped()

			p1.point.x=fin_line_obx_vel
			p1.point.y=fin_line_oby_vel
			p1.point.z=new_obs.pose.position.z
			l_marker.points.append(p1.point)
			marker_array_lines.markers.append(l_marker)
			# print('l_marker',l_marker)

		for x,y in ocup_map_40:

			l_ob = Point(x, y)
			l_ob = l_ob.buffer(0.05)###dilated obstacle

			if(l_ob.intersects(self.path_shapely_dilated2)): #or l_ob.intersects(self.path_shapely_dilated3)):


				dist_o=np.sqrt(np.power(np.array([x,y])-pose,2).sum())
				if dist_o<dist:
					xo=x
					yo=y
					# print('warning follow obstacle')
					# plt.plot(x,y, 'ko', markersize=2)


					try:
						trans = self.tf2_buffer_blink2odom.lookup_transform('velodyne', 'map', rospy.Time())
						# stamp = rospy.Time().now()
						# stamp = self.curr_pose.header.stamp 
						# da = ''
						# for o in obst:

						old_pose = PoseStamped()
						old_pose.header.frame_id='map'
						old_pose.header.stamp = msg.header.stamp
						old_pose.pose.position.x = x
						old_pose.pose.position.y = y
						old_pose.pose.position.z = self.curr_pose.pose.pose.position.z

						new_pose = tf2_geometry_msgs.do_transform_pose(old_pose, trans)

						marker = Marker()
						marker.header.frame_id = "velodyne"
						marker.type = marker.CUBE
						marker.action = marker.ADD
						marker.ns = "my_namespace";

						# marker scale
						marker.scale.x = 0.3
						marker.scale.y = 0.3
						marker.scale.z = 0.3

						# marker color
						marker.color.a = 1.0
						marker.color.r = 1.0
						marker.color.g = 1.0
						marker.color.b = 0.0

						# marker orientaiton
						marker.pose.orientation.x = 0.0
						marker.pose.orientation.y = 0.0
						marker.pose.orientation.z = 0.0
						marker.pose.orientation.w = 1.0

						# marker position
						marker.pose.position.x = new_pose.pose.position.x
						marker.pose.position.y = new_pose.pose.position.y
						marker.pose.position.z = new_pose.pose.position.z


						t = rospy.Duration(0.35) 
						marker.lifetime = t

						# obj.lifetime = t
						# obj.type = -1
						# obj.animation_speed = 0.5;

						markerArray_follow.markers.append(marker)
						# print(markerArray)
						# obstacle_Array.obstacle.append(obj)


						obj = Obstacle()
						obj.header.frame_id = "velodyne"
						obj.header.stamp = msg.header.stamp#rospy.Time.now()
						obj.ns = "my_namespace";


						# object orientaiton
						obj.pose.orientation.x = 0
						obj.pose.orientation.y = 0
						obj.pose.orientation.z = 0
						obj.pose.orientation.w = 1

						# object position
						obj.pose.position.x = new_pose.pose.position.x
						obj.pose.position.y = new_pose.pose.position.y
						obj.pose.position.z = new_pose.pose.position.z


						obj.scale.x = 1.2
						obj.scale.y = 1.2
						obj.scale.z = 1.0

						obj.id = 1#id_obj

						obj.color.r = 255
						obj.color.g = 0
						obj.color.b = 0
						obj.color.a = 255

						obj.track_status = 1   ############## important 1


						obj.lifetime = t
						obj.type = -1
						obj.animation_speed = 0.5;

						obstacle_Array.obstacle.append(obj)

						# id_obj=1


						
					except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
						print ('[Obstacle Detection] exception waiting for tf from frames velodyne to map')



		if self.global_plan_processed is not None:
			for index, (pose_plan, option) in enumerate(zip(self.global_plan_processed.points, self.global_plan_processed.road_options)):
				

				# if self.LANEFOLLOW==option:

				x = pose_plan.x
				y = pose_plan.y

				pose_plan_arr=np.array([x,y])
				# dist_to_point_instruction = np.linalg.norm(obj_pose-pose_actual)
				dist_to_plan_point = np.linalg.norm(pose-pose_plan_arr)
				if dist_to_plan_point>30:
					continue


				l_ob = Point(x, y)
				l_ob = l_ob.buffer(1.1)###dilated obstacle

				if l_ob.intersects(self.back_l_ob_ego_dilated):
					continue

				if( l_ob.intersects(self.path_shapely_dilated1)):# or l_ob.intersects(self.path_shapely_dilated2) ):
					# if (self.STRAIGHT==option or self.RIGHT==option or self.LEFT==option or self.CHANGELANELEFT==option or self.CHANGELANERIGHT==option):
					# if (self.CHANGELANELEFT==option or self.CHANGELANERIGHT==option):
					# 	# print(' dilated1 intersection', option)

					# 	collision_constraint = SpeedConstraint()
					# 	collision_constraint.header.stamp = rospy.Time().now()
					# 	if (self.CHANGELANELEFT==option or self.CHANGELANERIGHT==option):
					# 		collision_constraint.speed = 0.25
					# 		collision_constraint.reason = "\033[31m[From collision_node] near to changelane \033[0m" 

					# 	else:
					# 		collision_constraint.speed = 0.50
					# 		collision_constraint.reason = "\033[31m[From collision_node] near to intersection \033[0m" 

					# 	# print (collision_constraint.reason)
					# 	self.speed_constraint_pub.publish(collision_constraint)


					# 	marker = Marker()
					# 	marker.header.frame_id = "map"
					# 	marker.type = marker.SPHERE
					# 	marker.action = marker.ADD
					# 	marker.ns = "my_namespace"

					# 	marker.color.a = 1.0
					# 	marker.color.r = 1.0
					# 	marker.color.g = 1.0
					# 	marker.color.b = 1.0

					# 	# marker orientaiton
					# 	marker.pose.orientation.x = 0.0
					# 	marker.pose.orientation.y = 0.0
					# 	marker.pose.orientation.z = 0.0
					# 	marker.pose.orientation.w = 1.0

					# 	# marker position
					# 	marker.pose.position.x = x
					# 	marker.pose.position.y = y
					# 	marker.pose.position.z = self.curr_pose.pose.pose.position.z-25

					# 	# marker scale
					# 	marker.scale.x = 1.
					# 	marker.scale.y = 1.
					# 	marker.scale.z = 1.3


					# 	t = rospy.Duration(5.0) 
					# 	marker.lifetime = t



						
					# 	self.markerArray.markers.append(marker)
					pass


				if( l_ob.intersects(self.path_shapely_dilated2)):# or l_ob.intersects(self.path_shapely_dilated2) ):
					if (self.STRAIGHT==option or self.RIGHT==option or self.LEFT==option) and dist_to_plan_point>10.:# or self.CHANGELANELEFT==option or self.CHANGELANERIGHT==option) \
					    
						# print('dilated2  intersection', option)


						try:
							trans = self.tf2_buffer_blink2odom.lookup_transform('velodyne', 'map', rospy.Time())
							# stamp = rospy.Time().now()
							# stamp = self.curr_pose.header.stamp 
							# da = ''
							# for o in obst:

							old_pose = PoseStamped()
							old_pose.header.frame_id='map'
							old_pose.header.stamp = msg.header.stamp
							old_pose.pose.position.x = x
							old_pose.pose.position.y = y
							old_pose.pose.position.z = self.curr_pose.pose.pose.position.z

							new_pose = tf2_geometry_msgs.do_transform_pose(old_pose, trans)

							marker = Marker()
							marker.header.frame_id = "velodyne"
							marker.type = marker.CUBE
							marker.action = marker.ADD
							marker.ns = "my_namespace";

							# marker scale
							marker.scale.x = 2
							marker.scale.y = 2.3
							marker.scale.z = 2.3

							# marker color
							marker.color.a = 1.0
							marker.color.r = 1.0
							marker.color.g = 1.0
							marker.color.b = 0.0

							# marker orientaiton
							marker.pose.orientation.x = 0.0
							marker.pose.orientation.y = 0.0
							marker.pose.orientation.z = 0.0
							marker.pose.orientation.w = 1.0

							# marker position
							marker.pose.position.x = new_pose.pose.position.x
							marker.pose.position.y = new_pose.pose.position.y
							marker.pose.position.z = new_pose.pose.position.z


							t = rospy.Duration(0.35) 
							marker.lifetime = t

							# obj.lifetime = t
							# obj.type = -1
							# obj.animation_speed = 0.5;

							markerArray_follow.markers.append(marker)
							# print(markerArray)
							# obstacle_Array.obstacle.append(obj)


							obj = Obstacle()
							obj.header.frame_id = "velodyne"
							obj.header.stamp = msg.header.stamp#rospy.Time.now()
							obj.ns = "my_namespace";


							# object orientaiton
							obj.pose.orientation.x = 0
							obj.pose.orientation.y = 0
							obj.pose.orientation.z = 0
							obj.pose.orientation.w = 1

							# object position
							obj.pose.position.x = new_pose.pose.position.x
							obj.pose.position.y = new_pose.pose.position.y
							obj.pose.position.z = new_pose.pose.position.z


							obj.scale.x = 1.2
							obj.scale.y = 1.2
							obj.scale.z = 1.0

							obj.id = 1#id_obj

							obj.color.r = 255
							obj.color.g = 0
							obj.color.b = 0
							obj.color.a = 255

							obj.track_status = 1   ############## important 1


							obj.lifetime = t
							obj.type = -1
							obj.animation_speed = 0.5;

							obstacle_Array.obstacle.append(obj)

							# id_obj=1


							
						except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
							print ('[Obstacle Detection] exception waiting for tf from frames velodyne to map')				


		id = 0
		for m in self.markerArray.markers:
		   m.id = id
		   id += 1

		id = 0
		for m in markerArray_follow.markers:
		   m.id = id
		   id += 1

		id = 0
		for m in obstacle_Array.obstacle:
		   m.id = id
		   id += 1

		self.marker_tf_red_array_pub.publish(markerArray_tfl)
		self.obstacles_array_tfl_pub.publish(obstacle_Array_tfl)

		self.marker_intersection_warning_pub.publish(self.markerArray)
		self.pub.publish(markerArray_follow)
		self.pub_obj.publish(obstacle_Array)
		self.lines_ori_obst_pub.publish(marker_array_lines)
		# self.risk_of_collision.publish(collision)
		self.risk_of_collision.publish(collision)


	def ocupation_point_cloud(self,msg):

		try:
			trans= self.tf2_buffer_blink2odom.lookup_transform('map', 'velodyne', rospy.Time())

		except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as exept:
			print(exept)
			return

		cloud_out = do_transform_cloud(msg, trans)

		pc = ros_numpy.numpify(cloud_out)
		self.ocup_map=np.zeros((pc.shape[0],2))
		# points=np.zeros((pc.shape[0],3))
		self.ocup_map[:,0]=pc['x']
		self.ocup_map[:,1]=pc['y']




	def shutdown_cb(self, msg):
		if msg.data:

			# del self.signs_object_sub
			# del self.path_sub
			# del self.shutdown_sub
			# del self.traffic_signs_pub
			self.last_path_stamp = rospy.Time().now()
			self.path_shapely_dilated1 = None
			self.path_shapely_dilated2 = None
			self.back_l_ob_ego_dilated=None
			self.curr_pose = None
			self.ocup_map = None
			self.ocup_map_f = None

			self.obst_det3d_arr=None

			self.stops= None

			self.tf2_buffer_blink2odom = None
			self.tf2_listener_blink2odom = None
			self.obstacle_tfl= None
			self.obstacles_dataset_gt= None
			self.obstacles_3d= None
			self.obstacles_mot= None
			self.obstacles_mot_stereo= None

			self.markerArray = None
			self.global_plan_received=False
			self.path = None

			self.global_plan_processed=None
			

			del self.traffic_signs_sub
			del self.poly_path1_pub
			del self.poly_path2_pub
			del self.poly_path3_pub
			del self.ego_dilated_area
			del self.tfl_red_area
			del self.risk_of_collision
			del self.obstacles_array_tfl_pub
			del self.marker_tf_red_array_pub 
			del self.marker_intersection_warning_pub 
			del self.lines_ori_obst_pub 
			del self.pub 
			del self.pub_obj
			del self.speed_constraint_pub 
			del self.pose_sub 
			del self.cnn_path_sub 
			del self.path_sub     
			del self.lidar_sub 
			del self.gt_obstacles_sub 
			del self.obstacles_3d_sub 
			del self.obstacles_mot_sub 
			del self.obstacles_mot_stereo_sub 

			del self.global_plan_raw_sub 
			del self.obstacles_array_tfl_all_sub 
			print ("Bye!")
			del self.shutdown_sub 
			rospy.signal_shutdown("CollisionDetection finished ...")