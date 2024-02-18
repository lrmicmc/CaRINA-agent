#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import PoseWithCovarianceStamped, PoseStamped, TransformStamped, Transform, Vector3, Quaternion #Twist, 
from msgs_action.msg import VehicleState#, Throttle, Brake, SteeringAngle
from std_msgs.msg import Bool, String,  Header#Float64, 
from sensor_msgs.msg import Image, PointCloud2, CameraInfo
from nav_msgs.msg import Path as RosPath

##LRM messages
from msgs_navigation.msg import GlobalPlan,  SpeedConstraint
from msgs_perception.msg import ObstacleArray, Obstacle

import ros_numpy
import tf2_ros
from tf2_geometry_msgs import *
from tf.transformations import *
from cv_bridge import CvBridge, CvBridgeError
from image_geometry import PinholeCameraModel

from pathlib import Path
from threading import Thread
import time
import numpy as np
from collections import deque 
import cv2
import time
import random
import pickle
import copy
import os

from carla_msgs.msg import CarlaEgoVehicleControl#, CarlaSpeedometer, CarlaRoute, CarlaGnssRoute, CarlaTrafficLightStatusList


class CreateDatasetCarla(object):
	def __init__ (self):

		self.root_dataset=rospy.get_param("/dataset/root_folder")
		self.train_test=rospy.get_param("/dataset/train_test_folder")

		self.steering_angle=None
		# self.steering_angle_velocity=None
		self.speed=None
		# self.acceleration=None
		# self.jerk=None
		# self.engine_speed=None
		self.throttle=None
		# self.car_gear=None
		self.brake=None
		self.hand_brake=None
		self.points_lidar=None
		self.points_stereo=None
		self.current_pose=None
		self.old_pose=None
		self.old_poses_list=None
		self.old_poses_list_gps=None

		self.cvbridge = CvBridge()
		self.rgb_image =None
		self.rgb_right_image=None
		self.depth_image =None
		self.rgb_cam_info=None
		self.stereo_bev_image =None
		self.lidar_bev_image =None
		self.seg_image=None
		self.ins_image=None
		self.aerial_image=None
		self.aerialseg_image=None
		self.img_map_foot_print_dir=None

		self.global_plan=None



		self.global_plan_points_png=None
		self.global_plan_line_png=None
		self.gps_backward_png=None
		self.gps_backward_color_png=None


		# # size_img_front=1600

		# # self.rgb_image.shape
		self.front_global_plan_points_png=None
		self.front_global_plan_points_png_depth=None
		self.front_global_plan_line_png=None


		self.max_steering = np.deg2rad(30.)
		self.next_index_goal_plan=0
		self.len_global_plan=None
		self.visited_points_global_plan=[]
		self.first_frame=True

		self.LANEFOLLOW=0
		self.STRAIGHT=1
		self.RIGHT=2
		self.LEFT=3
		self.CHANGELANELEFT=4
		self.CHANGELANERIGHT=5
		self.UNKNOWN=6

		self.n_folder=0
		self.string_folder='{:03d}'.format(self.n_folder)

		self.option_string='lanefollow'
		self.option_string_dataset='lanefollow'
		self.pose_achieved_x=None
		self.pose_achieved_y=None
		self.obstacles_frame_0=[]

		self.last_time_run=0

		self.path = np.array([])

		self.tf2_buffer_stereo2map = tf2_ros.Buffer()
		self.tf2_listener_stere2map = tf2_ros.TransformListener(self.tf2_buffer_stereo2map)

		self.tf2_buffer_map2stereo = tf2_ros.Buffer()
		self.tf2_listener_map2stereo = tf2_ros.TransformListener(self.tf2_buffer_map2stereo)

		self.tf2_buffer_vel2map = tf2_ros.Buffer()
		self.tf2_listener_vel2map = tf2_ros.TransformListener(self.tf2_buffer_vel2map)

		self.tf2_buffer_map2vel = tf2_ros.Buffer()
		self.tf2_listener_map2vel = tf2_ros.TransformListener(self.tf2_buffer_map2vel)

		#Publisher
		self.speed_constraint_pub = rospy.Publisher('/carina/control/speed_constraint', SpeedConstraint, queue_size=1)
		# self.marker_route_pub = rospy.Publisher('/carina/route/points/route_points_array', MarkerArray, queue_size=1)
		# self.pub_global_plan_img = rospy.Publisher('/carina/debug/cnn/global_plan_img', Image, queue_size=1)
		# self.pub_global_plan_line_img = rospy.Publisher('/carina/debug/cnn/global_plan_line_img', Image, queue_size=1)
		# self.pub_front_global_plan_img = rospy.Publisher('/carina/debug/cnn/front_global_plan_img', Image, queue_size=1)
		# self.pub_front_global_plan_line_img = rospy.Publisher('/carina/debug/cnn/front_global_plan_line_img', Image, queue_size=1)
		self.pub_bev_detections = rospy.Publisher('/carina/sensor/lidar/bev/foot_print_objects', Image, queue_size=1)

		#subscriber
		self.pub_option_lane_sub = rospy.Subscriber('/carina/map/option_lane', String, self.option_cmd_cb, queue_size=1)

		# self.pub_option_lane = rospy.Publisher('/carina/map/option_lane', String, queue_size=1)






		self.pub_global_plan_img = rospy.Publisher('/carina/map/global/global_plan_img', Image, queue_size=1)
		self.pub_global_plan_line_img = rospy.Publisher('/carina/map/global/global_plan_line_img', Image, queue_size=1)
		self.pub_front_global_plan_img = rospy.Publisher('/carina/map/global/front_global_plan_img', Image, queue_size=1)
		self.pub_front_global_plan_line_img = rospy.Publisher('/carina/map/global/front_global_plan_line_img', Image, queue_size=1)
		self.pub_front_global_plan_depth = rospy.Publisher('/carina/map/global/front_global_plan_depth_img', Image, queue_size=1)
		self.pub_gps_backward_png= rospy.Publisher('/carina/map/global/pub_gps_backward_png_img', Image, queue_size=1)
		self.pub_gps_backward_color_png= rospy.Publisher('/carina/map/global/pub_gps_backward_color_png_img', Image, queue_size=1)



			


		self.gt_obstacles_sub = rospy.Subscriber('/carina/perception/dataset/obstacles_array', ObstacleArray, self.obstacles_dataset_gt_cb, queue_size=1)
		# self.throttle_sub = rospy.Subscriber('/carina/control/throttle_cmd', Throttle, self.throttle_cb)
		# self.brake_sub = rospy.Subscriber('/carina/control/brake_cmd', Brake, self.brake_cb)
		# self.steer_sub = rospy.Subscriber('/carina/control/steer_cmd', SteeringAngle, self.steer_cb)
		# self.hand_brake_sub = rospy.Subscriber('/carina/control/hand_brake_cmd', Bool, self.hand_brake_cb)
		# self.pose_gt_sub = rospy.Subscriber('/carina/localization/pose_gt', PoseWithCovarianceStamped, self.pose_gt_cb, queue_size=1)
		self.pose_sub = rospy.Subscriber('/carina/localization/pose', PoseWithCovarianceStamped, self.pose_gps_cb, queue_size=1)
		self.state_sub = rospy.Subscriber('/carina/vehicle/state', VehicleState, self.vehicle_state_cb, queue_size=1)
		# self.path_sub = rospy.Subscriber("/carina/navigation/path_ros", RosPath, self.path_callback, queue_size=1)
		self.path_sub = rospy.Subscriber("/carina/navigation/path_local_ros", RosPath, self.path_callback, queue_size=1)
		self.lidar_sub = rospy.Subscriber("/carina/sensor/lidar/front/point_cloud", PointCloud2, self.lidar_point_cloud_cb)
		# self.stereo_pc_sub = rospy.Subscriber("/carina/perception/stereo/point_cloud", PointCloud2, self.stereo_point_cloud_cb)
		self.image_rgb_sub = rospy.Subscriber('/carina/sensor/camera/left/image_raw', Image, self.rgb_imageCallback, queue_size=1)
		self.img_left_info_sub = rospy.Subscriber('/carina/sensor/camera/left/camera_info', CameraInfo, self.rgb_camera_infoCallback, queue_size=1)
		self.image_rgb_right_sub = rospy.Subscriber('/carina/sensor/camera/right/image_raw', Image, self.rgb_right_imageCallback, queue_size=1)
		self.image_depth_sub = rospy.Subscriber('/carina/sensor/stereo/depth', Image, self.depth_imageCallback, queue_size=1)
		# self.image_seg_sub = rospy.Subscriber('/carina/sensor/camera/left/sem/image_raw', Image, self.seg_imageCallback, queue_size=1)
		self.image_seg_sub = rospy.Subscriber('/carla/hero/CameraSemantic/image', Image, self.seg_imageCallback, queue_size=1)
		self.image_instance_sub = rospy.Subscriber('/carla/hero/CameraInstances/image', Image, self.instance_imageCallback, queue_size=1)
		# self.image_aerial_sub = rospy.Subscriber('/carina/sensor/camera/aerial/image_raw', Image, self.aerial_imageCallback, queue_size=1)
		self.image_aerial_sub = rospy.Subscriber('/carla/hero/CameraAerial/image', Image, self.aerial_imageCallback, queue_size=1)
		self.image_occup_sub = rospy.Subscriber('/carina/map/OccupancyGrid/image_raw', Image, self.occupancy_imageCallback, queue_size=1)
		# self.image_aerial_sub = rospy.Subscriber('/carina/sensor/camera/aerial_seg/image_raw', Image, self.aerialseg_imageCallback, queue_size=1)
		self.image_aerial_sub = rospy.Subscriber('/carla/hero/CameraAerialSeg/image', Image, self.aerialseg_imageCallback, queue_size=1)
		self.image_bev_sub =        rospy.Subscriber('/carina/sensor/lidar/bev_point_cloud', Image, self.lidar_bev_imageCallback, queue_size=1)
		self.image_bev_stereo_sub = rospy.Subscriber('/carina/sensor/stereo/bev_rgb_point_cloud', Image, self.stereo_bev_imageCallback, queue_size=1)#stereo bev
		self.global_plan_raw_sub = rospy.Subscriber('/carina/navigation/global_plan_raw', GlobalPlan, self.global_plan_cb, queue_size=1)
		self.shutdown_sub = rospy.Subscriber('/carina/vehicle/shutdown', Bool, self.shutdown_cb, queue_size=1)
		self.control_carla_sub = rospy.Subscriber('/carla/hero/vehicle_control_cmd',CarlaEgoVehicleControl, self.control_carla_cb, queue_size=1)

	# def global_plan_img_cb(self,msg):
	# 	try:
	# 		self.global_plan_points_png = self.cvbridge.imgmsg_to_cv2(msg, "bgr8")
	# 	except CvBridgeError as e:
	# 		print (e)
		

	# def global_plan_line_img_cb(self,msg):
	# 	try:
	# 		self.global_plan_line_png = self.cvbridge.imgmsg_to_cv2(msg, "mono8")
	# 	except CvBridgeError as e:
	# 		print (e)		

	# def front_global_plan_img_cb(self,msg):
	# 	try:
	# 		self.front_global_plan_points_png = self.cvbridge.imgmsg_to_cv2(msg, "bgr8")
	# 	except CvBridgeError as e:
	# 		print (e)



	# def front_global_plan_line_img_cb(self,msg):
	# 	try:
	# 		self.front_global_plan_line_png = self.cvbridge.imgmsg_to_cv2(msg, "mono8")
	# 	except CvBridgeError as e:
	# 		print (e)

	# def gps_backward_png_color_cb(self,msg):
	# 	try:
	# 		self.gps_backward_png_color = self.cvbridge.imgmsg_to_cv2(msg, "bgr8")
	# 		# print(type(self.gps_backward_png[0][0]))
	# 	except CvBridgeError as e:
	# 		print (e)


	# def gps_backward_png_cb(self,msg):
	# 	try:
	# 		self.gps_backward_png = self.cvbridge.imgmsg_to_cv2(msg, "mono8")
	# 		# print(type(self.gps_backward_png[0][0]))
	# 	except CvBridgeError as e:
	# 		print (e)



	# def front_global_plan_depth_cb(self,msg):
	# 	try:
	# 		self.front_global_plan_points_png_depth = self.cvbridge.imgmsg_to_cv2(msg, "mono16")
	# 		# print(type(self.front_global_plan_points_png_depth[0][0]))
	# 	except CvBridgeError as e:
	# 		print (e)



	def option_cmd_cb(self,msg):
		self.option_string_dataset=msg.data


		# if self.option_string_dataset=='turnleft' or self.option_string_dataset=='turnright':
		# 	sp = SpeedConstraint()
		# 	sp.header.stamp = rospy.Time().now()
			# sp.speed=8.8
			#sp.speed=3.0
			# sp.speed=2.5
			# sp.speed=1.8
			# sp.speed=0.50
			# sp.reason="near to intersection (dataset node)"
			# self.speed_constraint_pub.publish(sp)
			# print ("near to intersection (dataset node)")

	def global_plan_cb(self,msg):
		self.global_plan=msg
		self.len_global_plan=len(self.global_plan.points)

	def control_carla_cb(self,control):
		# The CARLA vehicle control data
		# 0. <= throttle <= 1. float32
		self.throttle = control.throttle
		# -1. <= steer <= 1.float32
		self.steering_angle=control.steer
		# 0. <= brake <= 1. float32
		self.brake = control.brake
		# hand_brake 0 or 1  bool
		self.hand_brake = control.hand_brake

	def vehicle_state_cb(self,msg):
		self.speed = msg.drive.speed


	def stereo_bev_imageCallback(self,im):
		try:
			self.stereo_bev_image = self.cvbridge.imgmsg_to_cv2(im, "bgr8")
		except CvBridgeError as e:
			print (e)

	def lidar_bev_imageCallback(self,im):
		try:
			self.lidar_bev_image = self.cvbridge.imgmsg_to_cv2(im, "mono8")
		except CvBridgeError as e:
			print (e)

	def rgb_imageCallback(self,im):
		try:
			self.rgb_image = self.cvbridge.imgmsg_to_cv2(im, "bgr8")
		except CvBridgeError as e:
			print (e)


	def rgb_right_imageCallback(self,im):
		try:
			self.rgb_right_image = self.cvbridge.imgmsg_to_cv2(im, "bgr8")
		except CvBridgeError as e:
			print (e)

	def depth_imageCallback(self,im):
		try:
			self.depth_image = self.cvbridge.imgmsg_to_cv2(im, "mono16")#mono16: CV_16UC1, 16-bit grayscale image 
		except CvBridgeError as e:
			print (e)

	def rgb_camera_infoCallback(self,msg):
		self.rgb_cam_info=msg
		self.camera_info_left=self.rgb_cam_info

	def seg_imageCallback(self,im):
		try:
			self.seg_image = self.cvbridge.imgmsg_to_cv2(im, "bgr8")
		except CvBridgeError as e:
			print (e)

	def instance_imageCallback(self,im):
		try:
			self.ins_image = self.cvbridge.imgmsg_to_cv2(im, "bgr8")
		except CvBridgeError as e:
			print (e)

	def aerial_imageCallback(self,im):
		try:
			self.aerial_image = self.cvbridge.imgmsg_to_cv2(im, "bgr8")
		except CvBridgeError as e:
			print (e)

	def occupancy_imageCallback(self,im):
		try:
			self.occupancy_image = self.cvbridge.imgmsg_to_cv2(im, "bgr8")
		except CvBridgeError as e:
			print (e)
	def aerialseg_imageCallback(self,im):
		try:
			self.aerialseg_image = self.cvbridge.imgmsg_to_cv2(im, "bgr8")
		except CvBridgeError as e:
			print (e)

	def normalize_angle(self, angle):
		return math.atan2(np.sin(angle), np.cos(angle))

	def obstacles_dataset_gt_cb(self,msg):
		if self.current_pose is None:
			return

		self.obstacles_dataset_gt=msg.obstacle

		map_scale=8
		size_image=700
		# bev_detections= np.zeros([size_image,size_image,3],dtype=np.uint8)
		# thickness=4
		img_map=np.zeros((size_image,size_image,1), dtype=np.uint8)
		img_map_dir=np.zeros((size_image,size_image,1), dtype=np.uint8)

		pose_ego = copy.deepcopy(self.current_pose.pose)



		obstacles_frame=[]
		obstacles_frame.append(pose_ego)

		try:
			trans = self.tf2_buffer_vel2map.lookup_transform('map', 'velodyne',  rospy.Time())
		except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
			print ('[Obstacle Detection] exception waiting for tf from frames velodyne to map')

		for obs in msg.obstacle:

			x_point=obs.pose.position.y
			y_point=obs.pose.position.x
			px = int(size_image/2)+int(x_point*map_scale)
			py = int(size_image/3)+int(y_point*map_scale)
			# if py<0 :
			# 	continue
			quaternion = ( obs.pose.orientation.x, obs.pose.orientation.y, obs.pose.orientation.z, obs.pose.orientation.w )
			euler = euler_from_quaternion(quaternion)
			yaw = euler[2]

			b=127.5
			m = (0-255)/(-math.pi- math.pi)
			x=self.normalize_angle(yaw)
			dir_value_pixel=m*x+b

			# dir_value_pixel= ((self.normalize_angle(yaw) + (math.pi)) *255)/(2*math.pi)
			cv2.circle(img_map,(px, py), 2, 255, -1)
			cv2.circle(img_map_dir,(px, py), 2, int(dir_value_pixel), -1)

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

			world_obs = tf2_geometry_msgs.do_transform_pose(old_obs, trans)
			obstacles_frame.append(world_obs)
			# print('actual frame', len(obstacles_frame))

		try:
			trans_map2vel = self.tf2_buffer_map2vel.lookup_transform('velodyne', 'map', rospy.Time())
		except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as exept:
			return

		for obs in self.obstacles_frame_0:
			world_obs = PoseStamped()
			world_obs.header.frame_id='map'
			world_obs.header.stamp = msg.header.stamp
			world_obs.pose.position.x = obs.pose.position.x
			world_obs.pose.position.y = obs.pose.position.y
			world_obs.pose.position.z = obs.pose.position.z
			world_obs.pose.orientation.x = obs.pose.orientation.x
			world_obs.pose.orientation.y = obs.pose.orientation.y
			world_obs.pose.orientation.z = obs.pose.orientation.z
			world_obs.pose.orientation.w = obs.pose.orientation.w

			velo_obs = tf2_geometry_msgs.do_transform_pose(world_obs, trans_map2vel)

			x_point=velo_obs.pose.position.y
			y_point=velo_obs.pose.position.x

			pose_prev_obj=np.array([x_point,y_point])

			dist_to_ego = np.linalg.norm(pose_prev_obj)

			if dist_to_ego > 100:
				# print('dist>100', dist_to_ego)
				continue

			px = int(size_image/2)+int(x_point*map_scale)
			py = int(size_image/3)+int(y_point*map_scale)

			if py<0:
				continue

			if  px>0 and py>0 and px<size_image and py<size_image:
				if img_map[py,px]==255 :
					continue

			quaternion = ( velo_obs.pose.orientation.x, velo_obs.pose.orientation.y, velo_obs.pose.orientation.z, velo_obs.pose.orientation.w )
			euler = euler_from_quaternion(quaternion)
			yaw = euler[2]
			# point_center=(int(px), int(py))
			# l_ag=ob.scale.x
			# w_ag=ob.scale.y
			# angle=180-np.rad2deg(self.normalize_angle(yaw))
			b=127.5
			m = (0-255)/(-math.pi- math.pi)
			x=self.normalize_angle(yaw)
			# print(x)
			dir_value_pixel=m*x+b
			# print(dir_value_pixel)
			# dir_value_pixel= ((self.normalize_angle(yaw) + (math.pi)) *255)/(2*math.pi)
			cv2.circle(img_map,(px, py), 2, 255, -1)
			cv2.circle(img_map_dir,(px, py), 2, int(dir_value_pixel), -1)
			# print(img_map[py,px,:])
			# print(img_map_dir[py,px,:])
			# obstacles_w_temp.append(world_obs)
			obstacles_frame.append(world_obs)
			# print('past frames',len(obstacles_frame))
		self.obstacles_frame_0=copy.deepcopy(obstacles_frame)
			# print('obs.scale.x', obs.scale.x, 'obs.scale.y', obs.scale.y)
			# angle_ob =  euler_from_quaternion([new_obs.pose.orientation.x, new_obs.pose.orientation.y, new_obs.pose.orientation.z, new_obs.pose.orientation.w])[2]
		px = int(size_image/2)
		py = int(size_image/3)
		cv2.circle(img_map,(px, py), 5, 255, -1)
		cv2.circle(img_map_dir,(px, py), 5, 127, -1)

		img_map_mask=img_map
		ret,img_map_mask = cv2.threshold(img_map_mask,1,255,cv2.THRESH_BINARY)
		img_map_dir_color=cv2.applyColorMap(img_map_dir, cv2.COLORMAP_HSV)
		img_map_dir_color = cv2.bitwise_and(img_map_dir_color,img_map_dir_color, mask= img_map_mask)

		self.img_map_foot_print_dir=img_map_dir_color

		if self.pub_bev_detections.get_num_connections() != 0:
			try:
				self.pub_bev_detections.publish(self.cvbridge.cv2_to_imgmsg(img_map_dir_color, "bgr8"))
			except CvBridgeError as e:
				print (e)

		if len(self.obstacles_frame_0) > 2000:
			self.obstacles_frame_0=self.obstacles_frame_0[:2000]
		# print('obstacles_frame_0' , len(self.obstacles_frame_0))

	def stereo_point_cloud_cb(self,data):

		pc = ros_numpy.numpify(data)
		pc2 = pc
		pc = ros_numpy.point_cloud2.split_rgb_field(pc)

		x=pc['x']
		x=np.reshape(x, (x.shape[0]*x.shape[1]))

		y=pc['y']
		y=np.reshape(y, (y.shape[0]*y.shape[1]))

		z=pc['z']
		z=np.reshape(z, (z.shape[0]*z.shape[1]))

		r=pc['r']
		r=np.reshape(r, (r.shape[0]*r.shape[1]))

		g=pc['g']
		g=np.reshape(g, (g.shape[0]*g.shape[1]))

		b=pc['b']
		b=np.reshape(b, (b.shape[0]*b.shape[1]))

		self.arr_stereo = np.zeros(x.shape[0] + y.shape[0] + z.shape[0] + r.shape[0]  + g.shape[0] + b.shape[0], dtype=np.float32)
		self.arr_stereo[::6] = x
		self.arr_stereo[1::6] = y
		self.arr_stereo[2::6] = z
		self.arr_stereo[3::6] = r
		self.arr_stereo[4::6] = g
		self.arr_stereo[5::6] = b

		self.points_stereo=np.zeros((pc.shape[0]*pc.shape[1],6))
		self.points_stereo[:,0]=pc2['z'].reshape((pc2.shape[0]*pc2.shape[1],))
		self.points_stereo[:,1]=-pc2['x'].reshape((pc2.shape[0]*pc2.shape[1],))
		self.points_stereo[:,2]=-pc2['y'].reshape((pc2.shape[0]*pc2.shape[1],))
		pc_rgb=ros_numpy.point_cloud2.split_rgb_field(pc2)	
		self.points_stereo[:,3]=pc_rgb['r'].reshape((pc2.shape[0]*pc2.shape[1],))
		self.points_stereo[:,4]=pc_rgb['g'].reshape((pc2.shape[0]*pc2.shape[1],))
		self.points_stereo[:,5]=pc_rgb['b'].reshape((pc2.shape[0]*pc2.shape[1],))


	def lidar_point_cloud_cb(self,data):

		pc = ros_numpy.numpify(data)
		# self.points_lidar=np.zeros((pc.shape[0],44))
		# points=np.zeros((pc.shape[0],3))
		# print('lidar ',pc['x'].shape)
		x=pc['x']
		y=pc['y']
		z=pc['z']
		intensity=pc['intensity']

		self.arr = np.zeros(x.shape[0] + y.shape[0] + z.shape[0] + intensity.shape[0], dtype=np.float32)
		self.arr[::4] = x
		self.arr[1::4] = y
		self.arr[2::4] = z
		self.arr[3::4] = intensity

		pc = ros_numpy.numpify(data)
		self.points_lidar=np.zeros((pc.shape[0],3))
		# points=np.zeros((pc.shape[0],3))

		self.points_lidar[:,0]=pc['x']
		self.points_lidar[:,1]=pc['y']
		self.points_lidar[:,2]=pc['z']

	def get_empty_instance(self):
	    """Empty annotation for single instance."""
	    instance = dict(
	    	# object id
	    	id=None,
	        # (list[float], required): list of 4 numbers representing
	        # the bounding box of the instance, in (x1, y1, x2, y2) order.
	        bbox=None,
	        # (int, required): an integer in the range
	        # [0, num_categories-1] representing the category label.
	        bbox_label=None,
	        #  (list[float], optional): list of 7 (or 9) numbers representing
	        #  the 3D bounding box of the instance,
	        #  in [x, y, z, w, h, l, yaw]
	        #  (or [x, y, z, w, h, l, yaw, vx, vy]) order.
	        bbox_3d=None,
	        # (bool, optional): Whether to use the
	        # 3D bounding box during training.
	        # bbox_3d_isvalid=None,
	        # (int, optional): 3D category label
	        # (typically the same as label).
	        bbox_label_3d=None,
	        # (float, optional): Projected center depth of the
	        # 3D bounding box compared to the image plane.
	        # depth=None,
	        #  (list[float], optional): Projected
	        #  2D center of the 3D bounding box.
	        # center_2d=None,
	        # (int, optional): Attribute labels
	        # (fine-grained labels such as stopping, moving, ignore, crowd).
	        # attr_label=None,
	        # (int, optional): The number of LiDAR
	        # points in the 3D bounding box.
	        # num_lidar_pts=None,
	        # (int, optional): The number of Radar
	        # points in the 3D bounding box.
	        # num_radar_pts=None,
	        # (int, optional): Difficulty level of
	        # detecting the 3D bounding box.
	        # difficulty=None,
	        # unaligned_bbox_3d=None
	        truncated=None,
	        occluded=None,
	        alpha=None,
	        )
	    return instance

	def get_empty_lidar_points(self):
	    lidar_points = dict(
	        # (int, optional) : Number of features for each point.
	        num_pts_feats=None,
	        # (str, optional): Path of LiDAR data file.
	        lidar_path=None,
	        # (list[list[float]], optional): Transformation matrix
	        # from lidar to ego-vehicle
	        # with shape [4, 4].
	        # (Referenced camera coordinate system is ego in KITTI.)
	        #lidar2ego=None,
	    )
	    return lidar_points

	def get_empty_standard_data_info(self,
	        camera_types=['CAM0', 'CAM1', 'CAM2', 'CAM3', 'CAM4']):

	    data_info = dict(
	        # (str): Sample id of the frame.
	        sample_idx=None,
	        # (str, optional): '000010'
	        # token=None,
	        **self.get_single_image_sweep(camera_types),
	        # (dict, optional): dict contains information
	        # of LiDAR point cloud frame.
	        lidar_points=self.get_empty_lidar_points(),
	        # (dict, optional) Each dict contains
	        # information of Radar point cloud frame.
	        # radar_points=get_empty_radar_points(),
	        # (list[dict], optional): Image sweeps data.
	        # image_sweeps=[],
	        # lidar_sweeps=[],
	        instances=[],
	        # (list[dict], optional): Required by object
	        # detection, instance  to be ignored during training.
	        instances_ignore=[])#,
	        # (str, optional): Path of semantic labels for each point.
	        # pts_semantic_mask_path=None,
	        # (str, optional): Path of instance labels for each point.
	        # pts_instance_mask_path=None)
	    return data_info

	def get_single_image_sweep(self,camera_types):
	    single_image_sweep = dict(
	        timestamp=None,
	        ego2global=None)
	    images = dict()
	    for cam_type in camera_types:
	        images[cam_type] = self.get_empty_img_info()
	    single_image_sweep['images'] = images
	    return single_image_sweep

	def get_empty_img_info(self):
	    img_info = dict(
	        # (str, required): the path to the image file.
	        img_path=None,
	        # (int) The height of the image.
	        height=100,#None,
	        # (int) The width of the image.
	        width=100,#None,
	        # (str, optional): Path of the depth map file
	        depth_map=None,
	        # (list[list[float]], optional) : Transformation
	        # matrix from camera to image with
	        # shape [3, 3], [3, 4] or [4, 4].
	        cam2img=[[1,1,1],[1,1,1],[1,1,1]],#None,
	        # (list[list[float]]): Transformation matrix from lidar
	        # or depth to image with shape [4, 4].
	        lidar2img=[[1,1,1],[1,1,1],[1,1,1]],#None,
	        # (list[list[float]], optional) : Transformation
	        # matrix from camera to ego-vehicle
	        # with shape [4, 4].
	        cam2ego=None,
	        lidar2cam=[[1,1,1],[1,1,1],[1,1,1]],)#None,)
	    return img_info

	def path_callback(self, msg):
		self.path=msg
		self.save_frame_dataset(msg,self.option_string_dataset)


	def pose_gps_cb(self,msg):
		# self.current_pose_gps = msg
		self.current_pose = msg

		if self.speed > 0.5:
			self.last_time_run = rospy.Time().now().to_sec()#msg.header.stamp.to_sec()

		time_stoped = (rospy.Time().now().to_sec() - self.last_time_run)

		if time_stoped > 2.:
			self.save_frame_dataset(self.path,self.option_string_dataset)
			self.last_time_run = rospy.Time().now().to_sec()#msg.header.stamp.to_sec()
		# if self.old_poses_list_gps==None:
		# 	self.old_poses_list_gps = [[self.current_pose_gps] for i in range(200)]
		# else:
		# 	deque_old_poses_list_gps = deque(self.old_poses_list_gps) 
		# 	deque_old_poses_list_gps.rotate(1) 
		# 	self.old_poses_list_gps = list(deque_old_poses_list_gps)
		# 	self.old_poses_list_gps[0] = [self.current_pose_gps]

	def save_frame_dataset(self, path, option_string_dataset):

		# if self.global_plan==None or :
		# 	return
		# print('data received')
		# # print ('points_stereo',self.points_stereo is None)
		# print ('current_pose',self.current_pose is None )
		# print ('rgb_left',self.rgb_image is None)
		# print ('rgb_right',self.rgb_right_image is None)
		# print ('stereo_bev_image',self.stereo_bev_image is None)
		# print ('lidar_bev_image',self.lidar_bev_image is None)
		# print ('aerial_image',self.aerial_image is None)
		# print ('occupancy_image',self.occupancy_image is None)
		# print ('aerialseg_image',self.aerialseg_image is None)
		# print ('seg_image',self.seg_image is None )
		# print ('seg_image',self.ins_image is None )
		# print ('global_plan',self.global_plan is None)

		if option_string_dataset=='lanefollow':
			rand=random.random()
			# print(rand)
			if rand >0.1:
				# print('return lanefollow')
				return


		if self.current_pose is None or self.rgb_image is None or self.stereo_bev_image is None or self.lidar_bev_image is None \
			or self.aerial_image is None or self.aerialseg_image is None  or self.seg_image is None or self.global_plan is None \
			or self.img_map_foot_print_dir is None: 
			# print(self.rgb_image, self.stereo_bev_image, self.lidar_bev_image, self.seg_image)
			return
		scale=8
		size_image=700  #size BEV

		#print('get new path ')
		route_pred_gt=path
		# old_pose=self.old_pose
		current_pose=copy.deepcopy(self.current_pose)
		points_lidar=copy.deepcopy(self.points_lidar)
		# points_stereo=self.points_stereo.copy()
		# old_poses_list=copy.deepcopy(self.old_poses_list_gps)
		global_plan=copy.deepcopy(self.global_plan)
		rgb_image=copy.deepcopy(self.rgb_image)
		rgb_right_image=copy.deepcopy(self.rgb_right_image)
		depth_image=copy.deepcopy(self.depth_image)

		seg_image=copy.deepcopy(self.seg_image)
		ins_image=copy.deepcopy(self.ins_image)
		aerial_image=copy.deepcopy(self.aerial_image)
		occupancy_image=copy.deepcopy(self.occupancy_image)
		aerialseg_image=copy.deepcopy(self.aerialseg_image)
		lidar_bev_image=copy.deepcopy(self.lidar_bev_image)
		stereo_bev_image=copy.deepcopy(self.stereo_bev_image)
		img_map_foot_print_dir=copy.deepcopy(self.img_map_foot_print_dir)


		# global_plan_points_png= copy.deepcopy(self.global_plan_points_png)
		# global_plan_line_png= copy.deepcopy(self.global_plan_line_png)
		# gps_backward_png= copy.deepcopy(self.gps_backward_png)
		# gps_backward_png_color= copy.deepcopy(self.gps_backward_png_color)


		# # size_img_front=1600

		# # self.rgb_image.shape
		# front_global_plan_points_png= copy.deepcopy(self.front_global_plan_points_png)
		# front_global_plan_points_png_depth=copy.deepcopy(self.front_global_plan_points_png_depth)
		# front_global_plan_points_png_depth= numpy.zeros([size_img_front,size_img_front,1],dtype=np.uint16)
		# front_global_plan_line_png= copy.deepcopy(self.front_global_plan_line_png)


		gps_b_c_png, gps_b_png, g_p_points_png, g_p_line_png, f_g_p_points_png, f_g_p_line_png, f_g_p_points_png_depth=self.publish_plan_img(current_pose, global_plan)
		
		gps_backward_png_color= copy.deepcopy(gps_b_c_png)
		gps_backward_png= copy.deepcopy(gps_b_png)
		global_plan_points_png= copy.deepcopy(g_p_points_png)
		global_plan_line_png= copy.deepcopy(g_p_line_png)
		front_global_plan_points_png= copy.deepcopy(f_g_p_points_png)
		front_global_plan_line_png= copy.deepcopy(f_g_p_line_png)
		front_global_plan_points_png_depth=copy.deepcopy(f_g_p_points_png_depth)

		# front_global_plan_points_png= front_global_plan_points_png
		# front_global_plan_points_png_depth= numpy.zeros([ self.rgb_image.shape[0], self.rgb_image.shape[1], 1], dtype=np.uint16)
		# front_global_plan_line_png= front_global_plan_line_png

		# path_pred_png= numpy.zeros([size_image,size_image,1],dtype=np.uint8)

		try:
			trans_stamp_inv= self.tf2_buffer_map2vel.lookup_transform('velodyne','map', rospy.Time())

		except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as exept:
			print(exept)
			return

		stamp = rospy.Time().now()
		# try:
		# 	trans_stamp_inv_stereo_to_map = self.tf2_buffer_map2stereo.lookup_transform('stereo','map', rospy.Time())
		# except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as exept:
		# 	print(exept)
		# 	return

		if self.old_pose==None or current_pose==None : #or global_plan==None:
			dist=10000
		else:
			dist=numpy.linalg.norm(np.array([self.old_pose.pose.pose.position.x, self.old_pose.pose.pose.position.y]) - np.array([current_pose.pose.pose.position.x, current_pose.pose.pose.position.y]))



		if self.first_frame:
			self.first_frame=False
			# print('first frame')
			return
		classes_list=[]
		for obs in self.obstacles_dataset_gt:
			for c in obs.classes:
				classes_list.append(c)

		# print('pedestrian in this frame: ',('CLASSIFICATION_PEDESTRIAN' in classes_list))
		# print('self.option_string ', self.option_string)
		# if dist>0.1 or self.option_string=='turnleft' or self.option_string=='turnright' or self.option_string=='changelaneright' \
		# 								or ('CLASSIFICATION_PEDESTRIAN' in classes_list) or self.option_string=='changelaneleft':
		# if ('CLASSIFICATION_PEDESTRIAN' in classes_list):

		self.old_pose=current_pose
		#print("dist > 0.1")
		#print('vehicle state: steering angle', self.steering_angle, 'speed', self.speed, 'throttle',self.throttle, 'brake',self.brake, 'h brake',self.hand_brake)

		# for p in old_poses_list:
		# 	new_tfmed_gps = tf2_geometry_msgs.do_transform_pose(p[0].pose, trans_stamp_inv)
		# 	poses_back.append( [(new_tfmed_gps.pose.position.y*scale)+(size_image/2), (new_tfmed_gps.pose.position.x*scale)+(size_image/3)] )

		# birdeyeview = self.lidar_bev_image#numpy.zeros([size_image,size_image,3],dtype=np.uint8)
		# birdeyeview_stereo = self.stereo_bev_image#numpy.zeros([size_image,size_image,3],dtype=np.uint8)
		# points_traj_past = numpy.zeros([size_image,size_image,3],dtype=np.uint8)
		# for point in points_stereo:
		# 	ximage=int((point[0]*scale)+(size_image/3))
		# 	yimage=int((point[1]*scale)+(size_image/2))
		# 	if ximage < size_image and yimage<size_image and ximage >=0 and yimage>=0 and point[2]>-5 and point[2]<5:
		# 		birdeyeview_stereo[ximage][yimage]=[point[5],point[4],point[3]]
		# for point in points_lidar:
		# 	ximage=int((point[0]*scale)+(size_image/3))
		# 	yimage=int((point[1]*scale)+(size_image/2))
		# 	if ximage < size_image and yimage<size_image and ximage >=0 and yimage>=0:
		# 		r =g=b=int(point[2]+5)*50
		# 		# birdeyeview[ximage][yimage]=[r,g,b]
		# 		birdeyeview_stereo[ximage][yimage]=[r,g,b]
		# birdeyeview_3ch = np.zeros_like(birdeyeview_stereo)
		# birdeyeview_3ch[:,:,0] = birdeyeview
		# birdeyeview_3ch[:,:,1] = birdeyeview
		# birdeyeview_3ch[:,:,2] = birdeyeview
		# cv2.imshow('seg image', self.rgb_image)
		# cv2.waitKey(1)
		# cv2.imshow('rgb image', self.seg_image)
		# cv2.waitKey(1)
		# cv2.imshow('bev points back', points_traj_past)
		# cv2.waitKey(1)
		im_color_lidar = cv2.applyColorMap(lidar_bev_image, cv2.COLORMAP_JET)
		im_color_lidar = cv2.bitwise_and(im_color_lidar, im_color_lidar, mask=lidar_bev_image)

		#birdeyeview_stereo = cv2.addWeighted(im_color_lidar,0.9,birdeyeview_stereo,0.9,0)
		# dst = cv2.addWeighted(points_traj_past, 0.7,dst, 0.8, 0)
		# cv2.imshow('dst', dst)
		# cv2.waitKey(1)
		# obst = self.obstacles_array
		# obst_pose = []
		# markerArray = MarkerArray()
		# markerArray.markers=[]


		#cv2.imshow('bev stereoimage', birdeyeview_stereo)
		#cv2.waitKey(1)
		# id = 0
		# for m in markerArray.markers:
		#    m.id = id
		#    id += 1
		# self.marker_route_pub.publish(markerArray)

		root_dataset=self.root_dataset #"/mnt/Data1/carla_dataset"
		# test="test"
		# train="train"

		train_test=self.train_test #"validation","train","test"
		# train_test=

		inputs='inputs'
		gt='gt'

		rgb_img='rgb_left'
		rgb_right_img='rgb_right'

		depth_img='depth_front'

		seg_img='segmentation_front'
		ins_img='instance_front'

		aerial_seg_img='segmentation_aerial'
		aerial_rgb_img='rgb_aerial'
		occupancy_img='occupancy'

		stereo_bev_img='stereo_bev'
		lidar_bev_img='lidar_bev'
		lidar_bev_color_img='lidar_bev_color'

		img_map_foot_print='objs_foot_print'

		depth_color_img='front_points_depth_color'

		global_plan_imgs='global_plan'
		global_plan_points_img='points'
		global_plan_line_img='img_line'

		front_global_plan_imgs='front_global_plan'
		front_global_plan_points_img='front_points'
		front_global_plan_points_img_depth='front_points_depth'

		front_global_plan_line_img='front_img_line'

		gps_backward='gps_backward'
		# gps_backward_txt='points'
		gps_backward_img='img_line'
		gps_backward_color_img='img_line_color'

		options='options'

		path_pred='path_pred'
		# path_pred_txt='txt_line'
		path_pred_img='img_line'
		vehicle_state_txt='vehicle_state'

		# # now = rospy.get_rostime()
		# now=rospy.Time.now()
		# rospy.loginfo("Current time %i %i", now.secs, now.nsecs)
		# filename='{:06d}'.format(now.secs) + '{:09d}'.format(now.nsecs)#   str(now.secs)+str(now.nsecs)
		# print(filename)
		#print('time linux',time.time_ns() )
		filename=str(time.time_ns())

		classes_list=[]

		for obs in self.obstacles_dataset_gt:
			for c in obs.classes:
				classes_list.append(c)

		gt_names=[]
		gt_bboxes_3d=[]
		truncated=[]
		occluded=[]
		alpha=[]
		bbox=[]
		dimensions=[]
		location=[]
		rotation_y=[]
		score=[]
		index=[]
		group_ids=[]
		difficulty=[]
		num_points_in_gt=[]
		# filename=int(time.time_ns())

		METAINFO = { 'classes':   ('Car', 'Cyclist', 'Pedestrian'),    }#,'Van','Motorcycle','Truck'),    }

	# if ('CLASSIFICATION_VAN' in classes_list) or ('CLASSIFICATION_BIKE' in classes_list) or ('CLASSIFICATION_MOTORCYCLE' in classes_list) or  \
	# 	('CLASSIFICATION_TRUCK' in classes_list) or ('CLASSIFICATION_OTHER_VEHICLE' in classes_list) or ('CLASSIFICATION_PEDESTRIAN' in classes_list):

		instance_list = []
		for obs in self.obstacles_dataset_gt:
			empty_instance = self.get_empty_instance()
			
			x = obs.pose.position.x
			y = obs.pose.position.y
			z = obs.pose.position.z

			angle_ob = euler_from_quaternion([obs.pose.orientation.x,	obs.pose.orientation.y, obs.pose.orientation.z, obs.pose.orientation.w])

			w=obs.scale.x
			l=obs.scale.y
			h=obs.scale.z

			if obs.classes[0]=='CLASSIFICATION_BIKE':
				c='Cyclist'
			elif obs.classes[0]=='CLASSIFICATION_MOTORCYCLE':
				c='Cyclist'
			elif obs.classes[0]=='CLASSIFICATION_PEDESTRIAN':
				c='Pedestrian'
			elif obs.classes[0]=='CLASSIFICATION_OTHER_VEHICLE'or obs.classes[0]=='CLASSIFICATION_CAR':
				c='Car'
			elif obs.classes[0]=='CLASSIFICATION_TRUCK':
				c='Car'
			elif obs.classes[0]=='CLASSIFICATION_VAN':
				c='Car'
			else:
				c='Car'
			empty_instance['bbox_label'] = METAINFO['classes'].index(c)
			empty_instance['bbox_label_3d'] = copy.deepcopy( empty_instance['bbox_label'])
			empty_instance['bbox'] = [0,0,0,0]
			empty_instance['bbox_3d'] = [x,y,z,w,l,h,angle_ob[2]]
			empty_instance['id'] = [obs.id]
			empty_instance['truncated']=0.
			empty_instance['occluded']=0
			empty_instance['alpha']=0.
			empty_instance['score']=1.
			instance_list.append(empty_instance)
		temp_data_info = self.get_empty_standard_data_info()

		temp_data_info['sample_idx']=filename
		temp_data_info['lidar_points']['num_pts_feats']=4
		temp_data_info['lidar_points']['lidar_path']= 'training/velodyne/points/'+str(filename)+'.bin'
		temp_data_info['instances'] = instance_list

		# root_dataset='/mnt/Data1/dataset_carla_tracking'
		prefix='pointcloud'
		#train_test='validation'#'training'
		velodyne='velodyne'
		stereo='stereo'
		labels='labels'

		path_labels_velodyne  =os.path.join(root_dataset,train_test,option_string_dataset,gt,prefix,velodyne,labels)
		# path_labels_stereo  =os.path.join(root_dataset,train_test,prefix,stereo,labels)

		path_velodyne=os.path.join(root_dataset,train_test,option_string_dataset,inputs,prefix,velodyne,'points')
		# path_stereo=os.path.join(root_dataset,train_test,prefix,stereo,'points')

		Path(path_labels_velodyne).mkdir(parents=True, exist_ok=True)
		# Path(path_labels_stereo).mkdir(parents=True, exist_ok=True)

		Path(path_velodyne).mkdir(parents=True, exist_ok=True)
		# Path(path_stereo).mkdir(parents=True, exist_ok=True)

		with open(os.path.join(path_labels_velodyne,str(filename)+'.pkl'),  'wb') as handle:
			pickle.dump(temp_data_info, handle, protocol=pickle.HIGHEST_PROTOCOL)
		# temp_data_info['lidar_points']['lidar_path']= 'training/stereo/points/'+str(filename)+'.bin'
		# with open(os.path.join(path_labels_stereo,str(filename)+'.pkl'),  'wb') as handle:
		# 	pickle.dump(temp_data_info, handle, protocol=pickle.HIGHEST_PROTOCOL)

		self.arr.astype('float32').tofile(os.path.join(path_velodyne,str(filename)+'.bin'))
		# self.arr_stereo.astype('float32').tofile(os.path.join(path_stereo,str(filename)+'.bin'))

		# print(self.string_folder)
		# Path(os.path.join(root_dataset,train_test,option_string_dataset,inputs,rgb_img,self.string_folder)).mkdir(parents=True, exist_ok=True)
		# cv2.imwrite(os.path.join(root_dataset,train_test,option_string_dataset,inputs,rgb_img,self.string_folder,filename+'.jpg'), rgb_image)
		Path(os.path.join(root_dataset,train_test,option_string_dataset,inputs,rgb_img)).mkdir(parents=True, exist_ok=True)
		cv2.imwrite(os.path.join(root_dataset,train_test,option_string_dataset,inputs,rgb_img,filename+'.jpg'), rgb_image)

		# Path(os.path.join(root_dataset,train_test,option_string_dataset,inputs,rgb_right_img,self.string_folder)).mkdir(parents=True, exist_ok=True)
		# cv2.imwrite(os.path.join(root_dataset,train_test,option_string_dataset,inputs,rgb_right_img,self.string_folder,filename+'.jpg'), rgb_right_image)
		
		Path(os.path.join(root_dataset,train_test,option_string_dataset,inputs,rgb_right_img)).mkdir(parents=True, exist_ok=True)
		cv2.imwrite(os.path.join(root_dataset,train_test,option_string_dataset,inputs,rgb_right_img,filename+'.jpg'), rgb_right_image)

		# Path(os.path.join(root_dataset,train_test,inputs,depth_img)).mkdir(parents=True, exist_ok=True)
		# cv2.imwrite(os.path.join(root_dataset,train_test,inputs,depth_img,filename+'.png'), depth_image)

		Path(os.path.join(root_dataset,train_test,option_string_dataset,gt,seg_img)).mkdir(parents=True, exist_ok=True)
		cv2.imwrite(os.path.join(root_dataset,train_test,option_string_dataset,gt,seg_img,filename+'.png'), seg_image)

		Path(os.path.join(root_dataset,train_test,option_string_dataset,gt,ins_img)).mkdir(parents=True, exist_ok=True)
		cv2.imwrite(os.path.join(root_dataset,train_test,option_string_dataset,gt,ins_img,filename+'.png'), ins_image)

		aerial_image=cv2.flip(aerial_image,-1)
		Path(os.path.join(root_dataset,train_test,option_string_dataset,gt, aerial_rgb_img)).mkdir(parents=True, exist_ok=True)
		cv2.imwrite(os.path.join(root_dataset,train_test,option_string_dataset,gt,aerial_rgb_img,filename+'.png'), aerial_image)

		Path(os.path.join(root_dataset,train_test,option_string_dataset,gt, occupancy_img)).mkdir(parents=True, exist_ok=True)
		cv2.imwrite(os.path.join(root_dataset,train_test,option_string_dataset,gt,occupancy_img,filename+'.png'), occupancy_image)

		aerialseg_image=cv2.flip(aerialseg_image,-1)
		Path(os.path.join(root_dataset,train_test,option_string_dataset,gt, aerial_seg_img)).mkdir(parents=True, exist_ok=True)
		cv2.imwrite(os.path.join(root_dataset,train_test,option_string_dataset,gt,aerial_seg_img,filename+'.png'), aerialseg_image)


		Path(os.path.join(root_dataset,train_test,option_string_dataset,inputs,lidar_bev_color_img)).mkdir(parents=True, exist_ok=True)
		cv2.imwrite(os.path.join(root_dataset,train_test,option_string_dataset,inputs,lidar_bev_color_img,filename+'.png'), im_color_lidar)

		Path(os.path.join(root_dataset,train_test,option_string_dataset,inputs,stereo_bev_img)).mkdir(parents=True, exist_ok=True)
		cv2.imwrite(os.path.join(root_dataset,train_test,option_string_dataset,inputs,stereo_bev_img,filename+'.png'), stereo_bev_image)

		Path(os.path.join(root_dataset,train_test,option_string_dataset,inputs,lidar_bev_img)).mkdir(parents=True, exist_ok=True)
		cv2.imwrite(os.path.join(root_dataset,train_test,option_string_dataset,inputs,lidar_bev_img,filename+'.png'), lidar_bev_image)

		Path(os.path.join(root_dataset,train_test,option_string_dataset,inputs,img_map_foot_print)).mkdir(parents=True, exist_ok=True)
		cv2.imwrite(os.path.join(root_dataset,train_test,option_string_dataset,inputs,img_map_foot_print,filename+'.png'), img_map_foot_print_dir)


		# Path(os.path.join(root_dataset,train_test,inputs,front_global_plan_imgs,depth_color_img)).mkdir(parents=True, exist_ok=True)
		# cv2.imwrite(os.path.join(root_dataset,train_test,inputs,front_global_plan_imgs,depth_color_img,filename+'.png'), im_color_depth)

		Path(os.path.join(            root_dataset,train_test,option_string_dataset,inputs,global_plan_imgs,global_plan_points_img)).mkdir(parents=True, exist_ok=True)
		cv2.imwrite(os.path.join(     root_dataset,train_test,option_string_dataset,inputs,global_plan_imgs,global_plan_points_img,filename+'.png'), global_plan_points_png)

		Path(os.path.join(            root_dataset,train_test,option_string_dataset,inputs,global_plan_imgs,global_plan_line_img)).mkdir(parents=True, exist_ok=True)
		cv2.imwrite(os.path.join(     root_dataset,train_test,option_string_dataset,inputs,global_plan_imgs,global_plan_line_img,filename+'.png'), global_plan_line_png)

		Path(os.path.join(            root_dataset,train_test,option_string_dataset,inputs,front_global_plan_imgs,front_global_plan_points_img)).mkdir(parents=True, exist_ok=True)
		cv2.imwrite(os.path.join(     root_dataset,train_test,option_string_dataset,inputs,front_global_plan_imgs,front_global_plan_points_img,filename+'.png'), front_global_plan_points_png)

		Path(os.path.join(            root_dataset,train_test,option_string_dataset,inputs,front_global_plan_imgs,front_global_plan_line_img)).mkdir(parents=True, exist_ok=True)
		cv2.imwrite(os.path.join(     root_dataset,train_test,option_string_dataset,inputs,front_global_plan_imgs,front_global_plan_line_img,filename+'.png'), front_global_plan_line_png)

		Path(os.path.join(            root_dataset,train_test,option_string_dataset,inputs,front_global_plan_imgs,front_global_plan_points_img_depth)).mkdir(parents=True, exist_ok=True)
		cv2.imwrite(os.path.join(     root_dataset,train_test,option_string_dataset,inputs,front_global_plan_imgs,front_global_plan_points_img_depth,filename+'.png'), front_global_plan_points_png_depth)

		# print(Path(os.path.join(            root_dataset,train_test,option_string_dataset,inputs,front_global_plan_imgs,front_global_plan_points_img_depth)))

		# if self.pub_global_plan_img.get_num_connections() != 0:
		# 	try:
		# 		self.pub_global_plan_img.publish(self.cvbridge.cv2_to_imgmsg(global_plan_points_png, "bgr8"))
		# 	except CvBridgeError as e:
		# 		print (e)

		# if self.pub_global_plan_line_img.get_num_connections() != 0:
		# 	try:
		# 		self.pub_global_plan_line_img.publish(self.cvbridge.cv2_to_imgmsg(global_plan_line_png, "mono8"))
		# 	except CvBridgeError as e:
		# 		print (e)
		# if self.pub_front_global_plan_img.get_num_connections() != 0:
		# 	try:
		# 		self.pub_front_global_plan_img.publish(self.cvbridge.cv2_to_imgmsg(front_global_plan_points_png, "bgr8"))
		# 	except CvBridgeError as e:
		# 		print (e)

		# if self.pub_front_global_plan_line_img.get_num_connections() != 0:
		# 	try:
		# 		self.pub_front_global_plan_line_img.publish(self.cvbridge.cv2_to_imgmsg(front_global_plan_line_png, "mono8"))
		# 	except CvBridgeError as e:
		# 		print (e)


		# header_list=[]
		# header_list.append('id')
		# raw_path_points_planelist=[]
		# for i,pxy in enumerate(raw_path_points):
		# 	# print(pxy)
		# 	raw_path_points_planelist.append(pxy[0])
		# 	raw_path_points_planelist.append(pxy[1])
		# 	header_list.append('px'+str(i))
		# 	header_list.append('py'+str(i))

		# header_str=','.join(map(str, header_list))+"\n"

		# raw_path_points_str=','.join(map(str, raw_path_points_planelist))+"\n"
		# path = os.path.join(root_dataset,train_test,option_string_dataset,inputs,global_plan_imgs,'global_plan_to_local.txt')
		# # print(path)
		# file = Path(path)
		# if file.exists():
		# 	#print(path+" exists") 
		# 	f = open(file, 'a')
		# 	f.write(filename +","+raw_path_points_str)
		# 	f.close()
		# else:
		# 	#print(path+"does not exist") 
		# 	f = open(file, "a")
		# 	f.write(header_str)
		# 	f.write(filename +","+raw_path_points_str)
		# 	f.close()

		# header_list=[]
		# header_list.append('id')
		# raw_local_points_uv_planelist=[]
		# for i,pxy in enumerate(raw_local_points_uv):
		# 	raw_local_points_uv_planelist.append(pxy[0])
		# 	raw_local_points_uv_planelist.append(pxy[1])
		# 	header_list.append('px'+str(i))
		# 	header_list.append('py'+str(i))
		# header_str=','.join(map(str, header_list))+"\n"

		# raw_local_points_uv_str=','.join(map(str, raw_local_points_uv_planelist))+"\n"
		# path = os.path.join(root_dataset,train_test,option_string_dataset,inputs,global_plan_imgs,'global_plan_to_local_cam_front.txt')
		# file = Path(path)
		# if file.exists():
		# 	#print(path+" exists") 
		# 	f = open(file, 'a')
		# 	f.write(filename +","+raw_local_points_uv_str)
		# 	f.close()
		# else:
		# 	#print(path+"does not exist") 
		# 	f = open(file, "a")
		# 	f.write(header_str)
		# 	f.write(filename +","+raw_local_points_uv_str)
		# 	f.close()
		path_pred_png= np.zeros([size_image,size_image,1],dtype=np.uint8)

		# poses_back=[]
		poses_pred_gt=[]
		poses_pred_gt_to_txt=[]
		
		for p in route_pred_gt.poses:
			new_route_pred_gt = tf2_geometry_msgs.do_transform_pose(p, trans_stamp_inv)
			poses_pred_gt.append( [(new_route_pred_gt.pose.position.y*scale)+(size_image/2), (new_route_pred_gt.pose.position.x*scale)+(size_image/3)] )
			poses_pred_gt_to_txt.append( [new_route_pred_gt.pose.position.x, new_route_pred_gt.pose.position.y] )



		pts = np.array(poses_pred_gt, np.int32)
		pts = pts.reshape((-1,1,2))
		# cv2.polylines(points_traj_past,[pts],False,(255,0,0))			
		# cv2.polylines(birdeyeview_stereo,[pts],False,(0,0,255))	
		cv2.polylines(path_pred_png,[pts],False,(255))	


		# header_list=[]
		# header_list.append('id')
		# poses_back_planelist=[]
		# for i,pxy in enumerate(poses_back):
		# 	poses_back_planelist.append(pxy[0])
		# 	poses_back_planelist.append(pxy[1])
		# 	header_list.append('px'+str(i))
		# 	header_list.append('py'+str(i))



		# header_str=','.join(map(str, header_list))+"\n"


		# poses_back_str=','.join(map(str, poses_back_planelist))+"\n"
		Path(os.path.join(            root_dataset,train_test,option_string_dataset,inputs,gps_backward,gps_backward_img)).mkdir(parents=True, exist_ok=True)
		cv2.imwrite(os.path.join(     root_dataset,train_test,option_string_dataset,inputs,gps_backward,gps_backward_img,filename+'.png'), gps_backward_png)

		Path(os.path.join(            root_dataset,train_test,option_string_dataset,inputs,gps_backward,gps_backward_color_img)).mkdir(parents=True, exist_ok=True)
		cv2.imwrite(os.path.join(     root_dataset,train_test,option_string_dataset,inputs,gps_backward,gps_backward_color_img,filename+'.png'), gps_backward_png_color)		# path = os.path.join(root_dataset,train_test,option_string_dataset,inputs,gps_backward,'gps_backward.txt')
		
		# file = Path(path)
		# if file.exists():
		# 	f = open(file, 'a')
		# 	f.write(filename+","+poses_back_str)
		# 	f.close()
		# else:
		# 	f = open(file, "a")
		# 	f.write(header_str)
		# 	f.write(filename+","+poses_back_str)
		# 	f.close()

		# header_list=[]
		# header_list.append('id')
		# header_list.append('option')
		# poses_back_planelist=[]
		# header_str=','.join(map(str, header_list))+"\n"
		# Path(os.path.join(            root_dataset,train_test,option_string_dataset,inputs,options)).mkdir(parents=True, exist_ok=True)
		# path = os.path.join(root_dataset,train_test,option_string_dataset,inputs,options,'options_drive.txt')
		# file = Path(path)
		# if file.exists():
		# 	f = open(file, 'a')
		# 	f.write(filename+","+option_string_dataset+"\n")
		# 	f.close()
		# else:
		# 	f = open(file, "a")
		# 	f.write(header_str)
		# 	f.write(filename+","+option_string_dataset+"\n")
		# 	f.close()

		header_list=[]
		header_list.append('id')
		poses_pred_gt_to_txt_planelist=[]
		for i,pxy in enumerate(poses_pred_gt_to_txt):
			poses_pred_gt_to_txt_planelist.append(pxy[0])
			poses_pred_gt_to_txt_planelist.append(pxy[1])
			header_list.append('px'+str(i))
			header_list.append('py'+str(i))
		header_str=','.join(map(str, header_list))+"\n"

		Path(os.path.join(            root_dataset,train_test,option_string_dataset,gt,path_pred,path_pred_img)).mkdir(parents=True, exist_ok=True)
		cv2.imwrite(os.path.join(     root_dataset,train_test,option_string_dataset,gt,path_pred,path_pred_img,filename+'.png'), path_pred_png)
		poses_pred_gt_to_txt_str=','.join(map(str, poses_pred_gt_to_txt_planelist))+"\n"
		path = os.path.join(root_dataset,train_test,option_string_dataset,gt,path_pred,'path_pred.txt')

		if len(poses_pred_gt_to_txt_planelist)== 400:

			file = Path(path)
			if file.exists():
				#print(path+" exists") 
				f = open(file, 'a')
				f.write(filename+","+poses_pred_gt_to_txt_str)
				f.close()
			else:
				#print(path+"does not exist") 
				f = open(file, "a")
				f.write(header_str)
				f.write(filename+","+poses_pred_gt_to_txt_str)
				f.close()
			# Path(os.path.join(            root_dataset,train_test,gt,vehicle_state.txt)).mkdir(parents=True, exist_ok=True)
			# cv2.imwrite(os.path.join(     root_dataset,train_test,gt,vehicle_state_txt,filename+'.png'), stereo_bev_image)
			# string_vh_state=filename+str(self.steering_angle)+str(self.speed)+str(self.throttle)+str(self.brake)+str(self.hand_brake)
			string_vh_state=[filename,",",self.steering_angle,",",self.speed,",",self.throttle,",",self.brake,",",self.hand_brake,"\n"]

			path = os.path.join(root_dataset,train_test,option_string_dataset,gt,'vehicle_state.txt')
			file = Path(path)
			if file.exists():
				#print(path+" exists") 
				f = open(file, 'a')
				for element in string_vh_state:
					f.write(str(element))

				f.close()
			else:
				#print(path+"does not exist") 
				f = open(file, "a")
				f.write("file,steering_angle,speed,throttle,brake,hand_brake\n")
				for element in string_vh_state:
					f.write(str(element))
				f.close()
		# else:
		# 	sp = SpeedConstraint()
		# 	sp.header.stamp = rospy.Time().now()
		# 	# sp.speed=8.8
		# 	#sp.speed=3.0
		# 	# sp.speed=2.5
		# 	# sp.speed=1.8
		# 	sp.speed=1.0

		# 	sp.reason="near to intersection"
		# 	self.speed_constraint_pub.publish(sp)
		# 	# print ("near to intersection")
		# 	# pass
		# 	# print('len waypoints gt: ',len(poses_pred_gt_to_txt_planelist))
		# 	# print(path)

	def publish_plan_img(self, current_pose, global_plan):
		print('publish_plan_img: ')

		scale=8
		size_image=700

		try:
			trans_stamp_inv= self.tf2_buffer_map2vel.lookup_transform('velodyne','map', rospy.Time())
		except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as exept:
			print(exept)

		stamp = rospy.Time().now()
		try:
			trans_stamp_inv_stereo_to_map = self.tf2_buffer_map2stereo.lookup_transform('stereo','map', rospy.Time())
		except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as exept:
			print(exept)


		if self.rgb_image is None:
			return

		global_plan_points_png= np.zeros([size_image,size_image,3],dtype=np.uint8)
		global_plan_line_png= np.zeros([size_image,size_image,1],dtype=np.uint8)


		front_global_plan_points_png= np.zeros([self.rgb_image.shape[0], self.rgb_image.shape[1], 3], dtype=np.uint8)
		front_global_plan_points_png_depth= np.zeros([ self.rgb_image.shape[0], self.rgb_image.shape[1], 1], dtype=np.uint16)
		front_global_plan_line_png= np.zeros([self.rgb_image.shape[0], self.rgb_image.shape[1], 1], dtype=np.uint8)


		raw_path_points=[]
		color_plan=[]
		raw_points_uv=[]
		# print('len global plan ',len(global_plan.points))
		# print(global_plan.points)
		# if self.visited_points_global_plan ==[]
		# else:
		# 	self.visited_points_global_plan.append[next_index_goal_plan]

		for index, (pose, option) in enumerate(zip(global_plan.points, global_plan.road_options)):

			old_pose = PoseStamped()
			old_pose.header.frame_id='map'
			old_pose.header.stamp = stamp
			old_pose.pose.position = pose

			new_pose = tf2_geometry_msgs.do_transform_pose(old_pose, trans_stamp_inv)
			raw_path_points.append([new_pose.pose.position.y*scale+(size_image/2), new_pose.pose.position.x*scale+(size_image/3)])

			new_pose_map_to_stereo = tf2_geometry_msgs.do_transform_pose(old_pose, trans_stamp_inv_stereo_to_map)
			# print('new_pose_map_to_stereo ',new_pose_map_to_stereo)
			camera = PinholeCameraModel()
			camera.fromCameraInfo(self.camera_info_left)
			p_image = camera.project3dToPixel((new_pose_map_to_stereo.pose.position.x,  new_pose_map_to_stereo.pose.position.y,  new_pose_map_to_stereo.pose.position.z))
			# print(p_image)
			projected=False
			if new_pose_map_to_stereo.pose.position.z>0:
				projected=True
			raw_points_uv.append((p_image, projected, new_pose_map_to_stereo.pose.position.z))

			# marker = Marker()
			# marker.header.frame_id = "velodyne"
			# marker.type = marker.SPHERE
			# marker.action = marker.ADD
			# marker.ns = "my_namespace";

			# # marker scale
			# marker.scale.x = 0.3
			# marker.scale.y = 0.3
			# marker.scale.z = 0.3

			# marker color
			r=0.0
			g=0.0
			b=0.0
			if self.LANEFOLLOW==option:
				self.option_string='lanefollow'  ###green
				g=1.0
			if self.LEFT==option:				# red
				self.option_string='turnleft'
				r=1.0
			if self.RIGHT==option:				#blue
				self.option_string='turnright'
				b=1.0
			if self.STRAIGHT==option:			#white
				self.option_string='straight'
				r=1.0
				g=1.0
				b=1.0
			if self.CHANGELANELEFT==option:			#yellow
				self.option_string='changelaneleft'
				r=1.0
				g=1.0
			if self.CHANGELANERIGHT==option:		#cyan
				self.option_string='changelaneright'
				b=1.0
				g=1.0
			if self.UNKNOWN==option:
				self.option_string='lanefollow'
				r=1.0
				g=1.0
				b=1.0

			color_plan.append((b*255,g*255,r*255))

			# marker.color.a = 1.0
			# marker.color.r = r
			# marker.color.g = g
			# marker.color.b = b

			# # marker orientaiton
			# marker.pose.orientation.x = 0.0
			# marker.pose.orientation.y = 0.0
			# marker.pose.orientation.z = 0.0
			# marker.pose.orientation.w = 1.0

			# # marker position
			# marker.pose.position.x = new_pose.pose.position.x
			# marker.pose.position.y = new_pose.pose.position.y
			# marker.pose.position.z = 0

			# t = rospy.Duration(5.0) 
			# marker.lifetime = t
			# markerArray.markers.append(marker)

			thickness=-1
			# birdeyeview_stereo=cv2.circle(birdeyeview_stereo, (int(new_pose.pose.position.y*scale+(size_image/2)), 
																# int(new_pose.pose.position.x*scale+(size_image/3))), 10, (b*255,g*255,r*255), thickness)
			# plan_local
			# cv2.circle(global_plan_points_png, (int(new_pose.pose.position.y*scale+(size_image/2)), 
			# 													int(new_pose.pose.position.x*scale+(size_image/3))), 10, (b*255,g*255,r*255), thickness)
			#if index==self.next_index_goal_plan:          
			dist_to_goal =np.linalg.norm(np.array([pose.x, pose.y]) - 
														np.array([current_pose.pose.pose.position.x, current_pose.pose.pose.position.y]))#dist between ego and the next goal
			if dist_to_goal<7.:  ## if the ego achieve the goal
				self.next_index_goal_plan=index#self.next_index_goal_plan+1  #next goal
				# self.option_string_dataset=self.option_string
				# dist_to_near_option=dist_to_goal
				# self.pose_achieved_x=pose.x
				# self.pose_achieved_y=pose.y



		# if (self.option_string_dataset=='changelaneleft' or self.option_string_dataset=='changelaneright') :
		# 	# if self.pose_achieved_y is not None:
		# 	dist_to_pose_achieved =np.linalg.norm(np.array([self.pose_achieved_x, self.pose_achieved_y]) - 
		# 												np.array([current_pose.pose.pose.position.x, current_pose.pose.pose.position.y]))
		# 	if dist_to_pose_achieved>7.:
		# 		self.option_string_dataset='lanefollow'

		# print('self.option_string_dataset: ',self.option_string_dataset)
		# self.pub_option_lane.publish(self.option_string_dataset)
		# print('next_index_goal_plan: ', self.next_index_goal_plan, global_plan.road_options[self.next_index_goal_plan])
		# global_plan.road_options

			# print('saved lanefollow')
		init_interval=self.next_index_goal_plan-5
		end_interval=self.next_index_goal_plan+5

		if init_interval<3:
			init_interval=0
		if end_interval>len(raw_path_points):
			end_interval=len(raw_path_points)

		raw_local_points=[]
		for i in range(init_interval, end_interval):
			raw_local_points.append(raw_path_points[i])
			color_point=color_plan[i]
			point_local=raw_path_points[i]
			# plan_local
			# cv2.circle(global_plan_points_png, (int(pose.pose.position.y*scale+(size_image/2)), 
			# 													int(pose.pose.position.x*scale+(size_image/3))), 10, color, thickness)
			# cv2.circle(birdeyeview_stereo, (int(point_local[0]), int(point_local[1])), 10, color_point, thickness)
			cv2.circle(global_plan_points_png, (int(point_local[0]), int(point_local[1])), 10, color_point, thickness)

		init_interval=self.next_index_goal_plan-2
		end_interval=self.next_index_goal_plan+3

		if init_interval<0:
			init_interval=0
		if end_interval>len(raw_path_points):
			end_interval=len(raw_path_points)


		raw_local_points_uv=[]
		raw_local_points_uv.append((int(self.rgb_image.shape[1]/2), self.rgb_image.shape[0]))
		for i in range(init_interval, end_interval):
			# print('projected ',raw_points_uv[i][1])

			if raw_points_uv[i][1]: #points cam ahead true or false
				u = raw_points_uv[i][0][0]
				v = raw_points_uv[i][0][1]

				if raw_points_uv[i][0][1]>0  and  raw_points_uv[i][0][1]<self.rgb_image.shape[0]:#  and raw_points_uv[i][0][0]>0  and  raw_points_uv[i][0][0]<1600 :
					# print(raw_points_uv[i][0][0])
					if raw_points_uv[i][0][0]<0:
						u=0

					if raw_points_uv[i][0][0]>self.rgb_image.shape[1]:
						u=self.rgb_image.shape[1]

					raw_local_points_uv.append((u,v))

					color_point=color_plan[i]
					# point_uv_local=()raw_points_uv[i][0]
					# print('point_uv_local ',point_uv_local)
					# plan_local
					# cv2.circle(global_plan_points_png, (int(pose.pose.position.y*scale+(size_image/2)), 
					# 													int(pose.pose.position.x*scale+(size_image/3))), 10, color, thickness)
					if u>0 and u<self.rgb_image.shape[1]  and raw_points_uv[i][2]>0:
						# cv2.circle(rgb_image, (int(u), int(v)), 15, color_point, thickness)
						cv2.circle(front_global_plan_points_png, (int(u), int(v)), int(500/raw_points_uv[i][2]), color_point, thickness)
						cv2.circle(front_global_plan_points_png_depth, (int(u), int(v)), int(500/raw_points_uv[i][2]), (int(1000*raw_points_uv[i][2])), thickness)
						
						# cv2.circle(global_plan_points_png, (int(point_local[0]), int(point_local[1])), 10, color_point, thickness)
		# im_color_depth = cv2.applyColorMap(front_global_plan_points_png_depth, cv2.COLORMAP_JET)
		# im_color_depth = cv2.bitwise_and(im_color_depth, im_color_depth, mask=front_global_plan_points_png_depth)
		# radius=3
		# color =(0,255,0)


		thickness=10

		# pts = np.array(raw_path_points, np.int32)
		# pts = pts.reshape((-1,1,2))
		# # cv2.polylines(points_traj_past,[pts],False,(255,0,0))			
		# # cv2.polylines(birdeyeview_stereo,[pts],False,(0,255,0),thickness)	
		# cv2.polylines(global_plan_line_png,[pts],False,(255),thickness)	

		pts = np.array(raw_local_points, np.int32)
		pts = pts.reshape((-1,1,2))
		# cv2.polylines(points_traj_past,[pts],False,(255,0,0))			
		# cv2.polylines(birdeyeview_stereo,[pts],False,(0,255,0),thickness)	
		cv2.polylines(global_plan_line_png,[pts],False,(255),thickness)	





		# print(raw_local_points_uv)
		raw_local_points_uv_rev=raw_local_points_uv[::-1]
		# print(raw_local_points_uv_rev)
		pts = np.array(raw_local_points_uv_rev, np.int32)
		# print('pts ',pts)

		pts = pts.reshape((-1,1,2))
		# print(pts)
		# cv2.polylines(points_traj_past,[pts],False,(255,0,0))			
		# cv2.polylines(birdeyeview_stereo,[pts],False,(0,255,0),thickness)	
		# cv2.polylines(rgb_image,[pts],False,(0,255,0),thickness)	
		cv2.polylines(front_global_plan_line_png,[pts],False,(255),thickness)	







		scale=8
		size_image=700




		# print('last tr poses ')

		try:
			trans_stamp_inv= self.tf2_buffer_map2vel.lookup_transform('velodyne','map', rospy.Time())
		except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as exept:
			print(exept)

		# def pose_gps_cb(self,msg):
		current_pose_gps = self.current_pose
		# current_pose = self.pose_msg

		if self.old_poses_list_gps==None:
			self.old_poses_list_gps = [[current_pose_gps,0] for i in range(200)]
		else:

			dist=np.linalg.norm(np.array([current_pose.pose.pose.position.x,current_pose.pose.pose.position.y]) - np.array([ self.old_poses_list_gps[0][0].pose.pose.position.x, self.old_poses_list_gps[0][0].pose.pose.position.y]))

			if dist>0.2:

				deque_old_poses_list_gps = deque(self.old_poses_list_gps) 
				deque_old_poses_list_gps.rotate(1) 
				self.old_poses_list_gps = list(deque_old_poses_list_gps)
				self.old_poses_list_gps[0] = [current_pose_gps,dist]


		gps_backward_png= np.zeros([size_image,size_image,1],dtype=np.uint8)
		img_map_mask= np.zeros([size_image,size_image,1],dtype=np.uint8)

		poses_back=[]

		for p, d in self.old_poses_list_gps:

			new_tfmed_gps = tf2_geometry_msgs.do_transform_pose(p.pose, trans_stamp_inv)
			poses_back.append( [(new_tfmed_gps.pose.position.y*scale)+(size_image/2), (new_tfmed_gps.pose.position.x*scale)+(size_image/3)] )
			x = (new_tfmed_gps.pose.position.y*scale)+(size_image/2)
			y = (new_tfmed_gps.pose.position.x*scale)+(size_image/3)

			x0=(size_image/2)
			y0=(size_image/3)


			dist_to_p=np.linalg.norm(np.array([x,y]) - np.array([x0,y0]))


			# print(x,y,dist_to_p)
			cv2.circle(gps_backward_png, (int(x), int(y)), 10, int(255-dist_to_p), thickness=-1)
			cv2.circle(img_map_mask, (int(x), int(y)), 10, 255, thickness=-1)


		ret,img_map_mask = cv2.threshold(img_map_mask,1,255,cv2.THRESH_BINARY)
		gps_backward_color_png=cv2.applyColorMap(gps_backward_png, cv2.COLORMAP_RAINBOW)
		gps_backward_color_png = cv2.bitwise_and(gps_backward_color_png,gps_backward_color_png, mask= img_map_mask)


		# pts = np.array(poses_back, np.int32)
		# pts = pts.reshape((-1,1,2))
		# cv2.polylines(points_traj_past,[pts],False,(0,255,255))
		# cv2.polylines(birdeyeview_stereo,[pts],False,(0,255,255))
		# cv2.polylines(gps_backward_png,[pts],False,(255),thickness)

		print('last 100 poses ')
		if self.pub_gps_backward_color_png.get_num_connections() != 0:
			try:
				self.pub_gps_backward_color_png.publish(self.cvbridge.cv2_to_imgmsg(gps_backward_color_png, "bgr8"))
			except CvBridgeError as e:
				print (e)

		if self.pub_gps_backward_png.get_num_connections() != 0:
			try:
				self.pub_gps_backward_png.publish(self.cvbridge.cv2_to_imgmsg(gps_backward_png, "mono8"))
			except CvBridgeError as e:
				print (e)

		if self.pub_global_plan_img.get_num_connections() != 0:
			try:
				self.pub_global_plan_img.publish(self.cvbridge.cv2_to_imgmsg(global_plan_points_png, "bgr8"))
			except CvBridgeError as e:
				print (e)

		if self.pub_global_plan_line_img.get_num_connections() != 0:
			try:
				self.pub_global_plan_line_img.publish(self.cvbridge.cv2_to_imgmsg(global_plan_line_png, "mono8"))
			except CvBridgeError as e:
				print (e)
		if self.pub_front_global_plan_img.get_num_connections() != 0:
			try:
				self.pub_front_global_plan_img.publish(self.cvbridge.cv2_to_imgmsg(front_global_plan_points_png, "bgr8"))
			except CvBridgeError as e:
				print (e)

		if self.pub_front_global_plan_line_img.get_num_connections() != 0:
			try:
				self.pub_front_global_plan_line_img.publish(self.cvbridge.cv2_to_imgmsg(front_global_plan_line_png, "mono8"))
			except CvBridgeError as e:
				print (e)

		if self.pub_front_global_plan_depth.get_num_connections() != 0:
			try:
				self.pub_front_global_plan_depth.publish(self.cvbridge.cv2_to_imgmsg(front_global_plan_points_png_depth, "mono16"))
			except CvBridgeError as e:
				print (e)

		return gps_backward_color_png, gps_backward_png, global_plan_points_png, global_plan_line_png, front_global_plan_points_png, front_global_plan_line_png, front_global_plan_points_png_depth




	def shutdown_cb(self, msg):
		if msg.data:
			self.path = np.array([])
			# self.last_path_stamp = 0.0

			print ("Bye!")
			# rospy.signal_shutdown("finished route")
			self.n_folder=self.n_folder+1
			self.string_folder='{:03d}'.format(self.n_folder)

if __name__ == '__main__':
	rospy.init_node("create_dataset", anonymous=True)
	print ("[Create dataset] running...")
	dataset=CreateDatasetCarla()
	rospy.spin()



