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


class CreateVideosCarla(object):
	def __init__ (self):

		self.root_dataset=rospy.get_param("/dataset/root_folder")
		# self.train_test=rospy.get_param("/dataset/train_test_folder")

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
		# self.rgb_right_image=None
		self.depth_image =None
		self.rgb_cam_info=None
		self.stereo_bev_image =None
		self.lidar_bev_image =None
		self.seg_image=None
		self.ins_image=None
		self.aerial_image=None
		self.aerialseg_image=None
		self.img_map_foot_print_dir=None
		self.rviz_img=None
		self.spectator=None

		self.instance_mask=None

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

		self.tf2_buffer_vel2map = tf2_ros.Buffer()
		self.tf2_listener_vel2map = tf2_ros.TransformListener(self.tf2_buffer_vel2map)

		self.tf2_buffer_map2vel = tf2_ros.Buffer()
		self.tf2_listener_map2vel = tf2_ros.TransformListener(self.tf2_buffer_map2vel)


		#subscriber
		self.rviz_sub = rospy.Subscriber('/rviz/image', Image, self.rviz_cb, queue_size=1)


		self.pub_option_lane_sub = rospy.Subscriber('/carina/map/option_lane', String, self.option_cmd_cb, queue_size=1)

		self.spectator = rospy.Subscriber('/carla/hero/CameraSpectator/image', Image, self.spectator_cb, queue_size=1)

		self.instance_mask_sub = rospy.Subscriber('/carina/perception/camera/image_bb', Image, self.instance_mask_cb, queue_size=1)

		# self.pub_option_lane = rospy.Publisher('/carina/map/option_lane', String, queue_size=1)

		self.global_plan_img_sub = rospy.Subscriber('/carina/map/global/global_plan_img', Image, self.global_plan_img_cb, queue_size=1)
		self.global_plan_line_img_sub = rospy.Subscriber('/carina/map/global/global_plan_line_img', Image, self.global_plan_line_img_cb, queue_size=1)
		self.front_global_plan_img_sub = rospy.Subscriber('/carina/map/global/front_global_plan_img', Image, self.front_global_plan_img_cb, queue_size=1)
		self.front_global_plan_line_img_sub = rospy.Subscriber('/carina/map/global/front_global_plan_line_img', Image, self.front_global_plan_line_img_cb, queue_size=1)

		self.front_global_plan_depth_sub = rospy.Subscriber('/carina/map/global/front_global_plan_depth_img', Image, self.front_global_plan_depth_cb, queue_size=1)

		self.gps_backward_png_sub = rospy.Subscriber('/carina/map/global/pub_gps_backward_png_img', Image, self.gps_backward_png_cb, queue_size=1)
		self.gps_backward_png_color_sub = rospy.Subscriber('/carina/map/global/pub_gps_backward_color_png_img', Image, self.gps_backward_png_color_cb, queue_size=1)



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
		# self.lidar_sub = rospy.Subscriber("/carina/sensor/lidar/front/point_cloud", PointCloud2, self.lidar_point_cloud_cb)
		# self.stereo_pc_sub = rospy.Subscriber("/carina/perception/stereo/point_cloud", PointCloud2, self.stereo_point_cloud_cb)
		self.image_rgb_sub = rospy.Subscriber('/carina/sensor/camera/left/image_raw', Image, self.rgb_imageCallback, queue_size=1)
		# self.img_left_info_sub = rospy.Subscriber('/carina/sensor/camera/left/camera_info', CameraInfo, self.rgb_camera_infoCallback, queue_size=1)
		# self.image_rgb_right_sub = rospy.Subscriber('/carina/sensor/camera/right/image_raw', Image, self.rgb_right_imageCallback, queue_size=1)
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
		# self.global_plan_raw_sub = rospy.Subscriber('/carina/navigation/global_plan_raw', GlobalPlan, self.global_plan_cb, queue_size=1)
		self.shutdown_sub = rospy.Subscriber('/carina/vehicle/shutdown', Bool, self.shutdown_cb, queue_size=1)
		self.control_carla_sub = rospy.Subscriber('/carla/hero/vehicle_control_cmd',CarlaEgoVehicleControl, self.control_carla_cb, queue_size=1)



	def normalize_angle(self, angle):
		return math.atan2(np.sin(angle), np.cos(angle))




	def instance_mask_cb(self,msg):
		try:
			self.instance_mask = self.cvbridge.imgmsg_to_cv2(msg, "bgr8")
		except CvBridgeError as e:
			print (e)

	def spectator_cb(self,msg):
		try:
			self.spectator = self.cvbridge.imgmsg_to_cv2(msg, "bgr8")
		except CvBridgeError as e:
			print (e)

	def rviz_cb(self,msg):
		try:
			self.rviz_img = self.cvbridge.imgmsg_to_cv2(msg, "bgr8")
		except CvBridgeError as e:
			print (e)
		



	def global_plan_img_cb(self,msg):
		try:
			self.global_plan_points_png = self.cvbridge.imgmsg_to_cv2(msg, "bgr8")
		except CvBridgeError as e:
			print (e)
		

	def global_plan_line_img_cb(self,msg):
		try:
			self.global_plan_line_png = self.cvbridge.imgmsg_to_cv2(msg, "mono8")
		except CvBridgeError as e:
			print (e)		

	def front_global_plan_img_cb(self,msg):
		try:
			self.front_global_plan_points_png = self.cvbridge.imgmsg_to_cv2(msg, "bgr8")
		except CvBridgeError as e:
			print (e)



	def front_global_plan_line_img_cb(self,msg):
		try:
			self.front_global_plan_line_png = self.cvbridge.imgmsg_to_cv2(msg, "mono8")
		except CvBridgeError as e:
			print (e)

	def gps_backward_png_color_cb(self,msg):
		try:
			self.gps_backward_png_color = self.cvbridge.imgmsg_to_cv2(msg, "bgr8")
			# print(type(self.gps_backward_png[0][0]))
		except CvBridgeError as e:
			print (e)


	def gps_backward_png_cb(self,msg):
		try:
			self.gps_backward_png = self.cvbridge.imgmsg_to_cv2(msg, "mono8")
			# print(type(self.gps_backward_png[0][0]))
		except CvBridgeError as e:
			print (e)



	def front_global_plan_depth_cb(self,msg):
		try:
			self.front_global_plan_points_png_depth = self.cvbridge.imgmsg_to_cv2(msg, "mono16")
			# print(type(self.front_global_plan_points_png_depth[0][0]))
		except CvBridgeError as e:
			print (e)



	def option_cmd_cb(self,msg):
		self.option_string_dataset=msg.data
		

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


	# def rgb_right_imageCallback(self,im):
	# 	try:
	# 		self.rgb_right_image = self.cvbridge.imgmsg_to_cv2(im, "bgr8")
	# 	except CvBridgeError as e:
	# 		print (e)

	def depth_imageCallback(self,im):
		try:
			self.depth_image = self.cvbridge.imgmsg_to_cv2(im, "mono16")#mono16: CV_16UC1, 16-bit grayscale image 
		except CvBridgeError as e:
			print (e)


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
	
			b=127.5
			m = (0-255)/(-math.pi- math.pi)
			x=self.normalize_angle(yaw)
			# print(x)
			dir_value_pixel=m*x+b

			cv2.circle(img_map,(px, py), 2, 255, -1)
			cv2.circle(img_map_dir,(px, py), 2, int(dir_value_pixel), -1)

			obstacles_frame.append(world_obs)
			# print('past frames',len(obstacles_frame))
		self.obstacles_frame_0=copy.deepcopy(obstacles_frame)
		px = int(size_image/2)
		py = int(size_image/3)
		cv2.circle(img_map,(px, py), 5, 255, -1)
		cv2.circle(img_map_dir,(px, py), 5, 127, -1)

		img_map_mask=img_map
		ret,img_map_mask = cv2.threshold(img_map_mask,1,255,cv2.THRESH_BINARY)
		img_map_dir_color=cv2.applyColorMap(img_map_dir, cv2.COLORMAP_HSV)
		img_map_dir_color = cv2.bitwise_and(img_map_dir_color,img_map_dir_color, mask= img_map_mask)

		self.img_map_foot_print_dir=img_map_dir_color

		if len(self.obstacles_frame_0) > 2000:
			self.obstacles_frame_0=self.obstacles_frame_0[:2000]


	def path_callback(self, msg):
		self.path=msg
		# self.save_frame_dataset(msg,self.option_string_dataset)


	def pose_gps_cb(self,msg):
		self.current_pose_gps = msg
		self.current_pose = msg

		print('saving frames')
		self.save_frame_dataset(self.path,self.option_string_dataset)


	def save_frame_dataset(self, path, option_string_dataset):


		if self.current_pose is None or self.rgb_image is None or self.stereo_bev_image is None or self.lidar_bev_image is None \
			or self.aerial_image is None or self.aerialseg_image is None  or self.seg_image is None \
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
		# rgb_right_image=copy.deepcopy(self.rgb_right_image)
		depth_image=copy.deepcopy(self.depth_image)

		seg_image=copy.deepcopy(self.seg_image)
		ins_image=copy.deepcopy(self.ins_image)
		aerial_image=copy.deepcopy(self.aerial_image)
		occupancy_image=copy.deepcopy(self.occupancy_image)
		aerialseg_image=copy.deepcopy(self.aerialseg_image)
		lidar_bev_image=copy.deepcopy(self.lidar_bev_image)
		stereo_bev_image=copy.deepcopy(self.stereo_bev_image)
		img_map_foot_print_dir=copy.deepcopy(self.img_map_foot_print_dir)


		global_plan_points_png= copy.deepcopy(self.global_plan_points_png)
		global_plan_line_png= copy.deepcopy(self.global_plan_line_png)
		gps_backward_png= copy.deepcopy(self.gps_backward_png)
		gps_backward_png_color= copy.deepcopy(self.gps_backward_png_color)


		front_global_plan_points_png= copy.deepcopy(self.front_global_plan_points_png)
		front_global_plan_points_png_depth=copy.deepcopy(self.front_global_plan_points_png_depth)
		# front_global_plan_points_png_depth= numpy.zeros([size_img_front,size_img_front,1],dtype=np.uint16)
		front_global_plan_line_png= copy.deepcopy(self.front_global_plan_line_png)
		rviz_img= copy.deepcopy(self.rviz_img)
		spectator= copy.deepcopy(self.spectator)
		instance_mask= copy.deepcopy(self.instance_mask)

		try:
			trans_stamp_inv= self.tf2_buffer_map2vel.lookup_transform('velodyne','map', rospy.Time())

		except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as exept:
			print(exept)
			return

		im_color_lidar = cv2.applyColorMap(lidar_bev_image, cv2.COLORMAP_JET)
		im_color_lidar = cv2.bitwise_and(im_color_lidar, im_color_lidar, mask=lidar_bev_image)


		root_dataset=self.root_dataset #"/mnt/Data1/carla_dataset"


		rgb_img='rgb_left'
		# rgb_right_img='rgb_right'

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

		depth_color_img='points_depth_color_front'

		global_plan_imgs='global_plan'
		global_plan_points_img='global_plan_bev'
		global_plan_line_img='global_line_bev'

		front_global_plan_imgs='front_global_plan'
		front_global_plan_points_img='front_global_points'
		front_global_plan_points_img_depth='front_global_points_depth'

		front_global_plan_line_img='front_global_line'

		gps_backward='past_gps_backward'
		# gps_backward_txt='points'
		gps_backward_img='past_line'
		gps_backward_color_img='past_line_color'

		options='options'

		path_pred='path_pred'
		# path_pred_txt='txt_line'
		path_pred_img='path_pred_line_img'
		vehicle_state_txt='vehicle_state'
		rviz='rviz'
		spec='spectator'
		inst_mask='instance_mask'

		filename=str(time.time_ns())

		# classes_list=[]

		# for obs in self.obstacles_dataset_gt:
		# 	for c in obs.classes:
		# 		classes_list.append(c)



		# print(self.string_folder)
		# Path(os.path.join(folder_video,rgb_img,self.string_folder)).mkdir(parents=True, exist_ok=True)
		# cv2.imwrite(os.path.join(folder_video,rgb_img,self.string_folder,filename+'.jpg'), rgb_image)
		folder_video=os.path.join(root_dataset,'images_video')

		print('saving to ',os.path.join(folder_video))

		Path(os.path.join(folder_video,inst_mask)).mkdir(parents=True, exist_ok=True)
		cv2.imwrite(os.path.join(folder_video,inst_mask,filename+'.jpg'), instance_mask)

		Path(os.path.join(folder_video,spec)).mkdir(parents=True, exist_ok=True)
		cv2.imwrite(os.path.join(folder_video,spec,filename+'.jpg'), spectator)

	
		Path(os.path.join(folder_video,rviz)).mkdir(parents=True, exist_ok=True)
		cv2.imwrite(os.path.join(folder_video,rviz,filename+'.jpg'), rviz_img)



		Path(os.path.join(folder_video,rgb_img)).mkdir(parents=True, exist_ok=True)
		cv2.imwrite(os.path.join(folder_video,rgb_img,filename+'.jpg'), rgb_image)

		# Path(os.path.join(folder_video,rgb_right_img,self.string_folder)).mkdir(parents=True, exist_ok=True)
		# cv2.imwrite(os.path.join(folder_video,rgb_right_img,self.string_folder,filename+'.jpg'), rgb_right_image)
		
		# Path(os.path.join(folder_video,rgb_right_img)).mkdir(parents=True, exist_ok=True)
		# cv2.imwrite(os.path.join(folder_video,rgb_right_img,filename+'.jpg'), rgb_right_image)

		# Path(os.path.join(root_dataset,train_test,inputs,depth_img)).mkdir(parents=True, exist_ok=True)
		# cv2.imwrite(os.path.join(root_dataset,train_test,inputs,depth_img,filename+'.png'), depth_image)

		# Path(os.path.join(folder_video,seg_img)).mkdir(parents=True, exist_ok=True)
		# cv2.imwrite(os.path.join(folder_video,seg_img,filename+'.jpg'), seg_image)

		# Path(os.path.join(folder_video,ins_img)).mkdir(parents=True, exist_ok=True)
		# cv2.imwrite(os.path.join(folder_video,ins_img,filename+'.jpg'), ins_image)

		aerial_image=cv2.flip(aerial_image,-1)
		Path(os.path.join(folder_video, aerial_rgb_img)).mkdir(parents=True, exist_ok=True)
		cv2.imwrite(os.path.join(folder_video,aerial_rgb_img,filename+'.jpg'), aerial_image)

		Path(os.path.join(folder_video, occupancy_img)).mkdir(parents=True, exist_ok=True)
		cv2.imwrite(os.path.join(folder_video,occupancy_img,filename+'.jpg'), occupancy_image)

		# aerialseg_image=cv2.flip(aerialseg_image,-1)
		# Path(os.path.join(folder_video, aerial_seg_img)).mkdir(parents=True, exist_ok=True)
		# cv2.imwrite(os.path.join(folder_video,aerial_seg_img,filename+'.jpg'), aerialseg_image)


		Path(os.path.join(folder_video,lidar_bev_color_img)).mkdir(parents=True, exist_ok=True)
		cv2.imwrite(os.path.join(folder_video,lidar_bev_color_img,filename+'.jpg'), im_color_lidar)

		Path(os.path.join(folder_video,stereo_bev_img)).mkdir(parents=True, exist_ok=True)
		cv2.imwrite(os.path.join(folder_video,stereo_bev_img,filename+'.jpg'), stereo_bev_image)

		Path(os.path.join(folder_video,lidar_bev_img)).mkdir(parents=True, exist_ok=True)
		cv2.imwrite(os.path.join(folder_video,lidar_bev_img,filename+'.jpg'), lidar_bev_image)

		Path(os.path.join(folder_video,img_map_foot_print)).mkdir(parents=True, exist_ok=True)
		cv2.imwrite(os.path.join(folder_video,img_map_foot_print,filename+'.jpg'), img_map_foot_print_dir)


		# Path(os.path.join(root_dataset,train_test,inputs,front_global_plan_imgs,depth_color_img)).mkdir(parents=True, exist_ok=True)
		# cv2.imwrite(os.path.join(root_dataset,train_test,inputs,front_global_plan_imgs,depth_color_img,filename+'.png'), im_color_depth)

		Path(os.path.join(            folder_video,global_plan_points_img)).mkdir(parents=True, exist_ok=True)
		cv2.imwrite(os.path.join(     folder_video,global_plan_points_img,filename+'.png'), global_plan_points_png)

		Path(os.path.join(            folder_video,global_plan_line_img)).mkdir(parents=True, exist_ok=True)
		cv2.imwrite(os.path.join(     folder_video,global_plan_line_img,filename+'.png'), global_plan_line_png)

		# Path(os.path.join(            folder_video,front_global_plan_imgs,front_global_plan_points_img)).mkdir(parents=True, exist_ok=True)
		# cv2.imwrite(os.path.join(     folder_video,front_global_plan_imgs,front_global_plan_points_img,filename+'.png'), front_global_plan_points_png)

		# Path(os.path.join(            folder_video,front_global_plan_imgs,front_global_plan_line_img)).mkdir(parents=True, exist_ok=True)
		# cv2.imwrite(os.path.join(     folder_video,front_global_plan_imgs,front_global_plan_line_img,filename+'.png'), front_global_plan_line_png)

		# Path(os.path.join(            folder_video,front_global_plan_imgs,front_global_plan_points_img_depth)).mkdir(parents=True, exist_ok=True)
		# cv2.imwrite(os.path.join(     folder_video,front_global_plan_imgs,front_global_plan_points_img_depth,filename+'.png'), front_global_plan_points_png_depth)


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


		Path(os.path.join(            folder_video,gps_backward_img)).mkdir(parents=True, exist_ok=True)
		cv2.imwrite(os.path.join(     folder_video,gps_backward_img,filename+'.png'), gps_backward_png)

		Path(os.path.join(            folder_video,gps_backward_color_img)).mkdir(parents=True, exist_ok=True)
		cv2.imwrite(os.path.join(     folder_video,gps_backward_color_img,filename+'.png'), gps_backward_png_color)		# path = os.path.join(folder_video,gps_backward,'gps_backward.txt')
		


		header_list=[]
		header_list.append('id')
		poses_pred_gt_to_txt_planelist=[]
		for i,pxy in enumerate(poses_pred_gt_to_txt):
			poses_pred_gt_to_txt_planelist.append(pxy[0])
			poses_pred_gt_to_txt_planelist.append(pxy[1])
			# header_list.append('px'+str(i))
			# header_list.append('py'+str(i))
		# header_str=','.join(map(str, header_list))+"\n"

		Path(os.path.join(            folder_video,path_pred_img)).mkdir(parents=True, exist_ok=True)
		cv2.imwrite(os.path.join(     folder_video,path_pred_img,filename+'.png'), path_pred_png)
		# poses_pred_gt_to_txt_str=','.join(map(str, poses_pred_gt_to_txt_planelist))+"\n"
		# path = os.path.join(folder_video,path_pred,'path_pred.txt')

		if len(poses_pred_gt_to_txt_planelist)== 400:

			string_vh_state=[filename,",",self.steering_angle,",",self.speed,",",self.throttle,",",self.brake,",",self.hand_brake,"\n"]

	
	def shutdown_cb(self, msg):
		if msg.data:
			self.path = np.array([])
			self.last_path_stamp = 0.0

			print ("Bye!")
			# rospy.signal_shutdown("finished route")
			self.n_folder=self.n_folder+1
			self.string_folder='{:03d}'.format(self.n_folder)

if __name__ == '__main__':
	rospy.init_node("create_images_video", anonymous=True)
	print ("[Create images] running...")
	dataset=CreateVideosCarla()
	rospy.spin()



