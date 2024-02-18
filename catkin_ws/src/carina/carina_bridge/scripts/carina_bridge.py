#!/usr/bin/env python3
import os
import numpy as np
from leaderboard.autoagents.autonomous_agent import  Track #AutonomousAgent,
from agents.navigation.local_planner import RoadOption
import rospy
import tf2_ros
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped, TransformStamped, Point, Vector3
from msgs_action.msg import VehicleState, Throttle, Brake, SteeringAngle
from nav_msgs.msg import Path, Odometry
from msgs_navigation.msg import GlobalPlan
from msgs_mapping.msg import HDMap as HDMapMsg
from msgs_perception.msg import ObstacleArray, Obstacle
from visualization_msgs.msg import MarkerArray, Marker
from sensor_msgs.msg import NavSatFix, Imu, Image, PointCloud2, CameraInfo
from sensor_msgs.point_cloud2 import create_cloud_xyz32, create_cloud, read_points, PointField
import xml.etree.ElementTree as ET
import math
from tf2_geometry_msgs import *
import tf
from std_msgs.msg import Float64, Header, Bool, String
from derived_object_msgs.msg import ObjectArray, Object
from carla_msgs.msg import CarlaEgoVehicleControl, CarlaSpeedometer, CarlaRoute, CarlaGnssRoute, CarlaTrafficLightStatusList
from carla_msgs.msg import CarlaTrafficLightInfoList, CarlaTrafficLightStatus, CarlaActorList, CarlaActorInfo
from carla_common.transforms import carla_location_to_ros_point, carla_rotation_to_ros_quaternion
from scipy import interpolate
from geometry_msgs.msg import Pose
from geometry_msgs.msg import PoseArray
from nav_msgs.msg import Path, Odometry, OccupancyGrid
from cv_bridge import CvBridge, CvBridgeError
import sys
# np.set_printoptions(threshold=sys.maxsize)
from enum import Enum
import time
from image_geometry import PinholeCameraModel
import random
import cv2
from cv_bridge import CvBridge, CvBridgeError
from collections import deque 

class CarinaBridge(object):

	def __init__ (self):

		print('Setup carla_bridge')
		########## stereo camera parameters ##########
		self.fov=rospy.get_param("/cameras/stereo/fov")#parameters["cameras"]["stereo"]["fov"]#122#cameras fov
		# self.fov=30.6             #cameras fov  argo
		self.baseline=rospy.get_param("/cameras/stereo/baseline")#parameters["cameras"]["stereo"]["baseline"]#0.24
		self.image_width=rospy.get_param("/cameras/stereo/width")#arameters["cameras"]["stereo"]["width"]#1600 
		self.image_height=rospy.get_param("/cameras/stereo/height")#parameters["cameras"]["stereo"]["height"]#1600
		# self.baseline=0.2986      #argo
		##############################################
		print(self.fov, self.baseline, self.image_width, self.image_height)

		self.create_dataset_path=rospy.get_param("/dataset/create_dataset_path")#parameters["dataset"]["create_dataset_path"]#False
		self.create_dataset_images=rospy.get_param("/dataset/create_dataset_images")#arameters["dataset"]["create_dataset_images"]#False
		self.create_dataset_depth=rospy.get_param("/dataset/create_dataset_depth")#parameters["dataset"]["create_dataset_depth"]#False
		self.create_occupancy_grid=rospy.get_param("/dataset/create_occupancy_grid")#parameters["dataset"]["create_occupancy_grid"]#False
		self.create_dataset_obstacles=rospy.get_param("/dataset/create_dataset_obstacles")#parameters["dataset"]["create_dataset_obstacles"]#False
		self.lateral_control_noise=rospy.get_param("/dataset/lateral_control_noise")

		print('Dataset: ')
		print('Path_dataset: ',self.create_dataset_path, 'Images:', self.create_dataset_images, 'Depth:', self.create_dataset_depth, \
			'Occup_grid:', self.create_occupancy_grid, 'Obstacles:', self.create_dataset_obstacles, 'Lat Control noise:', self.lateral_control_noise)

		track_env = rospy.get_param("/track_param")
		leaderboard= rospy.get_param("/leaderboard_param")

		if track_env=='SENSORS':
			self.track = Track.SENSORS
			self.lat_ref=0.0
			self.lon_ref=0.0
			self.datum=''
		elif track_env=='MAP':
			self.track = Track.MAP
			self.lat_ref=None
			self.lon_ref=None
			self.datum=None
		elif not (track_env == Track.MAP or track_env == Track.SENSORS):
			self.track = 'DATASET'
			self.lat_ref=0
			self.lon_ref=0
			self.datum=''

			# self.CREATE_DATASET=True  #---->

		print('Leaderboard Version:', leaderboard)
		print('CHALLENGE_TRACK_CODENAME: ', track_env)
		print('Configured Track:', self.track)

		self.vehicle_models_map={
			'vehicle.dodge.charger_2020':'CLASSIFICATION_CAR',
			'vehicle.dodge.charger_police_2020':'CLASSIFICATION_CAR',
			'vehicle.ford.crown':'CLASSIFICATION_CAR',
			'vehicle.lincoln.mkz_2020':'CLASSIFICATION_CAR',
			'vehicle.mercedes.coupe_2020':'CLASSIFICATION_CAR',
			'vehicle.mini.cooper_s_2021':'CLASSIFICATION_CAR',
			'vehicle.nissan.patrol_2021':'CLASSIFICATION_CAR',
			'vehicle.audi.a2':'CLASSIFICATION_CAR',
			'vehicle.audi.etron':'CLASSIFICATION_CAR',
			'vehicle.audi.tt':'CLASSIFICATION_CAR',
			'vehicle.bmw.grandtourer':'CLASSIFICATION_CAR',
			'vehicle.chevrolet.impala':'CLASSIFICATION_CAR',
			'vehicle.citroen.c3':'CLASSIFICATION_CAR',
			'vehicle.dodge.charger_police':'CLASSIFICATION_CAR',
			'vehicle.ford.mustang':'CLASSIFICATION_CAR',
			'vehicle.jeep.wrangler_rubicon':'CLASSIFICATION_CAR',
			'vehicle.lincoln.mkz_2017':'CLASSIFICATION_CAR',
			'vehicle.mercedes.coupe':'CLASSIFICATION_CAR',
			'vehicle.micro.microlino': 'CLASSIFICATION_CAR',
			'vehicle.mini.cooper_s' : 'CLASSIFICATION_CAR',
			'vehicle.nissan.micra': 'CLASSIFICATION_CAR',
			'vehicle.nissan.patrol' : 'CLASSIFICATION_CAR',
			'vehicle.seat.leon': 'CLASSIFICATION_CAR',
			'vehicle.tesla.model3' : 'CLASSIFICATION_CAR',
			'vehicle.toyota.prius' : 'CLASSIFICATION_CAR',
			
			'vehicle.carlamotors.european_hgv' : 'CLASSIFICATION_TRUCK',
			'vehicle.carlamotors.firetruck' : 'CLASSIFICATION_TRUCK',
			'vehicle.tesla.cybertruck' : 'CLASSIFICATION_TRUCK',
			'vehicle.carlamotors.carlacola' : 'CLASSIFICATION_TRUCK',
			'vehicle.mitsubishi.fusorosa' : 'CLASSIFICATION_TRUCK',

			'vehicle.ford.ambulance'  : 'CLASSIFICATION_VAN',
			'vehicle.mercedes.sprinter' : 'CLASSIFICATION_VAN',
			'vehicle.volkswagen.t2_2021' : 'CLASSIFICATION_VAN',
			'vehicle.volkswagen.t2' : 'CLASSIFICATION_VAN',
			
			'vehicle.harley-davidson.low_rider' : 'CLASSIFICATION_MOTORCYCLE',
			'vehicle.kawasaki.ninja' : 'CLASSIFICATION_MOTORCYCLE',
			'vehicle.yamaha.yzf' : 'CLASSIFICATION_MOTORCYCLE',
			'vehicle.vespa.zx125' : 'CLASSIFICATION_MOTORCYCLE',
			
			'vehicle.bh.crossbike' : 'CLASSIFICATION_BIKE',
			'vehicle.diamondback.century' : 'CLASSIFICATION_BIKE',
			'vehicle.gazelle.omafiets' : 'CLASSIFICATION_BIKE'
		}

		self.datum=None
		# self.speed_temp=0.0
		self.route = Path()
		self.global_plan_msg = GlobalPlan()
		self.publis_gt_waypoints = True
		self.throttle = 0.0
		self.brake = 0
		self.steering_angle = 0.0
		self.max_steering = np.deg2rad(30.)
		self.hand_brake = False
		self.pose = None
		self.last_pose = None
		self.origin = PoseStamped()
		self.origin.pose.position.x = 0
		self.origin.pose.position.y = 0
		self.origin.pose.position.z = 0
		self.origin_yaw = 0.0
		self.origin_gt = PoseStamped()
		self.origin_gt.pose.position.x = 0
		self.origin_gt.pose.position.y = 0
		self.origin_gt.pose.position.z = 0
		self.origin_gt_yaw = 0.0
		self.tree=None
		self.OpenDRIVE_loaded=False
		self.half_pc_old =None
		self.camera_info_left=None
		self.camera_info_right=None
		self.img_left_msg =None   
		self.img_right_msg=None
		self.img_left_msg =None  
		self.img_right_msg=None
		self.actual_speed=None
		self.actual_imu=None
		self.m_imu=None
		self.traffic_lights=None
		self.rgb_image=None
		#global plan options
		self.LANEFOLLOW=0
		self.STRAIGHT=1
		self.RIGHT=2
		self.LEFT=3
		self.CHANGELANELEFT=4
		self.CHANGELANERIGHT=5
		self.UNKNOWN=6

		self.old_poses_list=None
		self.old_poses_list_gps=None

		self.option_string='lanefollow'
		self.option_string_dataset='lanefollow'
		self.pose_achieved_x=None
		self.pose_achieved_y=None
		self.obstacles_frame_0=[]

		self.next_index_goal_plan=0

		self.cvbridge = CvBridge()

		self.tf2_buffer_stereo2map = tf2_ros.Buffer()
		self.tf2_listener_stere2map = tf2_ros.TransformListener(self.tf2_buffer_stereo2map)

		self.tf2_buffer_map2stereo = tf2_ros.Buffer()
		self.tf2_listener_map2stereo = tf2_ros.TransformListener(self.tf2_buffer_map2stereo)

		self.tf2_buffer_vel2map = tf2_ros.Buffer()
		self.tf2_listener_vel2map = tf2_ros.TransformListener(self.tf2_buffer_vel2map)

		self.tf2_buffer_map2vel = tf2_ros.Buffer()
		self.tf2_listener_map2vel = tf2_ros.TransformListener(self.tf2_buffer_map2vel)

		self.route_raw=None
		self.stamp=None
		self.yaw=None
		self.msg_global_plan=None
		self.msg_global_plan_gt=None
		self.global_plan_marker=None
		self.cam_left_frame=''
		self.global_path_published=False
		self.tf2_buffer_ẁorld2velodyne = tf2_ros.Buffer()
		self.tf2_listener_world2velodyne = tf2_ros.TransformListener(self.tf2_buffer_ẁorld2velodyne)
		self.tf_br_baselink_hero = tf2_ros.TransformBroadcaster()
		self.tf_br_odom_baselink = tf2_ros.TransformBroadcaster()
		self.tf_br_map_odom = tf2_ros.TransformBroadcaster()
		self.ob_array_tfl = ObstacleArray()
		self.ob_array_tfl.obstacle = []
		self.marker_array_tfl = MarkerArray()
		self.marker_array_tfl.markers = []
		self.bridge = CvBridge()
		self.pose_msg = None
		self.actor_list=[]
		self.actor_vehicles_dict=[]

		#PUBLISHERS
		self.gps_pub = rospy.Publisher('/carina/localization/pose', PoseWithCovarianceStamped, queue_size=1)
		# self.gps_gt_pub = rospy.Publisher('/carina/localization/pose_gt', PoseWithCovarianceStamped, queue_size=1)
		self.lidar_pub_front = rospy.Publisher('/carina/sensor/lidar/front/point_cloud', PointCloud2, queue_size=1)
		# self.pub_marker_objects_pub = rospy.Publisher('/carina/perception/radar/obstacles_marker_array', MarkerArray, queue_size=1)
		# self.pub_obstacle_objects_pub= rospy.Publisher('/carina/perception/radar/front/obstacles_array', ObstacleArray, queue_size=1)
		self.vehicle_state_pub = rospy.Publisher('/carina/vehicle/state', VehicleState, queue_size=1)
		self.route_pub = rospy.Publisher('/carina/navigation/waypoints', Path, queue_size=1, latch=True)
		self.odometry_pub = rospy.Publisher('/carina/localization/odom', Odometry, queue_size=1)
		self.global_plan_raw_pub = rospy.Publisher('/carina/navigation/global_plan_raw', GlobalPlan, queue_size=1, latch=True)
		self.marker_route_pub = rospy.Publisher('/carina/route/points/route_points_array', MarkerArray, queue_size=1, latch=True)
		self.imu_pub = rospy.Publisher('/carina/sensor/imu', Imu, queue_size=1)
		self.imu_gt_pub = rospy.Publisher('/carina/sensor/imu_gt', Imu, queue_size=1)
		self.nav_sat_fix_pub = rospy.Publisher('/carina/localization/nav_sat_fix', NavSatFix, queue_size=1)
		# self.nav_sat_fix_gt_pub = rospy.Publisher('/carina/localization/nav_sat_fix_gt', NavSatFix, queue_size=1)
		self.img_left_pub = rospy.Publisher('/carina/sensor/camera/left/image_raw', Image, queue_size=1)
		self.img_left_rect_color_pub = rospy.Publisher('/carina/sensor/camera/left/image_rect_color', Image, queue_size=1)
		self.img_left_info_pub = rospy.Publisher('/carina/sensor/camera/left/camera_info', CameraInfo,queue_size=1)
		self.img_right_pub = rospy.Publisher('/carina/sensor/camera/right/image_raw', Image, queue_size=1)
		self.img_right_rect_color_pub = rospy.Publisher('/carina/sensor/camera/right/image_rect_color', Image, queue_size=1)
		self.img_right_info_pub = rospy.Publisher('/carina/sensor/camera/right/camera_info', CameraInfo,queue_size=1)
		self.img_back_pub = rospy.Publisher('/carina/sensor/camera/back/image_raw', Image, queue_size=1)
		self.img_back_rect_color_pub = rospy.Publisher('/carina/sensor/camera/back/image_rect_color', Image, queue_size=1)
		self.img_back_info_pub = rospy.Publisher('/carina/sensor/camera/back/camera_info', CameraInfo,queue_size=1)
		self.control_carla_pub = rospy.Publisher('/carla/hero/vehicle_control_cmd',CarlaEgoVehicleControl, queue_size=1)
		# self.pub_marker_obstacles = rospy.Publisher('/carina/perception/dataset/obstacles_marker_array', MarkerArray, queue_size=1)
		# self.pub_obstacles = rospy.Publisher('/carina/perception/dataset/obstacles_array', ObstacleArray, queue_size=1)
		self.pub_option_lane = rospy.Publisher('/carina/map/option_lane', String, queue_size=1)

		# self.pub_global_plan_img = rospy.Publisher('/carina/map/global/global_plan_img', Image, queue_size=1)
		# self.pub_global_plan_line_img = rospy.Publisher('/carina/map/global/global_plan_line_img', Image, queue_size=1)
		# self.pub_front_global_plan_img = rospy.Publisher('/carina/map/global/front_global_plan_img', Image, queue_size=1)
		# self.pub_front_global_plan_line_img = rospy.Publisher('/carina/map/global/front_global_plan_line_img', Image, queue_size=1)
		# self.pub_front_global_plan_depth = rospy.Publisher('/carina/map/global/front_global_plan_depth_img', Image, queue_size=1)
		# self.pub_gps_backward_png = rospy.Publisher('/carina/map/global/pub_gps_backward_png_img', Image, queue_size=1)
		# self.pub_gps_backward_color_png = rospy.Publisher('/carina/map/global/pub_gps_backward_color_png_img', Image, queue_size=1)

		#SUBSCRIBERS
		self.shutdown_sub = rospy.Subscriber('/carina/vehicle/shutdown', Bool, self.shutdown_cb)
		self.throttle_sub = rospy.Subscriber('/carina/control/throttle_cmd', Throttle, self.throttle_cb)
		self.brake_sub = rospy.Subscriber('/carina/control/brake_cmd', Brake, self.brake_cb)
		self.steer_sub = rospy.Subscriber('/carina/control/steer_cmd', SteeringAngle, self.steer_cb)
		self.hand_brake_sub = rospy.Subscriber('/carina/control/hand_brake_cmd', Bool, self.hand_brake_cb)
		#####################################################################################################################
		####################################### raw sensory data coming from simulator ######################################
		#####################################################################################################################
		self.caminfol_sub=rospy.Subscriber('/carla/hero/CameraLeft/camera_info', CameraInfo, self.cam_info_left_rgb_cb)
		self.caml_sub=rospy.Subscriber('/carla/hero/CameraLeft/image', Image, self.camera_left_rgb_cb)
		self.caminfor_sub=rospy.Subscriber('/carla/hero/CameraRight/camera_info', CameraInfo, self.cam_info_right_rgb_cb)
		self.camr_sub=rospy.Subscriber('/carla/hero/CameraRight/image', Image, self.camera_right_rgb_cb)
		self.caminfor_sub=rospy.Subscriber('/carla/hero/CameraBack/camera_info', CameraInfo, self.cam_info_back_rgb_cb)
		self.camr_sub=rospy.Subscriber('/carla/hero/CameraBack/image', Image, self.camera_back_rgb_cb)
		self.gps_sub=rospy.Subscriber('/carla/hero/GPS', NavSatFix,  self.gps_geodesic_cb)
		self.speedometer_sub=rospy.Subscriber('/carla/hero/SPEED', CarlaSpeedometer,  self.speed_cb)
		# self.globalpw_sub=rospy.Subscriber('/carla/hero/global_plan', CarlaRoute,  self.global_plan_world_cb)
		self.globalp_sub=rospy.Subscriber('/carla/hero/global_plan_gnss', CarlaGnssRoute,  self.global_plan_gps_cb)
		if self.track == Track.MAP or not (self.track == Track.MAP or self.track == Track.SENSORS):#'DATASET':
			self.globalp_gt_sub=rospy.Subscriber('/carla/hero/global_plan_gnss_gt', CarlaGnssRoute,  self.global_plan_gps_gt_cb)
		# self.status_sub=rospy.Subscriber('/carla/hero/status', Bool,  self.status_cb)
		self.lidar_sub=rospy.Subscriber('/carla/hero/LIDAR', PointCloud2,  self.lidar_cb)
		self.imu_sub=rospy.Subscriber('/carla/hero/IMU', Imu,  self.imu_cb)
		if self.track == Track.MAP or not (self.track == Track.MAP or self.track == Track.SENSORS):#DATASET:
			self.opendrive_sub=rospy.Subscriber('/carla/hero/OpenDRIVE', String, self.opendrive_callback  )
			#if self.track == Track.MAP:
			self.hdmap_pub = rospy.Publisher('/carina/map/hdmap', HDMapMsg, queue_size=1, latch=True)
		if self.track == Track.MAP or self.track == Track.SENSORS:  #---->
			# self.pose_corrected_sub = rospy.Subscriber('/localization/ekf/global', Odometry, self.pose_corrected_cb)
			self.pose_corrected_sub = rospy.Subscriber('/localization/ekf/global', PoseWithCovarianceStamped, self.pose_corrected_cb)
		else: #self.track = 'DATASET'
			if self.create_dataset_depth:
				pass
				# self.img_depth_pub = rospy.Publisher('/carina/sensor/camera/left/depth/image_raw', Image, queue_size=1)
				# self.img_depth_info_pub = rospy.Publisher('/carina/sensor/camera/left/depth/camera_info', CameraInfo,queue_size=1)
				# self.img_depth_r_pub = rospy.Publisher('/carina/sensor/camera/left/depth/image_raw', Image, queue_size=1)
				# self.img_depth_r_info_pub = rospy.Publisher('/carina/sensor/camera/left/depth/camera_info', CameraInfo,queue_size=1)
			if self.create_dataset_images:
				pass
				# self.img_sem_pub = rospy.Publisher('/carina/sensor/camera/left/sem/image_raw', Image, queue_size=1)
				# self.img_sem_info_pub = rospy.Publisher('/carina/sensor/camera/left/sem/camera_info', CameraInfo,queue_size=1)
			if self.create_occupancy_grid:
				self.camera_aerial_sub=rospy.Subscriber('/carla/hero/CameraAerialSeg/image', Image, self.camera_aerial_seg_cb)
				# self.img_aerial_pub = rospy.Publisher('/carina/sensor/camera/aerial/image_raw', Image, queue_size=1)
				# self.img_aerial_info_pub = rospy.Publisher('/carina/sensor/camera/aerial/camera_info', CameraInfo,queue_size=1)
				# self.img_aerial_seg_pub = rospy.Publisher('/carina/sensor/camera/aerial_seg/image_raw', Image, queue_size=1)
				# self.img_aerial_seg_info_pub = rospy.Publisher('/carina/sensor/camera/aerial_seg/camera_info', CameraInfo,queue_size=1)
				self.occupation_map_pub = rospy.Publisher('/carina/map/OccupancyGrid', OccupancyGrid, queue_size=10, latch=True)
				self.occupation_map_image_pub = rospy.Publisher('/carina/map/OccupancyGrid/image_raw', Image, queue_size=1)
			if self.create_dataset_obstacles:
				# self.markers_sub=rospy.Subscriber('/carla/hero/Markers', MarkerArray,  self.markers_cb)
				self.objects_sub=rospy.Subscriber('/carla/hero/Objects', ObjectArray,  self.objects_cb)
				self.tfl_status_sub=rospy.Subscriber('/carla/hero/traffic_lights/status', CarlaTrafficLightStatusList,  self.traffic_light_status_list_cb)
				self.tfl_info_sub=rospy.Subscriber('/carla/hero/traffic_lights/info', CarlaTrafficLightInfoList,  self.traffic_light_info_list_cb)
				self.obj_info_sub=rospy.Subscriber('/carla/hero/actor_list_bjects', CarlaActorList,  self.actor_list_cb)
				self.pub_marker_obstacles = rospy.Publisher('/carina/perception/dataset/obstacles_marker_array', MarkerArray, queue_size=1)
				self.pub_obstacles = rospy.Publisher('/carina/perception/dataset/obstacles_array', ObstacleArray, queue_size=1)
				self.obstacle_tf_red_array_pub = rospy.Publisher('/carina/perception/lidar/obstacles_array_tfl_all',ObstacleArray, queue_size=1)
				self.marker_tf_red_array_pub = rospy.Publisher('/carina/perception/lidar/tfl_marker_array_all', MarkerArray, queue_size=1)
				self.poses_tfl_publisher = rospy.Publisher('/carina/perception/dataset/tfl/poses_array', PoseArray, queue_size=1)
			if self.create_dataset_path:
				self.path_pub = rospy.Publisher('/carina/navigation/path', Path, queue_size=1, latch=True)
				self.path_ros_pub = rospy.Publisher('/carina/navigation/path_global_ros', Path, queue_size=1, latch=True)

		# if self.track == Track.MAP or self.track == Track.SENSORS:#DATASET:
		# 	rospy.Timer(rospy.Duration(0.05), self.publish_tf)
		# else:
		# 	rospy.Timer(rospy.Duration(0.05), self.publish_tf)

		rospy.Timer(rospy.Duration(0.05), self.publish_tf)
		# rospy.Timer(rospy.Duration(0.05), self.publish_plan_img)


	def throttle_cb(self, msg):
		self.throttle = msg.value
			
	def brake_cb(self, msg):
		self.brake = msg.value

	def hand_brake_cb(self, msg):
		self.hand_brake = msg.data

	def steer_cb(self, msg):
	# 	#################################################################################################
	# 	###################################### SENDING CONTROL CMD TO CARLA ##################################################
	# 	#################################################################################################
		if msg.angle > self.max_steering:
				msg.angle = self.max_steering
		elif msg.angle < -self.max_steering:
				msg.angle = -self.max_steering
			
		self.steering_angle = -msg.angle/self.max_steering

		if self.lateral_control_noise and not (self.track == Track.MAP or self.track == Track.SENSORS): #DATASET
			self.steering_angle += np.random.normal(0, .50)
			# print('noise steering_angle', self.steering_angle)

		control = CarlaEgoVehicleControl()

		control.header.stamp = rospy.Time().now()
		# The CARLA vehicle control data
		# 0. <= throttle <= 1. float32
		control.throttle = self.throttle
		# -1. <= steer <= 1.float32
		control.steer = self.steering_angle
		# 0. <= brake <= 1. float32
		control.brake = self.brake
		# hand_brake 0 or 1  bool
		control.hand_brake = self.hand_brake
		# # reverse 0 or 1  bool
		# control.reverse = 
		# gear  int32
		# control.gear = 1
		# # manual gear shift bool
		# control.manual_gear_shift = 
		self.control_carla_pub.publish(control)

	def speed_cb(self, msg):
		self.actual_speed=msg.speed
		#################################################################################################
		####################################### STATE ###################################################
		#################################################################################################
		#vehicle state
		vehicle_state_msg = VehicleState()
		vehicle_state_msg.header.stamp = rospy.Time().now()#msg.header.stamp#rospy.Time().now()
		vehicle_state_msg.drive.speed=self.actual_speed#abs(input_data['SPEED'][1]['speed'])
		# self.speed_temp=vehicle_state_msg.drive.speed
		vehicle_state_msg.drive.steering_angle = -self.steering_angle*self.max_steering
		vehicle_state_msg.handbrake=self.hand_brake
		vehicle_state_msg.brake=int(self.brake)
		self.vehicle_state_pub.publish(vehicle_state_msg)


	def camera_left_rgb_cb(self, msg):
		self.cam_left_frame=msg.header.frame_id
		img_left_msg = Image()
		img_left_msg.header.stamp = rospy.Time().now()#msg.header.stamp
		img_left_msg.header.frame_id = self.cam_left_frame#"stereo"
		img_left_msg.height = msg.height#img_left.shape[0]
		img_left_msg.width = msg.width#img_left.shape[1]
		img_left_msg.encoding = msg.encoding#'rgb8'
		img_left_msg.step=msg.step#img_left.shape[1]*3
		img_left_msg.data=msg.data#img_left_flat[0].tolist()
		self.img_left_msg=img_left_msg
		if self.camera_info_left is not None and self.camera_info_right is not None and self.img_right_msg is not None  and self.img_back_msg is not None:
			self.img_left_info_pub.publish(self.camera_info_left)
			self.img_right_info_pub.publish(self.camera_info_right)
			self.img_back_info_pub.publish(self.camera_info_back)

			self.img_left_pub.publish(self.img_left_msg)    
			self.img_right_pub.publish(self.img_right_msg) 
			self.img_back_pub.publish(self.img_back_msg) 

			self.img_left_rect_color_pub.publish(self.img_left_msg)     
			self.img_right_rect_color_pub.publish(self.img_right_msg) 
			self.img_back_rect_color_pub.publish(self.img_back_msg) 
		try:
			self.rgb_image = self.cvbridge.imgmsg_to_cv2(msg, "bgr8")
		except CvBridgeError as e:
			print (e)
		self.publish_plan_img()

	def cam_info_left_rgb_cb(self, msg):
		camera_info_left= CameraInfo()
		camera_info_left.header.frame_id = msg.header.frame_id#"stereo"
		camera_info_left.header.stamp = rospy.Time().now()#msg.header.stamp
		camera_info_left.width =  msg.width#img_left_msg.width
		camera_info_left.height =  msg.height#img_left_msg.height
		camera_info_left.distortion_model= msg.distortion_model#'plumb_bob'
		camera_info_left.K =  msg.K#[fx2, 0, cx, 0, fy2, cy, 0, 0, 1]
		camera_info_left.D =  msg.D#[0, 0, 0, 0, 0]
		camera_info_left.R =  msg.R#[1.0, 0, 0, 0, 1.0, 0, 0, 0, 1.0]
		camera_info_left.P =  msg.P#[fx2, 0, cx, 0, 0, fy2, cy, 0, 0, 0, 1.0, 0]
		self.camera_info_left=camera_info_left
		# self.rgb_cam_info


	def camera_right_rgb_cb(self, msg):
		img_right_msg = Image()
		img_right_msg.header.stamp = rospy.Time().now()#msg.header.stamp
		img_right_msg.header.frame_id = self.cam_left_frame#"stereo"
		img_right_msg.height = msg.height#img_right.shape[0]
		img_right_msg.width = msg.width #img_right.shape[1]
		img_right_msg.encoding = msg.encoding #'rgb8'
		img_right_msg.step= msg.step#img_right.shape[1]*3
		img_right_msg.data= msg.data#img_right_flat[0].tolist()
		self.img_right_msg = img_right_msg

	def cam_info_right_rgb_cb(self, msg):
		camera_info_right= CameraInfo()
		camera_info_right.header.frame_id = self.cam_left_frame#"stereo"
		camera_info_right.header.stamp = rospy.Time().now()#msg.header.stamp #stamp
		camera_info_right.width =  msg.width#img_right_msg.width
		camera_info_right.height =  msg.height#img_right_msg.height
		camera_info_right.distortion_model= msg.distortion_model#'plumb_bob'
		fx2=msg.K[0]
		cx=msg.K[2]
		fy2=msg.K[4]
		cy=cx=msg.K[5]
		camera_info_right.K =  msg.K#[fx2, 0, cx, 0, fy2, cy, 0, 0, 1]
		camera_info_right.D =  msg.D#[0, 0, 0, 0, 0]
		camera_info_right.R =  msg.R#[1.0, 0, 0, 0, 1.0, 0, 0, 0, 1.0]
		camera_info_right.P = [fx2, 0, cx, -fx2*self.baseline, 0, fy2, cy, 0, 0, 0, 1.0, 0]
		self.camera_info_right=camera_info_right

	def camera_back_rgb_cb(self, msg):
		img_back_msg = Image()
		img_back_msg.header.stamp = rospy.Time().now()#msg.header.stamp
		img_back_msg.header.frame_id = self.cam_left_frame#"stereo"
		img_back_msg.height = msg.height#img_back.shape[0]
		img_back_msg.width = msg.width #img_back.shape[1]
		img_back_msg.encoding = msg.encoding #'rgb8'
		img_back_msg.step= msg.step#img_back.shape[1]*3
		img_back_msg.data= msg.data#img_back_flat[0].tolist()
		self.img_back_msg = img_back_msg

		# self.publish_plan_img()

	def cam_info_back_rgb_cb(self, msg):
		camera_info_back= CameraInfo()
		camera_info_back.header.frame_id = self.cam_left_frame#"stereo"
		camera_info_back.header.stamp = rospy.Time().now()#msg.header.stamp #stamp
		camera_info_back.width =  msg.width#img_back_msg.width
		camera_info_back.height =  msg.height#img_back_msg.height
		camera_info_back.distortion_model= msg.distortion_model#'plumb_bob'
		fx2=msg.K[0]
		cx=msg.K[2]
		fy2=msg.K[4]
		cy=cx=msg.K[5]
		camera_info_back.K =  msg.K#[fx2, 0, cx, 0, fy2, cy, 0, 0, 1]
		camera_info_back.D =  msg.D#[0, 0, 0, 0, 0]
		camera_info_back.R =  msg.R#[1.0, 0, 0, 0, 1.0, 0, 0, 0, 1.0]
		camera_info_back.P = [fx2, 0, cx, -fx2*self.baseline, 0, fy2, cy, 0, 0, 0, 1.0, 0]
		self.camera_info_back=camera_info_back

	def camera_aerial_seg_cb(self, msg):
		if self.actual_imu is None or self.pose_msg is None:
			return
		img_aerialseg_msg = Image()
		img_aerialseg_msg.header.stamp = rospy.Time().now()
		img_aerialseg_msg.header.frame_id = self.cam_left_frame
		img_aerialseg_msg.height = msg.height
		img_aerialseg_msg.width = msg.width
		img_aerialseg_msg.encoding =  msg.encoding 
		img_aerialseg_msg.step=msg.step
		img_aerialseg_msg.data=msg.data

		#################################################################################################################
		####################################################ocuppancy map ###############################################
		#################################################################################################################
		self.map_msg = map_msg = OccupancyGrid()
		map_msg.header.frame_id='map'
		try:
			cv_image = self.bridge.imgmsg_to_cv2(img_aerialseg_msg, "bgr8")
		except CvBridgeError as e:
			print(e)
		map_img = (cv_image[:,:,0]).astype(np.uint8)

		map_img = np.fliplr(map_img)

		map_img_0=np.zeros_like(map_img)
		map_img_0[map_img==128]=100
		map_img_0[map_img==50]=100
		map_img_0[map_img==153]=100

		img_map_msg = Image()
		img_map_msg.header.stamp = rospy.Time().now()
		img_map_msg.header.frame_id = "base_link"
		img_map_msg.height = map_img_0.shape[0]
		img_map_msg.width = map_img_0.shape[1]
		img_map_msg.encoding = 'mono8'
		img_map_msg.step=map_img_0.shape[1]
		img_map_msg.data=map_img_0.ravel().tolist()#img_left_flat[0].tolist()

		map_msg.data = map_img_0.ravel().tolist()

			# set up general info
		map_msg.info.resolution = 0.125#self.carla_map._pixel_density
		map_msg.info.width = map_img_0.shape[1]
		map_msg.info.height = map_img_0.shape[0]

			# set up origin orientation
		map_msg.info.origin.orientation.x = self.actual_imu.orientation.x#quat[0]
		map_msg.info.origin.orientation.y = self.actual_imu.orientation.y#quat[1]
		map_msg.info.origin.orientation.z = self.actual_imu.orientation.z#quat[2]
		map_msg.info.origin.orientation.w = self.actual_imu.orientation.w#quat[3]

			# set up origin position
		top_right_corner = float(map_img_0.shape[1]/2), 0.0
		xp=float(map_img_0.shape[0]/3)*map_msg.info.resolution*2  #* np.cos(self.yaw_gt)
		yp=-float(map_img_0.shape[1]/2)*map_msg.info.resolution  #* np.sin(self.yaw_gt)
		map_msg.info.origin.position.x = self.pose_msg.pose.pose.position.x + xp*np.cos(self.yaw) - yp*np.sin(self.yaw)  #to_world[0]
		map_msg.info.origin.position.y = self.pose_msg.pose.pose.position.y + xp*np.sin(self.yaw) + yp*np.cos(self.yaw)  #-to_world[1]
		map_msg.info.origin.position.z = self.pose_msg.pose.pose.position.z#-self.carla_map._converter._worldoffset[2]

		self.occupation_map_image_pub.publish(img_map_msg)     
		self.occupation_map_pub.publish(self.map_msg)


	def lidar_cb(self, msg):
		pc = None
		if self.lidar_pub_front.get_num_connections() > 0:
			pc = np.array(list(read_points(msg)))
			header = Header()
			header.stamp = rospy.Time().now()#msg.header.stamp
			header.frame_id='velodyne'#msg.header.frame_id#
			# pc = pc[..., [0,1,2]]
			# point_front=np.array([    [2.7,0.4,-1.9], [2.7,0.3,-1.9], [2.7,0.2,-1.9], [2.7,0.1,-1.9],[2.7,0.0, -1.9],
			# 						  [2.7,-0.4,-1.9],[2.7,-0.3,-1.9],[2.7,-0.2,-1.9],[2.7,-0.1,-1.9],
			# 						  [3,0.5,-1.9],   [3,0.4,-1.9],   [3,0.3,-1.9],   [3,0.2,-1.9], [3,0.1,-1.9],[3,0.0,-1.9],
			# 						  [3,-0.5,-1.9],  [3,-0.4,-1.9],  [3,-0.3,-1.9],  [3,-0.2,-1.9],[3,-0.1,-1.9],
			# 						  [3.3,0.5,-1.9], [3.3,0.4,-1.9], [3.3,0.3,-1.9], [3.3,0.2,-1.9], [3.3,0.1,-1.9],[3.3,0.0,-1.9],
			# 						  [3.3,-0.5,-1.9],[3.3,-0.4,-1.9],[3.3,-0.3,-1.9],[3.3,-0.2,-1.9],[3.3,-0.1,-1.9]
			# 						  ])
			# pc = np.concatenate((pc, point_front), axis=0) 
			# if self.half_pc_old is None:
			# 	pc_msg = create_cloud_xyz32(header, pc)
			# else:
			# 	pc_front = np.concatenate((self.half_pc_old, pc), axis=0)			
			# 	pc_msg = create_cloud_xyz32(header, pc_front)

			# self.half_pc_old=pc
			# self.lidar_pub_front.publish(pc_msg)
			pc = pc[..., [0,1,2,3]]
			# we take the opposite of y axis
			# (as lidar point are express in left handed coordinate system, and ros need right handed)
			# pc[:,1]=-pc[:,1]
			if self.half_pc_old is not None:
				pc_all = np.concatenate((self.half_pc_old, pc), axis=0)
			else:
				pc_all = pc

			self.half_pc_old=pc

			fields = [
			    PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
			    PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
			    PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
			    PointField(name='intensity', offset=12, datatype=PointField.FLOAT32, count=1)
			]

			# lidar_data = np.fromstring( bytes(pc), dtype=np.float32)
			# lidar_data = np.reshape(lidar_data, (int(lidar_data.shape[0] / 4), 4))
			point_cloud_msg = create_cloud(header, fields, pc_all)
			self.lidar_pub_front.publish(point_cloud_msg)

	def radar_cb(self, msg):
		pass


	def imu_cb(self, msg):
		m_imu = Imu()
		m_imu.header.stamp=rospy.Time().now()#msg.header.stamp#rospy.Time().now()
		m_imu.header.frame_id="imu"
		roll, pitch, yaw = tf.transformations.euler_from_quaternion([msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w])
		m_imu.orientation.x = msg.orientation.x#m_imu_ori[0]#m_imu_ori[0]#
		m_imu.orientation.y = msg.orientation.y#m_imu_ori[1]#m_imu_ori[1]#
		m_imu.orientation.z = msg.orientation.z#m_imu_ori[2]#m_imu_ori[2]#
		m_imu.orientation.w = msg.orientation.w#m_imu_ori[3]#m_imu_ori[3]#
		#m_imu.orientation_covariance=[-1.0, 0.0, 0.0,
		# 							   0.0, 0.0, 0.0,
		#                               0.0, 0.0, 0.0]
		m_imu.linear_acceleration=msg.linear_acceleration#Vector3(input_data['IMU'][1][0], -input_data['IMU'][1][1], input_data['IMU'][1][2])
		#m_imu.linear_acceleration = Vector3(x, y, z)
		m_imu.linear_acceleration_covariance=[0.001**2, 0.0, 0.0,
                                              0.0, 0.001**2, 0.0,
		                                      0.0, 0.0, 0.015**2]
		m_imu.angular_velocity =msg.angular_velocity#Vector3(-input_data['IMU'][1][3], input_data['IMU'][1][4], -input_data['IMU'][1][5])
		#m_imu.linear_acceleration = Vector3(x, y, z)
		m_imu.angular_velocity_covariance=[0.001**2, 0.0, 0.0,
	                                        0.0, 0.001**2, 0.0,
                                            0.0, 0.0, 0.001**2]
		self.actual_imu=m_imu
			
		roll, pitch, yaw = tf.transformations.euler_from_quaternion(
				[self.actual_imu.orientation.x, self.actual_imu.orientation.y, self.actual_imu.orientation.z, 
				self.actual_imu.orientation.w])
		self.yaw= yaw - np.pi/2
		qat=tf.transformations.quaternion_from_euler(roll, pitch, self.yaw)
		m_imu.orientation.x = qat[0]#m_imu_ori[0]#m_imu_ori[0]#
		m_imu.orientation.y = qat[1]#m_imu_ori[1]#m_imu_ori[1]#
		m_imu.orientation.z = qat[2]#m_imu_ori[2]#m_imu_ori[2]#
		m_imu.orientation.w = qat[3]#m_imu_ori[3]#m_imu_ori[3]#
		self.m_imu=m_imu
		# print('imu: ')
		# print(self.m_imu)
		self.imu_pub.publish(self.m_imu)


	def opendrive_callback(self, msg_sting_map):
		stamp=rospy.Time().now()#self.stamp#msg.header.stamp
		msg = HDMapMsg()
		msg.header.stamp = stamp
		msg.header.frame_id = 'map'

		msg.XML_HDMap=msg_sting_map.data
		tree = ET.ElementTree(ET.fromstring(str(msg.XML_HDMap)))
		root = tree.getroot()
		georeference = root.find('header/geoReference')
		self.datum = georeference.text

		str_list = self.datum.split(' ')
		for item in str_list:
			if '+lat_0' in item:
				self.lat_ref = float(item.split('=')[1])
			if '+lon_0' in item:
				self.lon_ref = float(item.split('=')[1])

		datum_list=self.datum.split(' ')
		newdatum=[]
		for d in datum_list:
			if '+geoidgrids' not in d:
				newdatum.append(d)
		_datum=' '.join(newdatum)
		self.hdmap_pub.publish(msg)
		self.OpenDRIVE_loaded=True


	def objects_cb(self, msg):
		objects= msg.objects.copy()
		actor_vehicles_dict=self.actor_vehicles_dict.copy()

		ob_array = ObstacleArray()
		ob_array.obstacle = []
		marker_array = MarkerArray()
		marker_array.markers = []

		classe_id = {0:"CLASSIFICATION_UNKNOWN",
					1:"CLASSIFICATION_UNKNOWN_SMALL",
					2:"CLASSIFICATION_UNKNOWN_MEDIUM",
					3:"CLASSIFICATION_UNKNOWN_BIG",
					4:"CLASSIFICATION_PEDESTRIAN",
					5:"CLASSIFICATION_BIKE",
					6:"CLASSIFICATION_CAR",
					7:"CLASSIFICATION_TRUCK",
					8:"CLASSIFICATION_MOTORCYCLE",
					9:"CLASSIFICATION_OTHER_VEHICLE",
					10:"CLASSIFICATION_BARRIER",
					11:"CLASSIFICATION_SIGN"
						}
		classe_color = {
						'CLASSIFICATION_UNKNOWN':(1,1,1),
						'CLASSIFICATION_UNKNOWN_SMALL':(1,1,1),
						'CLASSIFICATION_UNKNOWN_MEDIUM':(1,1,1),
						'CLASSIFICATION_UNKNOWN_BIG':(1,1,1),
						'CLASSIFICATION_PEDESTRIAN':(1,0,0),
						'CLASSIFICATION_BIKE':(1,1,0),
						'CLASSIFICATION_CAR':(0,1,0),
						'CLASSIFICATION_TRUCK':(0,0,1),
						'CLASSIFICATION_MOTORCYCLE':(0,1,1),
						'CLASSIFICATION_OTHER_VEHICLE':(0,0,0),
						'CLASSIFICATION_BARRIER':(0.5,0.5,0),
						'CLASSIFICATION_SIGN':(0,0.5,0.5),
						'CLASSIFICATION_VAN':(0,0.5,0.5)
					}

		try:
			trans = self.tf2_buffer_ẁorld2velodyne.lookup_transform('hero/LIDAR', 'map', rospy.Time())

			for o in objects:
				old_pose = PoseStamped()
				old_pose.header.frame_id='map'#'hero/LIDAR'
				old_pose.header.stamp = rospy.Time().now()#msg.header.stamp
				old_pose.pose.position.x = o.pose.position.y
				old_pose.pose.position.y = -o.pose.position.x
				old_pose.pose.position.z = o.pose.position.z

				dist=np.linalg.norm(np.array([self.pose[0],self.pose[1],self.pose[2]]) - np.array([ old_pose.pose.position.x, old_pose.pose.position.y, 
																															old_pose.pose.position.z]))
				dist_z=np.abs(self.pose[2]- old_pose.pose.position.z)

				if dist>90 or dist_z>10:
					continue
				old_pose.pose.orientation.x = o.pose.orientation.x
				old_pose.pose.orientation.y = o.pose.orientation.y
				old_pose.pose.orientation.z = o.pose.orientation.z
				old_pose.pose.orientation.w = o.pose.orientation.w				
				new_pose = tf2_geometry_msgs.do_transform_pose(old_pose, trans)

				if 0.0 in o.shape.dimensions:
					dim=(1.0,1.0,1.0)
				else:
					dim=o.shape.dimensions

				ob = Obstacle()
				ob.header.frame_id = "hero/LIDAR"
				ob.header.stamp = rospy.Time().now()#o.header.stamp#msg.header.stamp
				ob.id = o.id

				if o.id in actor_vehicles_dict:
					ob.vehicle_model = actor_vehicles_dict[o.id]
					ob.classes = [self.vehicle_models_map[actor_vehicles_dict[o.id]]]
				else:
					# print("Key error: the provided key is not a Vehicle or is not in vehicle list..............")
					ob.classes = [classe_id[o.classification]]
				color = classe_color[ob.classes[0]]

				ob.ns = 'signs_carla_tk4'
				ob.class_id=0
				ob.type = -1
				ob.pose.position.x=new_pose.pose.position.x#o.pose.position.x
				ob.pose.position.y=new_pose.pose.position.y#-o.pose.position.y
				if ob.classes[0] == "CLASSIFICATION_PEDESTRIAN":
					ob.pose.position.z=new_pose.pose.position.z #o.pose.position.z
				else:
					ob.pose.position.z=new_pose.pose.position.z + dim[2]/2#o.pose.position.z
				r,p,y=tf.transformations.euler_from_quaternion([new_pose.pose.orientation.x, new_pose.pose.orientation.y,
																													new_pose.pose.orientation.z,
																													new_pose.pose.orientation.w])
				pose_ori_plus_90= tf.transformations.quaternion_from_euler(r,p,y-np.pi/2)

				ob.pose.orientation.x = pose_ori_plus_90[0]
				ob.pose.orientation.y = pose_ori_plus_90[1]
				ob.pose.orientation.z = pose_ori_plus_90[2]
				ob.pose.orientation.w = pose_ori_plus_90[3]	

				ob.scale.x = dim[0]+0.2#2.0#tfl.scale.x
				ob.scale.y = dim[1]+0.2#2.0#tfl.scale.y
				# ob.scale.z = dim[2]#2.0#tfl.scale.z
				if  ob.classes[0] == 'CLASSIFICATION_MOTORCYCLE' or  ob.classes[0] == 'CLASSIFICATION_BIKE':#CLASSIFICATION_MOTORCYCLE or CLASSIFICATION_BIKE
					ob.scale.z = dim[2] + 0.50#tfl.scale.z
					ob.pose.position.z= new_pose.pose.position.z + (dim[2] + 0.50)/2 #o.pose.position.z
					ob.scale.x = dim[0]+1.#2.0#tfl.scale.x

				else:
					ob.scale.z = dim[2]#tfl.scale.z

				ob.color.r = color[0]
				ob.color.g = color[1]
				ob.color.b = color[2]
				ob.color.a =1
				ob.lifetime = rospy.Duration(0.25)
				ob.track_status = 1	
				ob_array.obstacle.append(ob)

				if self.pub_marker_obstacles.get_num_connections() > 0:

					mk = Marker()
					mk.header.frame_id = "hero/LIDAR"
					mk.header.stamp = rospy.Time().now()#o.header.stamp#msg.header.stamp
					mk.id = o.id
					mk.ns = 'signs_carla_tk4'
					mk.type = mk.CUBE
					mk.action = mk.ADD
					mk.pose.position.x=new_pose.pose.position.x#o.pose.position.x
					mk.pose.position.y=new_pose.pose.position.y#-o.pose.position.y
					if ob.classes[0] == "CLASSIFICATION_PEDESTRIAN":
						mk.pose.position.z=new_pose.pose.position.z #o.pose.position.z
					else:
						mk.pose.position.z=new_pose.pose.position.z + dim[2]/2#o.pose.position.z
					mk.pose.orientation.x=ob.pose.orientation.x#new_pose.pose.orientation.x
					mk.pose.orientation.y=ob.pose.orientation.y#new_pose.pose.orientation.y
					mk.pose.orientation.z=ob.pose.orientation.z#new_pose.pose.orientation.z
					mk.pose.orientation.w=ob.pose.orientation.w#new_pose.pose.orientation.w

					mk.scale.x = dim[0]+0.2#tfl.scale.x
					mk.scale.y = dim[1]+0.2#tfl.scale.y
					if  ob.classes[0] == 'CLASSIFICATION_MOTORCYCLE' or  ob.classes[0] == 'CLASSIFICATION_BIKE':#CLASSIFICATION_MOTORCYCLE or CLASSIFICATION_BIKE
						mk.scale.z = dim[2] + 0.70#tfl.scale.z
						mk.pose.position.z= new_pose.pose.position.z + (dim[2] + 0.70)/2 #o.pose.position.z
						mk.scale.x = dim[0]+1.#2.0#tfl.scale.x
					else:
						mk.scale.z = dim[2]#tfl.scale.z

					mk.color.r = color[0]
					mk.color.g = color[1]
					mk.color.b = color[2]
					mk.color.a =.61
					mk.lifetime = rospy.Duration(0.25)
					marker_array.markers.append(mk)

			self.pub_obstacles.publish(ob_array)
			if self.pub_marker_obstacles.get_num_connections() > 0:
				self.pub_marker_obstacles.publish(marker_array)

			self.obstacle_tf_red_array_pub.publish(self.ob_array_tfl)
			if self.marker_tf_red_array_pub.get_num_connections() > 0:
				self.marker_tf_red_array_pub.publish(self.marker_array_tfl)

			ob_array.obstacle = []
			marker_array.markers = []
		except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
			print ('[objects_cb] exception waiting for tf from frames map to hero/LIDAR')










	def traffic_light_status_list_cb(self, msg):
		if self.stamp is None:
			stamp=rospy.Time().now()
		else:
			stamp=rospy.Time().now()#self.stamp
		ob_array_tfl = ObstacleArray()
		ob_array_tfl.obstacle = []
		marker_array_tfl = MarkerArray()
		marker_array_tfl.markers = []

		ps = PoseArray()
		ps.header.frame_id = "hero/LIDAR"
		ps.header.stamp = rospy.Time.now()

		ps.poses=[]

		classe_id = {0:"traffic_light_red",
						1:"traffic_light_yellow",
						2:"traffic_light_green",
						3:"off",
						4:"unknown"
						}
		classe_color = {
						0:(1,0,0),
						1:(1,1,0),
						2:(0,1,0),
						3:(1,1,1),
						4:(1,1,1)
					}
		status=msg.traffic_lights

		if self.traffic_lights is not None:
				for tfl, status in zip(self.traffic_lights,status):
						r,p,yaw=tf.transformations.euler_from_quaternion([tfl.transform.orientation.x, 
																		  tfl.transform.orientation.y,
																		  tfl.transform.orientation.z,
																		  tfl.transform.orientation.w])
						c, s = np.cos(-yaw), np.sin(-yaw)
						j = np.matrix([[c, s], [-s, c]])
						m = np.dot(j, [tfl.trigger_volume.center.x, tfl.trigger_volume.center.y])
						xx,yy = float(m.T[0]), float(m.T[1])

						old_pose = PoseStamped()
						old_pose.header.frame_id='map'#'hero/LIDAR'
						old_pose.header.stamp = rospy.Time().now()#stamp
						old_pose.pose.position.x = (tfl.transform.position.y + yy)#tfl.trigger_volume.center.y)
						old_pose.pose.position.y = -(tfl.transform.position.x + xx)#tfl.trigger_volume.center.x)
						old_pose.pose.position.z = tfl.transform.position.z + tfl.trigger_volume.center.z

						dist=np.linalg.norm(np.array([self.pose[0],self.pose[1],self.pose[2]]) - 
								np.array([ old_pose.pose.position.x, old_pose.pose.position.y, old_pose.pose.position.z]))
						
						if dist>200:
							continue

						old_pose.pose.orientation.x = tfl.transform.orientation.x
						old_pose.pose.orientation.y = tfl.transform.orientation.y
						old_pose.pose.orientation.z = tfl.transform.orientation.z
						old_pose.pose.orientation.w = tfl.transform.orientation.w	

						old_pose2 = PoseStamped()
						old_pose2.header.frame_id='map'#'hero/LIDAR'
						old_pose2.header.stamp = rospy.Time().now()#stamp
						old_pose2.pose.position.x = tfl.transform.position.y 
						old_pose2.pose.position.y = -tfl.transform.position.x
						old_pose2.pose.position.z = tfl.transform.position.z 

						old_pose2.pose.orientation.x = tfl.transform.orientation.x
						old_pose2.pose.orientation.y = tfl.transform.orientation.y
						old_pose2.pose.orientation.z = tfl.transform.orientation.z
						old_pose2.pose.orientation.w = tfl.transform.orientation.w	

						color = classe_color[status.state]

						if CarlaTrafficLightStatus.RED==status.state or CarlaTrafficLightStatus.YELLOW==status.state:

							ob = Obstacle()
							ob.header.frame_id = "map"#"hero/LIDAR"
							ob.header.stamp = rospy.Time().now()#stamp#msg.header.stamp
							ob.id = tfl.id+1000
							ob.ns = 'signs_carla_tk4'
							ob.class_id=0
							ob.classes = [classe_id[status.state]]
							ob.type = -1

							ob.pose.position.x = old_pose.pose.position.x #tfl.transform.position.x
							ob.pose.position.y = old_pose.pose.position.y#-tfl.transform.position.y
							ob.pose.position.z = old_pose.pose.position.z#tfl.transform.position.z
			
							ob.pose.orientation.x = old_pose.pose.orientation.x#fl.transform.orientation.x
							ob.pose.orientation.y = old_pose.pose.orientation.y#fl.transform.orientation.y
							ob.pose.orientation.z = old_pose.pose.orientation.z#fl.transform.orientation.z
							ob.pose.orientation.w = old_pose.pose.orientation.w#fl.transform.orientation.w
							ob.scale.x = max(tfl.trigger_volume.size.y, tfl.trigger_volume.size.x)#2.0#tfl.scale.x
							ob.scale.y = max(tfl.trigger_volume.size.y, tfl.trigger_volume.size.x)#2.0#tfl.scale.y
							ob.scale.z = 2.0#tfl.scale.z
							
							ob.color.r = color[0]
							ob.color.g = color[1]
							ob.color.b = color[2]
							ob.color.a =1
							ob.lifetime = rospy.Duration(0.5)
							ob.track_status = 1	
							ob_array_tfl.obstacle.append(ob)

						if self.marker_tf_red_array_pub.get_num_connections() > 0:
							mk = Marker()
							mk.header.frame_id = "map"#"hero/LIDAR"
							mk.header.stamp = rospy.Time().now()#stamp#msg.header.stamp
							mk.id = tfl.id+1000
							mk.ns = 'signs_carla_tk4'
							mk.type = mk.CUBE
							mk.action = mk.ADD
							mk.pose.position.x = old_pose.pose.position.x 
							mk.pose.position.y = old_pose.pose.position.y 
							mk.pose.position.z = old_pose.pose.position.z 
							mk.pose.orientation.x = old_pose.pose.orientation.x
							mk.pose.orientation.y = old_pose.pose.orientation.y
							mk.pose.orientation.z = old_pose.pose.orientation.z
							mk.pose.orientation.w = old_pose.pose.orientation.w
							mk.scale.x = max(tfl.trigger_volume.size.y, tfl.trigger_volume.size.x)#tfl.scale.x
							mk.scale.y = max(tfl.trigger_volume.size.y, tfl.trigger_volume.size.x)#tfl.scale.y
							mk.scale.z = 0.5#tfl.scale.z
							mk.color.r = color[0]
							mk.color.g = color[1]
							mk.color.b = color[2]
							mk.color.a =1
							mk.lifetime = rospy.Duration(0.5)
							marker_array_tfl.markers.append(mk)

							mk = Marker()
							mk.header.frame_id = "map"#"hero/LIDAR"
							mk.header.stamp = rospy.Time().now()#stamp#msg.header.stamp
							mk.id = tfl.id+2000
							mk.ns = 'signs_carla_tk4'
							mk.type = mk.CUBE
							mk.action = mk.ADD
							mk.pose.position.x = old_pose2.pose.position.x #+ tfl.trigger_volume.center.y
							mk.pose.position.y = old_pose2.pose.position.y #+ tfl.trigger_volume.center.x
							mk.pose.position.z = old_pose2.pose.position.z +2.5#+ tfl.trigger_volume.center.z
							mk.pose.orientation.x = old_pose2.pose.orientation.x
							mk.pose.orientation.y = old_pose2.pose.orientation.y
							mk.pose.orientation.z = old_pose2.pose.orientation.z
							mk.pose.orientation.w = old_pose2.pose.orientation.w
							mk.scale.x = 0.50#tfl.scale.x
							mk.scale.y = 0.50#tfl.scale.y
							mk.scale.z = 5.0#tfl.scale.z
							mk.color.r = color[0]
							mk.color.g = color[1]
							mk.color.b = color[2]
							mk.color.a =1
							mk.lifetime = rospy.Duration(0.5)
							marker_array_tfl.markers.append(mk)
						    
						pose = Pose()
						pose.position.x = old_pose2.pose.position.x
						pose.position.y = old_pose2.pose.position.y
						pose.position.z = old_pose2.pose.position.z
						pose.orientation.x = old_pose2.pose.orientation.x
						pose.orientation.y = old_pose2.pose.orientation.y
						pose.orientation.z = old_pose2.pose.orientation.z
						pose.orientation.w = old_pose2.pose.orientation.w
						ps.poses.append( pose )
		self.ob_array_tfl = ob_array_tfl
		self.marker_array_tfl = marker_array_tfl
		self.poses_tfl_publisher.publish( ps )

	def traffic_light_info_list_cb(self, msg):
		self.traffic_lights=msg.traffic_lights

	def actor_list_cb(self, msg):
		self.actor_list=msg.actors
		actor_vehicles_dict={}
		for actor in self.actor_list:
			if actor.type.startswith('vehicle.'):
				actor_vehicles_dict[actor.id]=actor.type
		self.actor_vehicles_dict=actor_vehicles_dict


	#################################################################################################
	######################################## GPS ####################################################
	#################################################################################################

	def global_plan_gps_cb(self, msg):
		self.msg_global_plan=msg

	def global_plan_gps_gt_cb(self, msg):
		self.msg_global_plan_gt=msg

	def gps_geodesic_cb(self, msg):
		if self.lat_ref==None:
			return 













		self.stamp=rospy.Time().now()#msg.header.stamp
		if self.actual_speed is not None and self.steering_angle is not None:# and stamp.to_sec() > 5: #wait 5 secs to publush pose and transforms
			nav_sat_fix = NavSatFix()
			nav_sat_fix.header.stamp=rospy.Time().now()#msg.header.stamp#rospy.Time().now()
			nav_sat_fix.header.frame_id="gps"
			nav_sat_fix.latitude= msg.latitude#input_data['GPS'][1][0]
			nav_sat_fix.longitude= msg.longitude#input_data['GPS'][1][1]
			nav_sat_fix.altitude = msg.altitude#input_data['GPS'][1][2]
			nav_sat_fix.position_covariance=[10.0,   0.0,   0.0,
												  0.0,  10.0,   0.0,
												  0.0,   0.0,  10.0]
			self.nav_sat_fix_pub.publish(nav_sat_fix)




















			if 	not (self.track == Track.MAP or self.track == Track.SENSORS):#DATASET:# self.CREATE_DATASET:  #---->
				if self.create_dataset_path:
					pose = self.gpsToWorld([msg.latitude, msg.longitude, msg.altitude])
					self.pose=pose
					if self.lat_ref==None or self.msg_global_plan_gt==None or 	self.global_path_published:
						return
					self.publish_path()
					self.global_path_published=True
			else:
				if self.lat_ref==None or self.msg_global_plan==None or 	self.global_path_published:
					return
				self.publish_path()
				self.global_path_published=True




	def pose_corrected_cb(self, msg):
		pose_msg = PoseWithCovarianceStamped()
		pose_msg.header.stamp = self.stamp#msg.header.stamp
		pose_msg.header.frame_id = 'map'#msg.header.frame_id
		pose_msg.pose.pose.position.x = msg.pose.pose.position.x #- 1.2*math.cos(y)
		pose_msg.pose.pose.position.y = msg.pose.pose.position.y #- 1.2*math.sin(y)
		pose_msg.pose.pose.position.z = msg.pose.pose.position.z

		# pose_msg.pose.pose.orientation.x = self.actual_imu.orientation.x
		# pose_msg.pose.pose.orientation.y = self.actual_imu.orientation.y
		# pose_msg.pose.pose.orientation.z = self.actual_imu.orientation.z
		# pose_msg.pose.pose.orientation.w = self.actual_imu.orientation.w
		# self.m_imu
		pose_msg.pose.pose.orientation.x = self.m_imu.orientation.x
		pose_msg.pose.pose.orientation.y = self.m_imu.orientation.y
		pose_msg.pose.pose.orientation.z = self.m_imu.orientation.z
		pose_msg.pose.pose.orientation.w = self.m_imu.orientation.w

		# pose_msg.pose.pose.orientation.x = msg.pose.pose.orientation.x
		# pose_msg.pose.pose.orientation.y = msg.pose.pose.orientation.y
		# pose_msg.pose.pose.orientation.z = msg.pose.pose.orientation.z
		# pose_msg.pose.pose.orientation.w = msg.pose.pose.orientation.w
	
		if self.stamp.to_sec() >0.5: #waiting 1/2 seconds to publish pose
			self.pose=pose_msg
		

	def publish_path(self):
		self.waypoints=None
		self.waypoints_raw, self.global_plan_raw_msg = self.routeToWaypoints([self.msg_global_plan.road_options, self.msg_global_plan.coordinates])

		if not (self.track == Track.MAP or self.track == Track.SENSORS): #DATASET
			self.waypoints_raw_gt, self.global_plan_raw_gt_msg = self.routeToWaypoints([self.msg_global_plan_gt.road_options, self.msg_global_plan_gt.coordinates])

			x_x = np.arange(0, len(self.waypoints_raw_gt))
			x_y = self.waypoints_raw_gt[:,0]
			f = interpolate.interp1d(x_x, x_y)

			xnew = np.arange(0, len(self.waypoints_raw_gt)-1, 0.1)
			ynew = f(xnew)   # use interpolation function returned by `interp1d`
			x=ynew

			y_x = np.arange(0, len(self.waypoints_raw_gt))
			y_y = self.waypoints_raw_gt[:,1]
			f = interpolate.interp1d(y_x, y_y)

			xnew = np.arange(0, len(self.waypoints_raw_gt)-1, 0.1)
			ynew = f(xnew)   # use interpolation function returned by `interp1d`
			y=ynew

			z_x = np.arange(0, len(self.waypoints_raw_gt))
			z_y = self.waypoints_raw_gt[:,2]
			f = interpolate.interp1d(z_x, z_y)

			xnew = np.arange(0, len(self.waypoints_raw_gt)-1, 0.1)
			ynew = f(xnew)   # use interpolation function returned by `interp1d`
			z=ynew
			self.waypoints_raw_gt=np.stack((x, y, z), axis=-1)

		# np.save('/home/luis/town12_1_world.npy',self.waypoints_raw)
		self.route_raw = Path()
		self.route_raw.header.stamp = rospy.Time().now()#self.msg_global_plan.header.stamp#msg.header.stamp#rospy.Time().now()
		self.route_raw.header.frame_id = 'map'
		self.route_raw.poses = []

		if not (self.track == Track.MAP or self.track == Track.SENSORS): #DATASET
			self.waypoints=self.waypoints_raw_gt
		else:
			self.waypoints=self.waypoints_raw

		for p in self.waypoints:
			ps = PoseStamped()
			ps.header.frame_id = 'map'
			ps.header.stamp = rospy.Time().now()#self.msg_global_plan.header.stamp#msg.header.stamp#rospy.Time().now()
			ps.pose.position.x = p[0]
			ps.pose.position.y = p[1]
			ps.pose.position.z = p[2]
			self.route_raw.poses.append(ps)

		markerArray = MarkerArray()
		markerArray.markers = []
		color_plan=[]
		for index, (pose, option) in enumerate(zip(self.global_plan_raw_msg.points, self.global_plan_raw_msg.road_options)):

			marker = Marker()
			marker.header.frame_id = "map"
			marker.type = marker.SPHERE
			marker.action = marker.ADD
			marker.ns = "my_namespace";
			# marker scale
			marker.scale.x = 2.8
			marker.scale.y = 2.8
			marker.scale.z = 2.8
			# marker color
			r=0.0
			g=0.0
			b=0.0
			if self.LANEFOLLOW==option:
				g=1.0
			if self.LEFT==option:
				r=1.0
			if self.RIGHT==option:
				b=1.0
			if self.STRAIGHT==option:
				r=1.0
				g=1.0
				b=1.0
			if self.CHANGELANELEFT==option:
				r=1.0
				g=1.0
			if self.CHANGELANERIGHT==option:
				b=1.0
				g=1.0
			if self.UNKNOWN==option:
				r=1.0
				g=1.0

			color_plan.append((b*255,g*255,r*255))

			marker.color.a = 1.0
			marker.color.r = r
			marker.color.g = g
			marker.color.b = b
			# marker orientaiton
			marker.pose.orientation.x = 0.0
			marker.pose.orientation.y = 0.0
			marker.pose.orientation.z = 0.0
			marker.pose.orientation.w = 1.0
			# marker position
			marker.pose.position.x = pose.x
			marker.pose.position.y = pose.y
			marker.pose.position.z = pose.z

			t = rospy.Duration(0) 
			marker.lifetime = t
			markerArray.markers.append(marker)
		id = 0
		for m in markerArray.markers:
		   m.id = id
		   id += 1
		self.global_plan_marker=markerArray
		self.marker_route_pub.publish(self.global_plan_marker)
		self.global_plan_raw_pub.publish(self.global_plan_raw_msg)#GlobalPlan
		self.route_pub.publish(self.route_raw) #Path

		if not (self.track == Track.MAP or self.track == Track.SENSORS): #DATASET
			self.path_pub.publish(self.route_raw)
			self.path_ros_pub.publish(self.route_raw)


	def routeToWaypoints(self, route):
		stamp=rospy.Time().now()#self.stamp
		waypoints = []
		msg = GlobalPlan()
		msg.header.stamp = rospy.Time().now()#stamp#rospy.Time().now()
		msg.header.frame_id = 'map'
		msg.points = []
		msg.road_options = []
		# print(route)
		for option, pose in zip(route[0],route[1]):
			lat=pose.latitude
			lon=pose.longitude#way[0]['lon']
			z=pose.altitude#way.altitude#way[0]['z']
			utm_data = self.geodesicToMercator(self.lat_ref, self.lon_ref, lat, lon, z)

			p = Point()
			p.x = utm_data[1]
			p.y = utm_data[2]
			p.z = z
			msg.points.append(p)

			if option == RoadOption.LANEFOLLOW:
				msg.road_options.append(msg.LANEFOLLOW)
				road_opt=0
			elif option == RoadOption.LEFT:
				msg.road_options.append(msg.LEFT)
				road_opt=1
			elif option == RoadOption.RIGHT:
				msg.road_options.append(msg.RIGHT)
				road_opt=2
			elif option == RoadOption.STRAIGHT:
				msg.road_options.append(msg.STRAIGHT)
				road_opt=3
			elif option== RoadOption.CHANGELANELEFT:
				msg.road_options.append(msg.CHANGELANELEFT)
				road_opt=4
			elif option == RoadOption.CHANGELANERIGHT:
				msg.road_options.append(msg.CHANGELANERIGHT)
				road_opt=5
			else:
				msg.road_options.append(msg.UNKNOWN)
				road_opt=6
			waypoints.append([utm_data[1], utm_data[2], z, road_opt])
		waypoints = np.asarray(waypoints) 
		return np.asarray(waypoints), msg

	# def routeWorldToWaypoints(self, route):
	# 	stamp=rospy.Time().now()
	# 	waypoints = []
	# 	msg = GlobalPlan()
	# 	msg.header.stamp = stamp
	# 	msg.header.frame_id = 'map'
	# 	msg.points = []
	# 	msg.road_options = []
	# 	for option, pose in zip(route[0],route[1]):
	# 		p = Point()
	# 		p.x = pose.position.x
	# 		p.y = pose.position.y
	# 		p.z = pose.position.z
	# 		msg.points.append(p)
	# 		if option== RoadOption.LANEFOLLOW:
	# 			msg.road_options.append(msg.LANEFOLLOW)
	# 			road_opt=0
	# 		elif option == RoadOption.LEFT:
	# 			msg.road_options.append(msg.LEFT)
	# 			road_opt=1
	# 		elif option == RoadOption.RIGHT:
	# 			msg.road_options.append(msg.RIGHT)
	# 			road_opt=2
	# 		elif option== RoadOption.STRAIGHT:
	# 			msg.road_options.append(msg.STRAIGHT)
	# 			road_opt=3
	# 		elif option == RoadOption.CHANGELANELEFT:
	# 			msg.road_options.append(msg.CHANGELANELEFT)
	# 			road_opt=4
	# 		elif option== RoadOption.CHANGELANERIGHT:
	# 			msg.road_options.append(msg.CHANGELANERIGHT)
	# 			road_opt=5
	# 		else:
	# 			msg.road_options.append(msg.UNKNOWN)
	# 			road_opt=6
	# 		waypoints.append([p.x, p.y, p.z, road_opt])
	# 	waypoints = np.asarray(waypoints) 
	# 	return np.asarray(waypoints), msg

	def geodesicToMercator(self, lat_ref, lon_ref, lat, lon, z):
		# EARTH_RADIUS_EQUA = 6378137.0
		# scale = 1.0#np.cos(np.radians(lat))
		# x = scale * np.radians(lon) * EARTH_RADIUS_EQUA
		# y = scale * EARTH_RADIUS_EQUA * np.log(np.tan((90.0 + lat) * np.pi/ 360.0))
		"""
		Convert from GPS coordinates to world coordinates
		:param lat_ref: latitude reference for the current map
		:param lon_ref: longitude reference for the current map
		:param lat: lat to translate
		:param lat: lon to translate
		:return: dictionary with lat, lon and height
		"""
		EARTH_RADIUS_EQUA = 6378137.0
		scale = math.cos(lat_ref * math.pi / 180.0)

		mx_ref = scale * lon_ref * math.pi * EARTH_RADIUS_EQUA / 180.0
		my_ref = scale * EARTH_RADIUS_EQUA * math.log(math.tan((90.0 + lat_ref) * math.pi / 360.0))
		# mx += x
		# my -= y
		mx = scale * lon * math.pi * EARTH_RADIUS_EQUA / 180.0
		my = scale * EARTH_RADIUS_EQUA * math.log(math.tan((90.0 + lat) * math.pi / 360.0))
		x=mx-mx_ref
		y=my-my_ref
	    # return mx-mx_ref, my-my_ref, z
		# return in right hand coordinates
		return [0, y, -x]

	def gpsToWorld(self, gps):
		utm_data = self.geodesicToMercator( self.lat_ref, self.lon_ref, gps[0], gps[1], gps[2])
		# self.all_ll.append([gps[1], gps[0]]) 
		# np.save('/home/carla/ll_ll.npy', self.all_ll)
		return np.array([utm_data[1], utm_data[2], gps[2]])


	def publish_tf(self, event):


		#Dataset
		if self.track == Track.MAP or self.track == Track.SENSORS:  #---->
			
			tmsg = TransformStamped()
			tmsg.header.stamp = rospy.Time().now()#self.stamp#msg.header.stamp
			tmsg.header.frame_id = 'base_link'
			tmsg.child_frame_id = 'hero'
			tmsg.transform.translation.x = 0
			tmsg.transform.translation.y = 0
			tmsg.transform.translation.z = 0#0
			tmsg.transform.rotation.x = 0
			tmsg.transform.rotation.y = 0
			tmsg.transform.rotation.z = 0
			tmsg.transform.rotation.w = 1
			self.tf_br_baselink_hero.sendTransform(tmsg)
			if self.pose is not None:
				self.gps_pub.publish(self.pose)
			return

		if self.pose is None or self.yaw is None:
			return


		if self.last_pose is None:# and len(self.poseini) > 0:
			self.origin.pose.position.x = self.pose[0]#self.waypoints[0,0]
			self.origin.pose.position.y = self.pose[1]#self.waypoints[0,1]
			self.origin.pose.position.z = self.pose[2]
			self.last_pose = self.pose

		ori=tf.transformations.quaternion_from_euler(0, 0, self.yaw)

		pose_msg = PoseWithCovarianceStamped()
		pose_msg.header.stamp = rospy.Time().now()#self.stamp#msg.header.stamp
		pose_msg.header.frame_id = 'map'
		pose_msg.pose.pose.position.x = self.pose[0]#pose[0]#pose_ekf[0]
		pose_msg.pose.pose.position.y = self.pose[1]#pose[1]#pose_ekf[1]
		pose_msg.pose.pose.position.z = self.pose[2]#msg.altitude
		pose_msg.pose.pose.orientation.x = ori[0]#self.actual_imu.orientation.x#
		pose_msg.pose.pose.orientation.y = ori[1]#self.actual_imu.orientation.y#x
		pose_msg.pose.pose.orientation.z = ori[2]#self.actual_imu.orientation.z#
		pose_msg.pose.pose.orientation.w = ori[3]#self.actual_imu.orientation.w#

		error_gps = self.gpsToWorld([0.000005, 0.000005, 0.000005])

		pose_msg.pose.covariance=[error_gps[0], 0.0, 0.0, 0.0, 0.0, 0.0,
												0.0, error_gps[0], 0.0, 0.0, 0.0, 0.0,
												0.0, 0.0, error_gps[0], 0.0, 0.0, 0.0,
												0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
												0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
												0.0, 0.0, 0.0, 0.0, 0.0, 0.0
												]	
		self.pose_msg=pose_msg
		self.gps_pub.publish(pose_msg)
		# hero_2_gps_transf = self.tf2_buffer_ẁorld2velodyne.lookup_transform('hero/GPS', 'hero', rospy.Time())
		# x_h2gps=hero_2_gps_transf.transform.translation.x
		# y_h2gps=hero_2_gps_transf.transform.translation.y
		# z_h2gps=hero_2_gps_transf.transform.translation.z
		if self.track == Track.MAP or self.track == Track.SENSORS:  #---->
			return

		#Dataset
		odom_msg = Odometry()
		odom_msg.header.stamp = rospy.Time().now()#self.stamp#msg.header.stamp
		odom_msg.header.frame_id ='odom'#'map'#'odom'
		odom_msg.child_frame_id = 'base_link'#'gps'#'base_link'#'gps'#'base_link'
		odom_msg.pose.pose.position.x = self.pose[0]-self.origin.pose.position.x#self.pose[0]-y_h2gps - self.origin.pose.position.x#pose_ekf[0] - self.origin.pose.position.x #+ 1.0#pose[0]
		odom_msg.pose.pose.position.y = self.pose[1]-self.origin.pose.position.y#self.pose[1]-x_h2gps - self.origin.pose.position.y#pose_ekf[1] - self.origin.pose.position.y
		odom_msg.pose.pose.position.z = self.pose[2]-self.origin.pose.position.z#self.pose[2]-z_h2gps - self.origin.pose.position.z#0
		odom_msg.pose.pose.orientation.x = ori[0]#self.actual_imu.orientation.x#
		odom_msg.pose.pose.orientation.y = ori[1]#self.actual_imu.orientation.y#
		odom_msg.pose.pose.orientation.z = ori[2]#self.actual_imu.orientation.z#
		odom_msg.pose.pose.orientation.w = ori[3]#self.actual_imu.orientation.w#
		self.odometry_pub.publish(odom_msg)

		tmsg = TransformStamped()
		tmsg.header.stamp = rospy.Time().now()#self.stamp#msg.header.stamp
		tmsg.header.frame_id = 'base_link'
		tmsg.child_frame_id = 'hero'
		tmsg.transform.translation.x = 0
		tmsg.transform.translation.y = 0
		tmsg.transform.translation.z = 0#0
		tmsg.transform.rotation.x = 0
		tmsg.transform.rotation.y = 0
		tmsg.transform.rotation.z = 0
		tmsg.transform.rotation.w = 1
		self.tf_br_baselink_hero.sendTransform(tmsg)

		tmsg = TransformStamped()
		tmsg.header.stamp = rospy.Time().now()#self.stamp#msg.header.stamp
		tmsg.header.frame_id = 'odom'
		tmsg.child_frame_id = 'base_link'
		tmsg.transform.translation.x = odom_msg.pose.pose.position.x
		tmsg.transform.translation.y = odom_msg.pose.pose.position.y
		tmsg.transform.translation.z = odom_msg.pose.pose.position.z#0
		tmsg.transform.rotation.x = odom_msg.pose.pose.orientation.x
		tmsg.transform.rotation.y = odom_msg.pose.pose.orientation.y
		tmsg.transform.rotation.z = odom_msg.pose.pose.orientation.z
		tmsg.transform.rotation.w = odom_msg.pose.pose.orientation.w
		self.tf_br_odom_baselink.sendTransform(tmsg)

		tmsg = TransformStamped()
		tmsg.header.stamp = rospy.Time().now()#self.stamp#msg.header.stamp
		tmsg.header.frame_id = 'map'
		tmsg.child_frame_id = 'odom'
		tmsg.transform.translation.x = self.origin.pose.position.x #+ 1.0
		tmsg.transform.translation.y = self.origin.pose.position.y
		tmsg.transform.translation.z = self.origin.pose.position.z
		tmsg.transform.rotation.x = 0#self.origin.pose.orientation.x#0.0#
		tmsg.transform.rotation.y = 0#self.origin.pose.orientation.y#0.0#
		tmsg.transform.rotation.z = 0#self.origin.pose.orientation.z#0.0#
		tmsg.transform.rotation.w = 1#self.origin.pose.orientation.w#1.0#
		self.tf_br_map_odom.sendTransform(tmsg)


	def publish_plan_img(self):
	# 	print('publish_plan_img: ')

	# 	scale=8
	# 	size_image=700

	# 	try:
	# 		trans_stamp_inv= self.tf2_buffer_map2vel.lookup_transform('velodyne','map', rospy.Time())
	# 	except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as exept:
	# 		print(exept)

	# 	stamp = rospy.Time().now()
	# 	try:
	# 		trans_stamp_inv_stereo_to_map = self.tf2_buffer_map2stereo.lookup_transform('stereo','map', rospy.Time())
	# 	except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as exept:
	# 		print(exept)


	# 	if self.rgb_image is None:
	# 		return

	# 	global_plan_points_png= np.zeros([size_image,size_image,3],dtype=np.uint8)
	# 	global_plan_line_png= np.zeros([size_image,size_image,1],dtype=np.uint8)


	# 	front_global_plan_points_png= np.zeros([self.rgb_image.shape[0], self.rgb_image.shape[1], 3], dtype=np.uint8)
	# 	front_global_plan_points_png_depth= np.zeros([ self.rgb_image.shape[0], self.rgb_image.shape[1], 1], dtype=np.uint16)
	# 	front_global_plan_line_png= np.zeros([self.rgb_image.shape[0], self.rgb_image.shape[1], 1], dtype=np.uint8)


	# 	raw_path_points=[]
		color_plan=[]
	# 	raw_points_uv=[]
	# 	# print('len global plan ',len(global_plan.points))
	# 	# print(global_plan.points)
	# 	# if self.visited_points_global_plan ==[]
	# 	# else:
	# 	# 	self.visited_points_global_plan.append[next_index_goal_plan]

		for index, (pose, option) in enumerate(zip(self.global_plan_raw_msg.points, self.global_plan_raw_msg.road_options)):

			# old_pose = PoseStamped()
			# old_pose.header.frame_id='map'
			# old_pose.header.stamp = stamp
			# old_pose.pose.position = pose

			# new_pose = tf2_geometry_msgs.do_transform_pose(old_pose, trans_stamp_inv)
			# raw_path_points.append([new_pose.pose.position.y*scale+(size_image/2), new_pose.pose.position.x*scale+(size_image/3)])

			# new_pose_map_to_stereo = tf2_geometry_msgs.do_transform_pose(old_pose, trans_stamp_inv_stereo_to_map)
			# # print('new_pose_map_to_stereo ',new_pose_map_to_stereo)
			# camera = PinholeCameraModel()
			# camera.fromCameraInfo(self.camera_info_left)
			# p_image = camera.project3dToPixel((new_pose_map_to_stereo.pose.position.x,  new_pose_map_to_stereo.pose.position.y,  new_pose_map_to_stereo.pose.position.z))
			# # print(p_image)
			# projected=False
			# if new_pose_map_to_stereo.pose.position.z>0:
			# 	projected=True
			# raw_points_uv.append((p_image, projected, new_pose_map_to_stereo.pose.position.z))

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
														np.array([self.pose_msg.pose.pose.position.x, self.pose_msg.pose.pose.position.y]))#dist between ego and the next goal
			if dist_to_goal<7.:  ## if the ego achieve the goal
				self.next_index_goal_plan=index#self.next_index_goal_plan+1  #next goal
				self.option_string_dataset=self.option_string
				dist_to_near_option=dist_to_goal
				self.pose_achieved_x=pose.x
				self.pose_achieved_y=pose.y



		if (self.option_string_dataset=='changelaneleft' or self.option_string_dataset=='changelaneright') :
			# if self.pose_achieved_y is not None:
			dist_to_pose_achieved =np.linalg.norm(np.array([self.pose_achieved_x, self.pose_achieved_y]) - 
														np.array([self.pose_msg.pose.pose.position.x, self.pose_msg.pose.pose.position.y]))
			if dist_to_pose_achieved>7.:
				self.option_string_dataset='lanefollow'

		# print('self.option_string_dataset: ',self.option_string_dataset)
		self.pub_option_lane.publish(self.option_string_dataset)
	# 	# print('next_index_goal_plan: ', self.next_index_goal_plan, global_plan.road_options[self.next_index_goal_plan])
	# 	# global_plan.road_options

	# 		# print('saved lanefollow')
	# 	init_interval=self.next_index_goal_plan-5
	# 	end_interval=self.next_index_goal_plan+5

	# 	if init_interval<3:
	# 		init_interval=0
	# 	if end_interval>len(raw_path_points):
	# 		end_interval=len(raw_path_points)

	# 	raw_local_points=[]
	# 	for i in range(init_interval, end_interval):
	# 		raw_local_points.append(raw_path_points[i])
	# 		color_point=color_plan[i]
	# 		point_local=raw_path_points[i]
	# 		# plan_local
	# 		# cv2.circle(global_plan_points_png, (int(pose.pose.position.y*scale+(size_image/2)), 
	# 		# 													int(pose.pose.position.x*scale+(size_image/3))), 10, color, thickness)
	# 		# cv2.circle(birdeyeview_stereo, (int(point_local[0]), int(point_local[1])), 10, color_point, thickness)
	# 		cv2.circle(global_plan_points_png, (int(point_local[0]), int(point_local[1])), 10, color_point, thickness)

	# 	init_interval=self.next_index_goal_plan-2
	# 	end_interval=self.next_index_goal_plan+3

	# 	if init_interval<0:
	# 		init_interval=0
	# 	if end_interval>len(raw_path_points):
	# 		end_interval=len(raw_path_points)


	# 	raw_local_points_uv=[]
	# 	raw_local_points_uv.append((int(self.rgb_image.shape[1]/2), self.rgb_image.shape[0]))
	# 	for i in range(init_interval, end_interval):
	# 		# print('projected ',raw_points_uv[i][1])

	# 		if raw_points_uv[i][1]: #points cam ahead true or false
	# 			u = raw_points_uv[i][0][0]
	# 			v = raw_points_uv[i][0][1]

	# 			if raw_points_uv[i][0][1]>0  and  raw_points_uv[i][0][1]<self.rgb_image.shape[0]:#  and raw_points_uv[i][0][0]>0  and  raw_points_uv[i][0][0]<1600 :
	# 				# print(raw_points_uv[i][0][0])
	# 				if raw_points_uv[i][0][0]<0:
	# 					u=0

	# 				if raw_points_uv[i][0][0]>self.rgb_image.shape[1]:
	# 					u=self.rgb_image.shape[1]

	# 				raw_local_points_uv.append((u,v))

	# 				color_point=color_plan[i]
	# 				# point_uv_local=()raw_points_uv[i][0]
	# 				# print('point_uv_local ',point_uv_local)
	# 				# plan_local
	# 				# cv2.circle(global_plan_points_png, (int(pose.pose.position.y*scale+(size_image/2)), 
	# 				# 													int(pose.pose.position.x*scale+(size_image/3))), 10, color, thickness)
	# 				if u>0 and u<self.rgb_image.shape[1]  and raw_points_uv[i][2]>0:
	# 					# cv2.circle(rgb_image, (int(u), int(v)), 15, color_point, thickness)
	# 					cv2.circle(front_global_plan_points_png, (int(u), int(v)), int(500/raw_points_uv[i][2]), color_point, thickness)
	# 					cv2.circle(front_global_plan_points_png_depth, (int(u), int(v)), int(500/raw_points_uv[i][2]), (int(1000*raw_points_uv[i][2])), thickness)
						
	# 					# cv2.circle(global_plan_points_png, (int(point_local[0]), int(point_local[1])), 10, color_point, thickness)
	# 	# im_color_depth = cv2.applyColorMap(front_global_plan_points_png_depth, cv2.COLORMAP_JET)
	# 	# im_color_depth = cv2.bitwise_and(im_color_depth, im_color_depth, mask=front_global_plan_points_png_depth)
	# 	# radius=3
	# 	# color =(0,255,0)


	# 	thickness=10

	# 	# pts = np.array(raw_path_points, np.int32)
	# 	# pts = pts.reshape((-1,1,2))
	# 	# # cv2.polylines(points_traj_past,[pts],False,(255,0,0))			
	# 	# # cv2.polylines(birdeyeview_stereo,[pts],False,(0,255,0),thickness)	
	# 	# cv2.polylines(global_plan_line_png,[pts],False,(255),thickness)	

	# 	pts = np.array(raw_local_points, np.int32)
	# 	pts = pts.reshape((-1,1,2))
	# 	# cv2.polylines(points_traj_past,[pts],False,(255,0,0))			
	# 	# cv2.polylines(birdeyeview_stereo,[pts],False,(0,255,0),thickness)	
	# 	cv2.polylines(global_plan_line_png,[pts],False,(255),thickness)	





	# 	# print(raw_local_points_uv)
	# 	raw_local_points_uv_rev=raw_local_points_uv[::-1]
	# 	# print(raw_local_points_uv_rev)
	# 	pts = np.array(raw_local_points_uv_rev, np.int32)
	# 	# print('pts ',pts)

	# 	pts = pts.reshape((-1,1,2))
	# 	# print(pts)
	# 	# cv2.polylines(points_traj_past,[pts],False,(255,0,0))			
	# 	# cv2.polylines(birdeyeview_stereo,[pts],False,(0,255,0),thickness)	
	# 	# cv2.polylines(rgb_image,[pts],False,(0,255,0),thickness)	
	# 	cv2.polylines(front_global_plan_line_png,[pts],False,(255),thickness)	







	# 	scale=8
	# 	size_image=700




	# 	print('last tr poses ')

	# 	try:
	# 		trans_stamp_inv= self.tf2_buffer_map2vel.lookup_transform('velodyne','map', rospy.Time())
	# 	except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as exept:
	# 		print(exept)

	# 	# def pose_gps_cb(self,msg):
	# 	current_pose_gps = self.pose_msg
	# 	# current_pose = self.pose_msg

	# 	if self.old_poses_list_gps==None:
	# 		self.old_poses_list_gps = [[current_pose_gps,0] for i in range(200)]
	# 	else:

	# 		dist=np.linalg.norm(np.array([self.pose_msg.pose.pose.position.x,self.pose_msg.pose.pose.position.y]) - np.array([ self.old_poses_list_gps[0][0].pose.pose.position.x, self.old_poses_list_gps[0][0].pose.pose.position.y]))

	# 		if dist>0.2:

	# 			deque_old_poses_list_gps = deque(self.old_poses_list_gps) 
	# 			deque_old_poses_list_gps.rotate(1) 
	# 			self.old_poses_list_gps = list(deque_old_poses_list_gps)
	# 			self.old_poses_list_gps[0] = [current_pose_gps,dist]


	# 	gps_backward_png= np.zeros([size_image,size_image,1],dtype=np.uint8)
	# 	img_map_mask= np.zeros([size_image,size_image,1],dtype=np.uint8)

	# 	poses_back=[]

	# 	for p, d in self.old_poses_list_gps:

	# 		new_tfmed_gps = tf2_geometry_msgs.do_transform_pose(p.pose, trans_stamp_inv)
	# 		poses_back.append( [(new_tfmed_gps.pose.position.y*scale)+(size_image/2), (new_tfmed_gps.pose.position.x*scale)+(size_image/3)] )
	# 		x = (new_tfmed_gps.pose.position.y*scale)+(size_image/2)
	# 		y = (new_tfmed_gps.pose.position.x*scale)+(size_image/3)

	# 		x0=(size_image/2)
	# 		y0=(size_image/3)


	# 		dist_to_p=np.linalg.norm(np.array([x,y]) - np.array([x0,y0]))


	# 		print(x,y,dist_to_p)
	# 		cv2.circle(gps_backward_png, (int(x), int(y)), 10, int(255-dist_to_p), thickness=-1)
	# 		cv2.circle(img_map_mask, (int(x), int(y)), 10, 255, thickness=-1)


	# 	ret,img_map_mask = cv2.threshold(img_map_mask,1,255,cv2.THRESH_BINARY)
	# 	gps_backward_color_png=cv2.applyColorMap(gps_backward_png, cv2.COLORMAP_RAINBOW)
	# 	gps_backward_color_png = cv2.bitwise_and(gps_backward_color_png,gps_backward_color_png, mask= img_map_mask)


	# 	# pts = np.array(poses_back, np.int32)
	# 	# pts = pts.reshape((-1,1,2))
	# 	# cv2.polylines(points_traj_past,[pts],False,(0,255,255))
	# 	# cv2.polylines(birdeyeview_stereo,[pts],False,(0,255,255))
	# 	# cv2.polylines(gps_backward_png,[pts],False,(255),thickness)

	# 	print('last 100 poses ')
	# 	if self.pub_gps_backward_color_png.get_num_connections() != 0:
	# 		try:
	# 			self.pub_gps_backward_color_png.publish(self.cvbridge.cv2_to_imgmsg(gps_backward_color_png, "bgr8"))
	# 		except CvBridgeError as e:
	# 			print (e)

	# 	if self.pub_gps_backward_png.get_num_connections() != 0:
	# 		try:
	# 			self.pub_gps_backward_png.publish(self.cvbridge.cv2_to_imgmsg(gps_backward_png, "mono8"))
	# 		except CvBridgeError as e:
	# 			print (e)

	# 	if self.pub_global_plan_img.get_num_connections() != 0:
	# 		try:
	# 			self.pub_global_plan_img.publish(self.cvbridge.cv2_to_imgmsg(global_plan_points_png, "bgr8"))
	# 		except CvBridgeError as e:
	# 			print (e)

	# 	if self.pub_global_plan_line_img.get_num_connections() != 0:
	# 		try:
	# 			self.pub_global_plan_line_img.publish(self.cvbridge.cv2_to_imgmsg(global_plan_line_png, "mono8"))
	# 		except CvBridgeError as e:
	# 			print (e)
	# 	if self.pub_front_global_plan_img.get_num_connections() != 0:
	# 		try:
	# 			self.pub_front_global_plan_img.publish(self.cvbridge.cv2_to_imgmsg(front_global_plan_points_png, "bgr8"))
	# 		except CvBridgeError as e:
	# 			print (e)

	# 	if self.pub_front_global_plan_line_img.get_num_connections() != 0:
	# 		try:
	# 			self.pub_front_global_plan_line_img.publish(self.cvbridge.cv2_to_imgmsg(front_global_plan_line_png, "mono8"))
	# 		except CvBridgeError as e:
	# 			print (e)

	# 	if self.pub_front_global_plan_depth.get_num_connections() != 0:
	# 		try:
	# 			self.pub_front_global_plan_depth.publish(self.cvbridge.cv2_to_imgmsg(front_global_plan_points_png_depth, "mono16"))
	# 		except CvBridgeError as e:
	# 			print (e)





	def shutdown_cb(self, msg):
		if msg.data:
			
			del self.gps_pub
			del self.lidar_pub_front
			del self.vehicle_state_pub
			del self.route_pub
			del self.shutdown_sub
			del self.odometry_pub
			del self.global_plan_raw_pub 

			del self.imu_pub 
			del self.imu_gt_pub 
			del self.nav_sat_fix_pub 
			
			del self.img_left_pub 
			del self.img_left_rect_color_pub 
			del self.img_left_info_pub 
			del self.img_right_pub 
			del self.img_right_rect_color_pub 
			del self.img_right_info_pub 

			del self.control_carla_pub 

			del self.marker_route_pub 

			del self.throttle_sub 
			del self.brake_sub 
			del self.steer_sub 
			del self.hand_brake_sub 

			del self.caml_sub
			del self.camr_sub
			del self.caminfol_sub
			del self.caminfor_sub
			del self.gps_sub
			del self.speedometer_sub
			del self.globalp_sub
			del self.lidar_sub
			del self.imu_sub

			if self.track == Track.MAP or not (self.track == Track.MAP or self.track == Track.SENSORS):#'DATASET':
				del self.opendrive_sub
				# if self.track == Track.MAP:
				del self.hdmap_pub
			if self.track == Track.MAP or self.track == Track.SENSORS:  #---->
				del self.pose_corrected_sub
			else: #self.track = 'DATASET'
				if self.create_dataset_depth:
					pass
				if self.create_dataset_images:
					pass
				if self.create_occupancy_grid:
					del self.camera_aerial_sub
					del self.occupation_map_pub 
				if self.create_dataset_obstacles:
					del self.objects_sub
					del self.tfl_status_sub
					del self.tfl_info_sub
					del self.obj_info_sub

					del self.pub_marker_obstacles 
					del self.pub_obstacles 

					del self.obstacle_tf_red_array_pub
					del self.marker_tf_red_array_pub
					del self.poses_tfl_publisher

				if self.create_dataset_path:
					del self.path_pub 
					del self.path_ros_pub
			print ("Bye!")
			time.sleep(2.0)
			rospy.signal_shutdown("carina_bridge_node finished ...")

if __name__ == '__main__':
	rospy.init_node("carina_bridge_node",anonymous=True)
	print("[CarinaBridge node] running...")
	repl = CarinaBridge()
	rospy.spin()
