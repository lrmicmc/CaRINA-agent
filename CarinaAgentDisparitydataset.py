#!/usr/bin/env python3
# import subprocess
# process = subprocess.Popen(['/bin/bash', '-c', "source /home/luis/carla/TRACK/scenario_runner_t2/team_code/catkin_ws/devel/setup.bash"], stderr=subprocess.STDOUT, stdout=subprocess.PIPE)
# # process.wait()
# stdout, stderr = process.communicate()
# if process.returncode:
# 	print(stdout, stderr)
# 	raise RuntimeError("Could not set source ros")
# os.system("bash -c source '/home/luis/carla/TRACK/scenario_runner_t2/team_code/catkin_ws/devel/setup.bash'" )
import os
import scipy.misc

import subprocess



import carla
import numpy as np
import matplotlib.pyplot as plt
import time
import sys
from threading import Thread
from leaderboard.autoagents.autonomous_agent import AutonomousAgent, Track
from agents.navigation.local_planner import RoadOption
#from LatLongUTMconversion import *
from ekf_filter import *

from sensor_msgs.msg import Imu
from geometry_msgs.msg import Vector3 

#ros
import rospy
import tf
import tf2_ros
# import utm


#messages
from geometry_msgs.msg import PoseWithCovarianceStamped, PoseStamped
from std_msgs.msg import Float64, Header, Bool
from sensor_msgs.msg import Image, PointCloud2, CameraInfo
from msgs_action.msg import VehicleState, Throttle, Brake, SteeringAngle
from nav_msgs.msg import Path, Odometry
from geometry_msgs.msg import TransformStamped
from rosgraph_msgs.msg import Clock
from msgs_navigation.msg import GlobalPlan
from geometry_msgs.msg import Point
from msgs_mapping.msg import HDMap as HDMapMsg

# from msgs_agent_bridge.msg import Object, ObjectArray
from msgs_perception.msg import Obstacle
from msgs_perception.msg import ObstacleArray
from visualization_msgs.msg import Marker
from visualization_msgs.msg import MarkerArray
from shapely.geometry import LineString

from geometry_msgs.msg import Vector3

from sensor_msgs.msg import NavSatFix

#pointcloud
from sensor_msgs.point_cloud2 import create_cloud_xyz32
import json
import xml.etree.ElementTree as ET

import math


from lane_level import LaneLevel
from road_network import RoadNetwork

import cv2
# import mmcv
# from mmflow.apis import inference_model, init_model
from mmflow.datasets import visualize_flow, write_flow, read_flow


def get_entry_point():
	return 'CarinaAgent'

class CarinaAgent(AutonomousAgent):
	def setup(self, path_to_conf_file):

		#collect dataset
		self.create_dataset=False
		create_dataset_str = os.environ.get('CREATE_DATASET')
		if create_dataset_str=='True':
			print('create_dataset_str',create_dataset_str)
			self.create_dataset=True


		#self.create_dataset=True

		# Set environment variables
		# os.environ['API_USER'] = 'username'
		# os.environ['API_PASSWORD'] = 'secret'
			
		# track = os.getenv('CHALLENGE_TRACK_CODENAME')
		# print(track)

		track_env = os.environ.get('CHALLENGE_TRACK_CODENAME')
		# print(Track.MAP)

		self.track = Track.MAP

		if track_env=='SENSORS':
			self.track = Track.SENSORS
		print(track_env)
		print(self.track)


		#clock
		self.start_time = time.time()
		self.last_time = 0.0
		self.fps = 20.0

		#running nodes
		command_path='/'
		for c in os.path.realpath(__file__).split('/')[0:-1]:
			command_path = os.path.join(command_path, c)
		command_path=os.path.join(command_path, 'execute_carina.sh')

		os.system("bash " + command_path)

		rospy.set_param("/use_sim_time", True)


  #       # set use_sim_time via commandline before init-node
		# process = subprocess.Popen("rosparam set use_sim_time true", shell=True, stderr=subprocess.STDOUT, stdout=subprocess.PIPE)
		# process.wait()
		# if process.returncode:
		# 	raise RuntimeError("Could not set use_sim_time")

		rospy.init_node('carina_agent_node', anonymous=True)

		self.clock_ini = rospy.Time.now()#rospy.Time()
		# publish first clock value '0'
		self.clock_publisher = rospy.Publisher('clock', Clock, queue_size=10, latch=True)
		# self.clock_publisher.publish(Clock(rospy.Time.from_sec(0)))
		self.clock_publisher.publish(Clock(self.clock_ini))
		self.stamp=None

		self.speed_temp=0.0








		#route
		self.convert_route_to_way = False               
		self.route = Path()
		self.global_plan_msg = GlobalPlan()
		self.route.header.stamp = rospy.Time().now()
		self.publis_gt_waypoints = True
		#control
		self.throttle = 0.0
		self.brake = 0
		self.steering_angle = 0.0
		self.max_steering = np.deg2rad(30.)
		self.hand_brake = False
		
		#gps
		self.pose = None
		self.last_pose = None
		self.last_pose_gt = None

		self.last_gps = None
		self.last_gps_gt = None

		self.last_yaw = 0.0
		self.last_yaw_gt = 0.0

		self.last_gps_index = -1.0
		self.last_gps_gt_index = -1.0

		self.first_tick = True

		self.first_y = 0.0
		self.ekf_gps = EKF_filter()
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
		self.all_points_gps = []
		self.all_points_ekf = []
		self.all_ll = []

		#sensors
		self.update_sensors_running = True
		self.last_image_index = -1.0
		self.last_pc_index = -1.0
		# self.last_radar_index = -1.0

		self.queue_pc_front = []
		self.queue_pc_back = []
		self.thread_camera = Thread(target=self.update_cameras)
		#self.thread_camera.start()
		self.thread_lidar = Thread(target=self.update_lidar)
		#self.thread_lidar.start()
		self.thread_map = Thread(target=self.update_map)

		#hd map
		self.last_hdmap_frame = 0

		self.tree=None
		self.OpenDRIVE_loaded=False

		self.half_pc_old =None
		#publisher
		self.gps_pub = rospy.Publisher('/carina/localization/pose', PoseWithCovarianceStamped, queue_size=1)
		self.gps_gt_pub = rospy.Publisher('/carina/localization/pose_gt', PoseWithCovarianceStamped, queue_size=1)

		self.lidar_pub_front = rospy.Publisher('/carina/sensor/lidar/front/point_cloud', PointCloud2, queue_size=1)


		# self.pub_marker_radar = rospy.Publisher('/carina/perception/radar/obstacles_marker_array', MarkerArray, queue_size=1)
		# self.pub_obj_radar = rospy.Publisher('/carina/perception/radar/front/obstacles_array', ObstacleArray, queue_size=1)


		self.lidar_pub_back = rospy.Publisher('/carina/sensor/lidar/back/point_cloud', PointCloud2, queue_size=1)
		self.vehicle_state_pub = rospy.Publisher('/carina/vehicle/state', VehicleState, queue_size=1)
		self.route_pub = rospy.Publisher('/carina/navigation/waypoints', Path, queue_size=1)
		self.route_vis_pub = rospy.Publisher('/carina/navigation/waypoints_vis', Path, queue_size=1)

		self.route_raw_pub = rospy.Publisher('/carina/navigation/waypoints_raw', Path, queue_size=1)

		self.shutdown_pub = rospy.Publisher('/carina/vehicle/shutdown', Bool, queue_size=1)
		self.shutdown_using_map = rospy.Publisher('/carina/map/shutdown_using_map', Bool, queue_size=1)		# self.radar_pub_front = rospy.Publisher('/carina/sensor/radar/front/obstacles_array', ObstacleArray, queue_size=1)


		self.odometry_pub = rospy.Publisher('/carina/localization/odom', Odometry, queue_size=1)
		self.global_plan_pub = rospy.Publisher('/carina/navigation/global_plan', GlobalPlan, queue_size=1)
		self.global_plan_raw_pub = rospy.Publisher('/carina/navigation/global_raw_plan', GlobalPlan, queue_size=1)

		self.hdmap_pub = rospy.Publisher('/carina/map/hdmap', HDMapMsg, queue_size=1)
		self.clock_pub = rospy.Publisher('/clock', Clock, queue_size=1)
		self.imu_pub = rospy.Publisher('/carina/sensor/imu', Imu, queue_size=1)
		self.imu_gt_pub = rospy.Publisher('/carina/sensor/imu_gt', Imu, queue_size=1)


		self.nav_sat_fix_pub = rospy.Publisher('/carina/localization/nav_sat_fix', NavSatFix, queue_size=1)
		self.nav_sat_fix_gt_pub = rospy.Publisher('/carina/localization/nav_sat_fix_gt', NavSatFix, queue_size=1)


		self.tf_br_odom_baselink = tf2_ros.TransformBroadcaster()
		self.tf_br_map_odom = tf2_ros.TransformBroadcaster()
		self.tf_br_hdmap_odom = tf2_ros.TransformBroadcaster()


		#pub cameras
		self.img_left_pub = rospy.Publisher('/carina/sensor/camera/left/image_raw', Image, queue_size=1)
		self.img_left_rect_color_pub = rospy.Publisher('/carina/sensor/camera/left/image_rect_color', Image, queue_size=1)
		self.img_left_info_pub = rospy.Publisher('/carina/sensor/camera/left/camera_info', CameraInfo,queue_size=1)

		self.img_depth_pub = rospy.Publisher('/carina/sensor/camera/left/depth/image_raw', Image, queue_size=1)
		self.img_depth_info_pub = rospy.Publisher('/carina/sensor/camera/left/depth/camera_info', CameraInfo,queue_size=1)

		self.img_depth_r_pub = rospy.Publisher('/carina/sensor/camera/left/depth/image_raw', Image, queue_size=1)
		self.img_depth_r_info_pub = rospy.Publisher('/carina/sensor/camera/left/depth/camera_info', CameraInfo,queue_size=1)

		self.img_sem_pub = rospy.Publisher('/carina/sensor/camera/left/sem/image_raw', Image, queue_size=1)
		self.img_sem_info_pub = rospy.Publisher('/carina/sensor/camera/left/sem/camera_info', CameraInfo,queue_size=1)

		self.img_right_pub = rospy.Publisher('/carina/sensor/camera/right/image_raw', Image, queue_size=1)
		self.img_right_rect_color_pub = rospy.Publisher('/carina/sensor/camera/right/image_rect_color', Image, queue_size=1)
		self.img_right_info_pub = rospy.Publisher('/carina/sensor/camera/right/camera_info', CameraInfo,queue_size=1)

		#subscriber
		self.throttle_sub = rospy.Subscriber('/carina/control/throttle_cmd', Throttle, self.throttle_cb)
		self.brake_sub = rospy.Subscriber('/carina/control/brake_cmd', Brake, self.brake_cb)
		self.steer_sub = rospy.Subscriber('/carina/control/steer_cmd', SteeringAngle, self.steer_cb)
		self.hand_brake_sub = rospy.Subscriber('/carina/control/hand_brake_cmd', Bool, self.hand_brake_cb)

		self.C_min = 5.944748225238571e-04
		self.C_max = 10.237604306862353e+03
		self.C_L=1.
		self.c_L=1.		
		self.K_max = 1.196835466593837e+02		
		self.sL = 1.223430214110182e+02
		self.Delta_sL = 0.002660093525200

		# self.len_lines = 2#len(lines)
		# self.end_list_float = self.len_lines/2
		# self.end_list = int(end_list_float)
		# self.x_L = [0]*end_list
		# self.y_L = [0]*end_list

		self.c_L = 1.
		# x_L = (lines[0::2])
		# y_L = (lines[1::2])
		self.fov=None #cameras fov
		self.baseline=None

		self.pose_a=[0,0]
		self.pose_back=[0,0]


	def sensors(self):
		
		# sensors = [
		# 			 {'type': 'sensor.camera.rgb', 'x':-0.5, 'y':0.0, 'z':2.8, 'roll':0.0, 'pitch':0.0, 'yaw':0.0,
		# 				'width':1600, 'height': 1600, 'fov':75, 'sensor_tick': 0.0, 'id': 'CameraLeft'},
		# 			 {'type': 'sensor.camera.rgb', 'x':-0.5, 'y':self.baseline, 'z':2.8, 'roll':0.0, 'pitch':0.0, 'yaw':0.0,
		# 				'width':1600, 'height': 1600, 'fov':75, 'sensor_tick': 0.0, 'id': 'CameraRight'},

		# 			 {'type': 'sensor.camera.semantic_segmentation', 'x':-0.5, 'y':0.0, 'z':2.8, 'roll':0.0, 'pitch':0.0, 'yaw':0.0,
		# 				'width':1600, 'height': 1600, 'fov':75, 'sensor_tick': 0.0, 'id': 'CameraSemantic'},

		# 			 {'type': 'sensor.lidar.ray_cast', 'x': 0.25, 'y': 0.0, 'z': 2.0, 'roll': 0.0, 'pitch': 0.0,
		# 				'yaw': 0.0, 'channels':64,'points_per_second':350000, 'upper_fov':5.0,
		# 				 'lower_fov':-20.0, 'range': 10000, 'sensor_tick': 0.0, 'rotation_frequency':30.0, 'id': 'LIDAR'},

		# 			 {'type': 'sensor.other.imu', 'x': -1.45, 'y': 0.0, 'z': 2.0, 'roll': 0.0, 'pitch': 0.0, 'yaw': 0.0, 'id': 'IMU'},
		# 			 {'type': 'sensor.other.imu', 'x': -1.45, 'y': 0.0, 'z': 2.0, 'roll': 0.0, 'pitch': 0.0, 'yaw': 0.0, 'id': 'imu_gt'},

		# 			 #{'type': 'sensor.other.gnss', 'x': 0.0, 'y': 0.0, 'z': 2.0, 'reading_frequency': 20, 'id': 'GPS_front'},
		# 			 {'type': 'sensor.other.gnss', 'x': -1.45, 'y': 0.0, 'z': 2.0, 'reading_frequency': 20, 'id': 'GPS_back'},
		# 			 {'type': 'sensor.other.gnss', 'x': -1.45, 'y': 0.0, 'z': 2.0, 'reading_frequency': 20, 'id': 'gps_gt'},


  #     #       		 {'type': 'sensor.other.radar', 'x': 2.7, 'y': 0.0, 'z': 0.7, 'roll': 0.0, 'pitch': 0.0,
		# 				# 'yaw': 0.0,  'fov': 30, 'id': 'RADAR'},
		# 			 {'type': 'sensor.opendrive_map', 'reading_frequency': 1, 'id': 'OpenDRIVE'},
		# 			 {'type': 'sensor.speedometer',  'reading_frequency': 20, 'id': 'SPEED'}                             
		# 			 # {'type': 'sensor.hd_map', 'reading_frequency': 10, 'id': 'hdmap'}
		# 		  ]

		# print(sensors)

		sensors = []

		if self.track == Track.MAP:
			OpenDRIVE={'type': 'sensor.opendrive_map', 'reading_frequency': 1, 'id': 'OpenDRIVE'}
			sensors.append(OpenDRIVE)

		# self.fov=91#cameras fov
		self.fov=30.6#cameras fov  argo
		# self.baseline=0.24
		self.baseline=0.2986      #argo


		# CameraLeft={'type': 'sensor.camera.rgb', 'x':0.25, 'y':0.0, 'z':2.8, 'roll':0.0, 'pitch':0.0, 'yaw':0.0,
		# 				'width':1600, 'height': 1600, 'fov':self.fov, 'sensor_tick': 0.0, 'id': 'CameraLeft'}
		# sensors.append(CameraLeft)	
		# CameraRight={'type': 'sensor.camera.rgb', 'x':0.25, 'y':self.baseline, 'z':2.8, 'roll':0.0, 'pitch':0.0, 'yaw':0.0,
		# 				'width':1600, 'height': 1600, 'fov':self.fov, 'sensor_tick': 0.0, 'id': 'CameraRight'}
		# sensors.append(CameraRight)
		CameraLeft={'type': 'sensor.camera.rgb', 'x':0.25, 'y':0.0, 'z':1.7, 'roll':0.0, 'pitch':0.0, 'yaw':0.0,
						'width':2464, 'height': 2056, 'fov':self.fov, 'sensor_tick': 0.0, 'id': 'CameraLeft'}
		sensors.append(CameraLeft)	
		CameraRight={'type': 'sensor.camera.rgb', 'x':0.25, 'y':self.baseline, 'z':1.7, 'roll':0.0, 'pitch':0.0, 'yaw':0.0,
						'width':2464, 'height': 2056, 'fov':self.fov, 'sensor_tick': 0.0, 'id': 'CameraRight'}
		sensors.append(CameraRight)
		if self.create_dataset==True:
			# self.fov=30.6
			depthCameraLeft={'type': 'sensor.camera.depth', 'x':0.25, 'y':0.0, 'z':1.7, 'roll':0.0, 'pitch':0.0, 'yaw':0.0,
							'width':2464, 'height': 2056, 'fov':self.fov, 'sensor_tick': 0.0, 'id': 'depthCameraLeft'}
			sensors.append(depthCameraLeft)	
			depthCameraRight={'type': 'sensor.camera.depth', 'x':0.25, 'y':self.baseline, 'z':1.7, 'roll':0.0, 'pitch':0.0, 'yaw':0.0,
							'width':2464, 'height': 2056, 'fov':self.fov, 'sensor_tick': 0.0, 'id': 'depthCameraRight'}
			sensors.append(depthCameraRight)

			CameraSemanticLeft={'type': 'sensor.camera.semantic_segmentation', 'x':0.25, 'y':0.0, 'z':1.7, 'roll':0.0, 'pitch':0.0, 'yaw':0.0,
						'width':2464, 'height': 2056, 'fov':self.fov, 'sensor_tick': 0.0, 'id': 'CameraSemanticLeft'}
			sensors.append(CameraSemanticLeft)


			CameraSemanticRight={'type': 'sensor.camera.semantic_segmentation', 'x':0.25, 'y':self.baseline, 'z':1.7, 'roll':0.0, 'pitch':0.0, 'yaw':0.0,
						'width':2464, 'height': 2056, 'fov':self.fov, 'sensor_tick': 0.0, 'id': 'CameraSemanticRight'}
			sensors.append(CameraSemanticRight)


		LIDAR={'type': 'sensor.lidar.ray_cast', 'x': 0.25, 'y': 0.0, 'z': 2.05, 'roll': 0.0, 'pitch': 0.0,
						'yaw': 0.0, 'channels':64,'points_per_second':350000, 'upper_fov':5.0,
						 'lower_fov':-20.0, 'range': 10000, 'sensor_tick': 0.0, 'rotation_frequency':30.0, 'id': 'LIDAR'}
		sensors.append(LIDAR)
		IMU={'type': 'sensor.other.imu', 'x': -1.45, 'y': 0.0, 'z': 2.0, 'roll': 0.0, 'pitch': 0.0, 'yaw': 0.0, 'id': 'IMU'}
		# IMU={'type': 'sensor.other.imu', 'x': 0.0, 'y': 0.0, 'z': 2.0, 'roll': 0.0, 'pitch': 0.0, 'yaw': 0.0, 'id': 'IMU'}

		sensors.append(	IMU)	
					 #{'type': 'sensor.other.gnss', 'x': 0.0, 'y': 0.0, 'z': 2.0, 'reading_frequency': 20, 'id': 'GPS_front'},
		GPS={'type': 'sensor.other.gnss', 'x': -1.45, 'y': 0.0, 'z': 2.0, 'reading_frequency': 20, 'id': 'GPS'}
		# GPS={'type': 'sensor.other.gnss', 'x': 0.0, 'y': 0.0, 'z': 2.0, 'reading_frequency': 20, 'id': 'GPS'}
		sensors.append(GPS)	
      #       		 {'type': 'sensor.other.radar', 'x': 2.7, 'y': 0.0, 'z': 0.7, 'roll': 0.0, 'pitch': 0.0,
						# 'yaw': 0.0,  'fov': 30, 'id': 'RADAR'},
		SPEED={'type': 'sensor.speedometer',  'reading_frequency': 20, 'id': 'SPEED'}   
		sensors.append(SPEED)		                          
					 # {'type': 'sensor.hd_map', 'reading_frequency': 10, 'id': 'hdmap'}
		if self.create_dataset==True:
			# gps_gt={'type': 'sensor.other.gnss', 'x': 0.0, 'y': 0.0, 'z': 2.0, 'reading_frequency': 20, 'id': 'gps_gt'}
			gps_gt={'type': 'sensor.other.gnss', 'x': -1.45, 'y': 0.0, 'z': 2.0, 'reading_frequency': 20, 'id': 'gps_gt'}
			sensors.append(gps_gt)
			imu_gt={'type': 'sensor.other.imu', 'x': -1.45, 'y': 0.0, 'z': 2.0, 'roll': 0.0, 'pitch': 0.0, 'yaw': 0.0, 'id': 'imu_gt'}
			# imu_gt={'type': 'sensor.other.imu', 'x': 0.0, 'y': 0.0, 'z': 2.0, 'roll': 0.0, 'pitch': 0.0, 'yaw': 0.0, 'id': 'imu_gt'}
			sensors.append(imu_gt)


	


		# print(sensors)
		return sensors






	def update_cameras(self, sensor_data):
		#################################################################################################
		###################################### CAMERAS ##################################################
		#################################################################################################

		if sensor_data['CameraLeft'][0] > self.last_image_index:
			self.last_image_index = sensor_data['CameraLeft'][0]

			stamp = self.stamp#rospy.Time().now()


			self.pose_a=self.pose
			dst_pose = np.sqrt( (self.pose_a[0] - self.pose_back[0])**2 + (self.pose_a[1] - self.pose_back[1])**2 )
			# print(dst_pose)



			#LEFT
			img_left = sensor_data['CameraLeft'][1]
			img_left = img_left[:,:,0:3]
			img_left = img_left[:,:,::-1]
			# img_left_flat = np.reshape(img_left, (1, img_left.shape[0]*img_left.shape[1]*3))

			# img_left_msg = Image()
			# img_left_msg.header.stamp = stamp
			# img_left_msg.header.frame_id = "stereo"
			# img_left_msg.height = img_left.shape[0]
			# img_left_msg.width = img_left.shape[1]
			# img_left_msg.encoding = 'rgb8'
			# img_left_msg.step=img_left.shape[1]*3
			# img_left_msg.data=img_left_flat[0].tolist()
	
			# camera_info_left= CameraInfo()
			# camera_info_left.header.frame_id = "stereo"
			# camera_info_left.header.stamp = stamp
			# camera_info_left.width = img_left_msg.width
			# camera_info_left.height = img_left_msg.height
			# camera_info_left.distortion_model='plumb_bob'
			# cx = camera_info_left.width/2.0
			# cy = camera_info_left.height/2.0
			# fx2 = camera_info_left.width / (2.0 * np.tan(self.fov * np.pi / 360.0)) #fov=50 (sensor configuration)
			# fy2 = fx2
			# camera_info_left.K = [fx2, 0, cx, 0, fy2, cy, 0, 0, 1]
			# camera_info_left.D = [0, 0, 0, 0, 0]
			# camera_info_left.R = [1.0, 0, 0, 0, 1.0, 0, 0, 0, 1.0]
			# camera_info_left.P = [fx2, 0, cx, 0, 0, fy2, cy, 0, 0, 0, 1.0, 0]

			#RIGHT
			img_right = sensor_data['CameraRight'][1]
			img_right = img_right[:,:,0:3]
			img_right = img_right[:,:,::-1]
			img_right_flat = np.reshape(img_right, (1, img_right.shape[0]*img_right.shape[1]*3))

			img_right_msg = Image()
			img_right_msg.header.stamp = stamp
			img_right_msg.header.frame_id = "stereo"
			img_right_msg.height = img_right.shape[0]
			img_right_msg.width = img_right.shape[1]
			img_right_msg.encoding = 'rgb8'
			img_right_msg.step=img_right.shape[1]*3
			img_right_msg.data=img_right_flat[0].tolist()
	
			camera_info_right= CameraInfo()
			camera_info_right.header.frame_id = "stereo"
			camera_info_right.header.stamp = stamp
			camera_info_right.width = img_right_msg.width
			camera_info_right.height = img_right_msg.height
			camera_info_right.distortion_model='plumb_bob'
			cx = camera_info_right.width/2.0
			cy = camera_info_right.height/2.0
			fx2 = camera_info_right.width/(2.0 * np.tan(self.fov * np.pi / 360.0)) #fov=50 (sensor configuration)
			fy2 = fx2
			# camera_info_right.K = [fx2, 0, cx, 0, fy2, cy, 0, 0, 1]
			# camera_info_right.D = [0, 0, 0, 0, 0]
			# camera_info_right.R = [1.0, 0, 0, 0, 1.0, 0, 0, 0, 1.0]
			# camera_info_right.P = [fx2, 0, cx, -fx2*self.baseline, 0, fy2, cy, 0, 0, 0, 1.0, 0]
			
			#pubs

			# self.img_left_pub.publish(img_left_msg)     
			# self.img_left_rect_color_pub.publish(img_left_msg)     
			# self.img_left_info_pub.publish(camera_info_left)
			# self.img_right_pub.publish(img_right_msg) 
			# self.img_right_rect_color_pub.publish(img_right_msg)             
			# self.img_right_info_pub.publish(camera_info_right)

			# self.img_seg_pub.publish(img_seg_msg)


			if self.create_dataset  and self.speed_temp>0.1 and dst_pose>8: 
				# print('save dataset')
				self.pose_back=self.pose_a
			# if self.create_dataset==True: 

				#semantic
				# img_sem = sensor_data['CameraSemantic'][1]
				# img_sem = img_sem[:,:,0:3]
				# img_sem = img_sem[:,:,::-1]
				# img_sem_flat = np.reshape(img_sem, (1, img_sem.shape[0]*img_sem.shape[1]*3))

				# img_sem_msg = Image()
				# img_sem_msg.header.stamp = stamp
				# img_sem_msg.header.frame_id = "stereo"
				# img_sem_msg.height = img_sem.shape[0]
				# img_sem_msg.width = img_sem.shape[1]
				# img_sem_msg.encoding = 'rgb8'
				# img_sem_msg.step=img_sem.shape[1]*3
				# img_sem_msg.data=img_sem_flat[0].tolist()
		
				# camera_info_sem= CameraInfo()
				# camera_info_sem.header.frame_id = "stereo"
				# camera_info_sem.header.stamp = stamp
				# camera_info_sem.width = img_sem_msg.width
				# camera_info_sem.height = img_sem_msg.height
				# camera_info_sem.distortion_model='plumb_bob'
				# cx = camera_info_sem.width/2.0
				# cy = camera_info_sem.height/2.0
				# fx2 = camera_info_sem.width / (2.0 * np.tan(self.fov * np.pi / 360.0)) #fov=50 (sensor configuration)
				# fy2 = fx2
				# camera_info_sem.K = [fx2, 0, cx, 0, fy2, cy, 0, 0, 1]
				# camera_info_sem.D = [0, 0, 0, 0, 0]
				# camera_info_sem.R = [1.0, 0, 0, 0, 1.0, 0, 0, 0, 1.0]
				# camera_info_sem.P = [fx2, 0, cx, 0, 0, fy2, cy, 0, 0, 0, 1.0, 0]


				#depth
				img_depth = sensor_data['depthCameraLeft'][1]
				img_depth = img_depth[:,:,0:3]
				img_depth = img_depth[:,:,::-1]
				# img_depth_flat = np.reshape(img_depth, (1, img_depth.shape[0]*img_depth.shape[1]*3))

				# img_depth_msg = Image()
				# img_depth_msg.header.stamp = stamp
				# img_depth_msg.header.frame_id = "stereo"
				# img_depth_msg.height = img_depth.shape[0]
				# img_depth_msg.width = img_depth.shape[1]
				# img_depth_msg.encoding = 'rgb8'
				# img_depth_msg.step=img_depth.shape[1]*3
				# img_depth_msg.data=img_depth_flat[0].tolist()
		
				# camera_info_depth= CameraInfo()
				# camera_info_depth.header.frame_id = "stereo"
				# camera_info_depth.header.stamp = stamp
				# camera_info_depth.width = img_depth_msg.width
				# camera_info_depth.height = img_depth_msg.height
				# camera_info_depth.distortion_model='plumb_bob'
				# cx = camera_info_depth.width/2.0
				# cy = camera_info_depth.height/2.0
				# fx2 = camera_info_depth.width / (2.0 * np.tan(self.fov * np.pi / 360.0)) #fov=50 (sensor configuration)
				# fy2 = fx2
				# camera_info_depth.K = [fx2, 0, cx, 0, fy2, cy, 0, 0, 1]
				# camera_info_depth.D = [0, 0, 0, 0, 0]
				# camera_info_depth.R = [1.0, 0, 0, 0, 1.0, 0, 0, 0, 1.0]
				# camera_info_depth.P = [fx2, 0, cx, 0, 0, fy2, cy, 0, 0, 0, 1.0, 0]



				img_depth_r = sensor_data['depthCameraRight'][1]
				img_depth_r = img_depth_r[:,:,0:3]
				img_depth_r = img_depth_r[:,:,::-1]
				# img_depth_r_flat = np.reshape(img_depth_r, (1, img_depth_r.shape[0]*img_depth_r.shape[1]*3))

				# img_depth_r_msg = Image()
				# img_depth_r_msg.header.stamp = stamp
				# img_depth_r_msg.header.frame_id = "stereo"
				# img_depth_r_msg.height = img_depth_r.shape[0]
				# img_depth_r_msg.width = img_depth_r.shape[1]
				# img_depth_r_msg.encoding = 'rgb8'
				# img_depth_r_msg.step=img_depth_r.shape[1]*3
				# img_depth_r_msg.data=img_depth_r_flat[0].tolist()
		
				# camera_info_depth_r= CameraInfo()
				# camera_info_depth_r.header.frame_id = "stereo"
				# camera_info_depth_r.header.stamp = stamp
				# camera_info_depth_r.width = img_depth_r_msg.width
				# camera_info_depth_r.height = img_depth_r_msg.height
				# camera_info_depth_r.distortion_model='plumb_bob'
				# cx = camera_info_depth_r.width/2.0
				# cy = camera_info_depth_r.height/2.0
				# fx2 = camera_info_depth_r.width / (2.0 * np.tan(self.fov * np.pi / 360.0)) #fov=50 (sensor configuration)
				# fy2 = fx2
				# camera_info_depth_r.K = [fx2, 0, cx, 0, fy2, cy, 0, 0, 1]
				# camera_info_depth_r.D = [0, 0, 0, 0, 0]
				# camera_info_depth_r.R = [1.0, 0, 0, 0, 1.0, 0, 0, 0, 1.0]
				# camera_info_depth_r.P = [fx2, 0, cx, -fx2*self.baseline, 0, fy2, cy, 0, 0, 0, 1.0, 0]


				img_semanticLeft = sensor_data['CameraSemanticLeft'][1]
				img_semanticLeft= img_semanticLeft[:,:,0:3]
				img_semanticLeft = img_semanticLeft[:,:,::-1]


				img_semanticRight = sensor_data['CameraSemanticRight'][1]
				img_semanticRight = img_semanticRight[:,:,0:3]
				img_semanticRight = img_semanticRight[:,:,::-1]
				# self.img_sem_pub.publish(img_sem_msg)     
				# self.img_sem_info_pub.publish(camera_info_sem)

				# self.img_depth_pub.publish(img_depth_msg)     
				# self.img_depth_info_pub.publish(camera_info_depth)

				# self.img_depth_r_pub.publish(img_depth_r_msg)     
				# self.img_depth_r_info_pub.publish(camera_info_depth_r)

				stampnow = rospy.Time().now()
				# save left disp
				cv2.imwrite('/mnt/Data2/carla_depth_dataset/CarlaDisp/training/rgb/left/'+str(stampnow)+'.jpg', cv2.cvtColor(img_left, cv2.COLOR_RGB2BGR))
				cv2.imwrite('/mnt/Data2/carla_depth_dataset/CarlaDisp/training/rgb/right/'+str(stampnow)+'.jpg', cv2.cvtColor(img_right, cv2.COLOR_RGB2BGR))
				cv2.imwrite('/mnt/Data2/carla_depth_dataset/CarlaDisp/training/semantic/left/'+str(stampnow)+'.png', cv2.cvtColor(img_semanticLeft, cv2.COLOR_RGB2BGR))
				cv2.imwrite('/mnt/Data2/carla_depth_dataset/CarlaDisp/training/semantic/right/'+str(stampnow)+'.png', cv2.cvtColor(img_semanticRight, cv2.COLOR_RGB2BGR))

				# cv2.imwrite('/mnt/Data2/carla_depth_dataset/depth/'+str(stamp)+'.png',	  cv2.cvtColor(img_depth, cv2.COLOR_RGB2BGR))

				R=img_depth[:,:,0]
				G=img_depth[:,:,1]
				B=img_depth[:,:,2]

				R=R.astype(np.float32)
				G=G.astype(np.float32)
				B=B.astype(np.float32)

				normalized = (R + G * 256 + B * 256 * 256) / (256 * 256 * 256 - 1)
				in_meters = 1000 * normalized
				Z=in_meters
				B=self.baseline#0.2986
				fov=self.fov #30.6
				# print(depth.shape)
				f=fx2#depth.shape[1]/ (2.0 * np.tan(fov * np.pi / 360.0)) 
				disparity=B*f/Z
				flowy=np.zeros_like(disparity)

				flowxy=np.dstack((disparity,flowy))
				flowxy=flowxy*-1
				# print(np.max(flowxy))
				# print(np.min(flowxy))

				flowxy=flowxy.astype(np.float32)
		

				visualize_flow(flowxy, '/mnt/Data2/carla_depth_dataset/CarlaDisp/training/flow_viz/left/'+str(stampnow)+'.png')
				write_flow(flowxy, '/mnt/Data2/carla_depth_dataset/CarlaDisp/training/flow/left/'+str(stampnow)+'.flo')

				# save Right disp
				R=img_depth_r[:,:,0]
				G=img_depth_r[:,:,1]
				B=img_depth_r[:,:,2]

				R=R.astype(np.float32)
				G=G.astype(np.float32)
				B=B.astype(np.float32)

				normalized = (R + G * 256 + B * 256 * 256) / (256 * 256 * 256 - 1)
				in_meters = 1000 * normalized
				Z=in_meters
				B=self.baseline#0.2986
				fov=self.fov #30.6
				# print(depth.shape)
				f=fx2#depth.shape[1]/ (2.0 * np.tan(fov * np.pi / 360.0)) 
				disparity=B*f/Z
				flowy=np.zeros_like(disparity)

				flowxy=np.dstack((disparity,flowy))
				flowxy=flowxy
				flowxy=flowxy.astype(np.float32)
		

				visualize_flow(flowxy, '/mnt/Data2/carla_depth_dataset/CarlaDisp/training/flow_viz/right/'+str(stamp)+'.png')
				write_flow(flowxy, '/mnt/Data2/carla_depth_dataset/CarlaDisp/training/flow/right/'+str(stampnow)+'.flo')

	def update_radar(self, sensor_data):
		#################################################################################################
		####################################### LIDAR ###################################################
		#################################################################################################
		#Lidar  
		stamp=self.stamp
		if self.pub_obj_radar.get_num_connections() >  0  or self.pub_marker_radar.get_num_connections() > 0:
			# print('radar')

			markerArray = MarkerArray()
			obstacle_Array = ObstacleArray()
			markerArray.markers=[]
			obstacle_Array.obstacle=[]
			t = rospy.Duration(0.35)  
			id_obj=0              
			if sensor_data['RADAR'][0] > self.last_radar_index:
				self.last_radar_index = sensor_data['RADAR'][0]


				# pc = None
				stamp = stamp#rospy.Time().now()

				# if self.radar_pub_front.get_num_connections() > 0:
				# print(sensor_data['RADAR'])

				# sensor_data['RADAR'][1]
				# header = Header()
				# header.stamp = stamp
				# header.frame_id='velodyne'

				for o in sensor_data['RADAR'][1]:
					# print('o',o)
					#[depth, azimuth, altitute, velocity] ###for carla leaderboard
					depth=o[0]
					altitude=o[1]
					azimuth=o[2]
					# velocity=o[3]

					x = depth * np.cos(altitude) * np.cos(azimuth)
					y = depth * np.cos(altitude) * np.sin(azimuth)
					z = depth * np.sin(altitude)

					marker = Marker()
					marker.header.frame_id = "velodyne"
					marker.type = marker.CUBE
					marker.action = marker.ADD
					# marker.ns = "my_namespace"

					# marker scale
					marker.scale.x = 0.3
					marker.scale.y = 0.3
					marker.scale.z = 0.3

					# marker color
					marker.color.a = 1.0
					marker.color.r = 0.0
					marker.color.g = 1.0
					marker.color.b = 1.0

					# marker orientaiton
					marker.pose.orientation.x = 0.0
					marker.pose.orientation.y = 0.0
					marker.pose.orientation.z = 0.0
					marker.pose.orientation.w = 1.0

					# marker position
					marker.pose.position.x = x#0.0
					marker.pose.position.y = y#0.0
					marker.pose.position.z = z#0.0
					marker.lifetime = t
					markerArray.markers.append(marker)

					obj = Obstacle()
					obj.header.frame_id = "velodyne"
					obj.header.stamp = stamp#rospy.Time.now()
					obj.ns = "my_namespace";


					#q = quaternion_from_euler(0, 0, ori)
					obj.pose.orientation.x = 0
					obj.pose.orientation.y = 0
					obj.pose.orientation.z = 0
					obj.pose.orientation.w = 1

					# object position
					obj.pose.position.x = x#rect[0][0]
					obj.pose.position.y = y#rect[0][1]
					obj.pose.position.z = z#-1.0


					obj.scale.x = 0.3
					obj.scale.y = 0.3
					obj.scale.z = 0.3

					obj.id = id_obj
					marker.id =id_obj

					obj.color.r = 255
					obj.color.g = 0
					obj.color.b = 0
					obj.color.a = 255

					obj.track_status = 1   ############## important 1


					obj.lifetime = t
					obj.type = -1
					obj.animation_speed = 0.5;

					markerArray.markers.append(marker)
					obstacle_Array.obstacle.append(obj)

					id_obj+=1
			self.pub_marker_radar.publish(markerArray)
			self.pub_obj_radar.publish(obstacle_Array)

		#################################################################################################
		####################################### LIDAR ###################################################
		#################################################################################################
	def update_lidar(self, sensor_data):
		#Lidar                  
		stamp=self.stamp
		if sensor_data['LIDAR'][0] > self.last_pc_index:
			self.last_pc_index = sensor_data['LIDAR'][0]
			
			pc = None
			#stamp = stamp#rospy.Time().now()

			if self.lidar_pub_front.get_num_connections() > 0:

				pc = sensor_data['LIDAR'][1]
				header = Header()
				header.stamp = stamp
				header.frame_id='velodyne'
				#pc = np.concatenate((-pc, -pc + np.random.normal(0, 0.0001, pc.shape)), axis=0)                
				pc = pc[..., [0,1,2]]
				pc[:,1]=-pc[:,1]

				point_front=np.array([[2.7,0.4,-1.9], [2.7,0.3,-1.9], [2.7,0.2,-1.9], [2.7,0.1,-1.9],[2.7,0.0, -1.9],
									  [2.7,-0.4,-1.9],[2.7,-0.3,-1.9],[2.7,-0.2,-1.9],[2.7,-0.1,-1.9],
									  [2.3,0.5,-1.9], [2.3,0.4,-1.9], [2.3,0.3,-1.9], [2.3,0.2,-1.9], [2.3,0.1,-1.9],[2.3,0.0,-1.9],
									  [2.3,-0.5,-1.9],[2.3,-0.4,-1.9],[2.3,-0.3,-1.9],[2.3,-0.2,-1.9],[2.3,-0.1,-1.9],
									  # [3,0.5,-1.9], [3,0.4,-1.9], [3,0.3,-1.9], [3,0.2,-1.9], [3,0.1,-1.9],[3,0.0,-1.9],
									  # [3,-0.5,-1.9],[3,-0.4,-1.9],[3,-0.3,-1.9],[3,-0.2,-1.9],[3,-0.1,-1.9],
									  [2.5,0.5,-1.9], [2.5,0.4,-1.9], [2.5,0.3,-1.9], [2.5,0.2,-1.9], [2.5,0.1,-1.9],[2.5,0.0,-1.9],
									  [2.5,-0.5,-1.9],[2.5,-0.4,-1.9],[2.5,-0.3,-1.9],[2.5,-0.2,-1.9],[2.5,-0.1,-1.9]
									  ])
				pc = np.concatenate((pc, point_front), axis=0) 
				#pc_front = pc[np.where(pc[:, 0] >= 1.0)[0]]
				# pc_front = pc#pc[np.where(pc[:, 0] >= 1.0)[0]]

				if self.half_pc_old is None:
					pc_msg = create_cloud_xyz32(header, pc)
				else:
					pc_front = np.concatenate((self.half_pc_old, pc), axis=0)			
					pc_msg = create_cloud_xyz32(header, pc_front)

				self.half_pc_old=pc
				self.lidar_pub_front.publish(pc_msg)




				# if len(self.queue_pc_front)>3:
				# 	del self.queue_pc_front[0]
				# 	self.queue_pc_front.append(pc_front)

				# 	pc_front = np.concatenate(self.queue_pc_front, axis=0)
				# 	pc_msg = create_cloud_xyz32(header, pc_front)
				# 	self.lidar_pub_front.publish(pc_msg)
				# 	self.last_pc_front = None
				# else:
				# 	self.queue_pc_front.append(pc_front)

				# if self.last_pc_front is None:
				#   self.last_pc_front = pc_front
				#   print ("Front 1")
				# else:
				#   pc_front = np.concatenate((self.last_pc_front, pc_front), axis=0)               
				#   pc_msg = create_cloud_xyz32(header, pc_front)
				#   self.lidar_pub_front.publish(pc_msg)
				#   self.last_pc_front = None
				#   print ("Front 2")

			# if self.lidar_pub_back.get_num_connections()> 0:
				
			# 	if pc is not None:
			# 		header = Header()
			# 		header.stamp = stamp
			# 		header.frame_id='velodyne'
			# 		pc_back = pc[np.where(pc[:,0]<1.0)[0]]
					
			# 		if len(self.queue_pc_back) > 3:
			# 			del self.queue_pc_back[0]
			# 			self.queue_pc_back.append(pc_back)

			# 			pc_back = np.concatenate(self.queue_pc_back, axis=0)
			# 			pc_msg_back = create_cloud_xyz32(header, pc_back)
			# 			self.lidar_pub_back.publish(pc_msg_back)
			# 		else:
			# 			self.queue_pc_back.append(pc_back)

			# 	else:
			# 		pc = sensor_data['LIDAR'][1]
			# 		header = Header()
			# 		header.stamp = stamp
			# 		header.frame_id='velodyne'
			# 		#pc = np.concatenate((-pc, -pc + np.random.normal(0, 0.0001, pc.shape)), axis=0)                
			# 		pc = -pc[..., [1,0,2]]
			# 		pc_back = pc[np.where(pc[:, 0] < 1.0)[0]]

			# 		if len(self.queue_pc_back) > 3:
			# 			del self.queue_pc_back[0]
			# 			self.queue_pc_back.append(pc_back)

			# 			pc_back = np.concatenate(self.queue_pc_back, axis=0)
			# 			pc_msg_back = create_cloud_xyz32(header, pc_back)
			# 			self.lidar_pub_back.publish(pc_msg_back)
			# 		else:
			# 			self.queue_pc_back.append(pc_back)

						
	def routeToWaypoints(self, route):
		#################################################################################################
		####################################### ROUTE ###################################################
		#################################################################################################
		stamp=self.stamp
		waypoints = []
		
		msg = GlobalPlan()
		msg.header.stamp = stamp#rospy.Time().now()
		msg.header.frame_id = 'map'
		msg.points = []
		msg.road_options = []


		for way in route:
			lat=way[0]['lat']
			lon=way[0]['lon']
			z=way[0]['z']
			utm_data = self.geodesicToMercator(lat, lon)


			p = Point()
			p.x = utm_data[1]
			p.y = utm_data[2]
			p.z = z
			msg.points.append(p)

			if way[1] == RoadOption.LANEFOLLOW:
				msg.road_options.append(msg.LANEFOLLOW)
				road_opt=0
			elif way[1] == RoadOption.LEFT:
				msg.road_options.append(msg.LEFT)
				road_opt=1
			elif way[1] == RoadOption.RIGHT:
				msg.road_options.append(msg.RIGHT)
				road_opt=2
			elif way[1] == RoadOption.STRAIGHT:
				msg.road_options.append(msg.STRAIGHT)
				road_opt=3
			elif way[1] == RoadOption.CHANGELANELEFT:
				msg.road_options.append(msg.CHANGELANELEFT)
				road_opt=4
			elif way[1] == RoadOption.CHANGELANERIGHT:
				msg.road_options.append(msg.CHANGELANERIGHT)
				road_opt=5
			else:
				msg.road_options.append(msg.UNKNOWN)
				road_opt=6
			waypoints.append([utm_data[1], utm_data[2], z, road_opt])

		waypoints = np.asarray(waypoints) 




		
		return np.asarray(waypoints), msg

	def geodesicToMercator(self, lat, lon):
		EARTH_RADIUS_EQUA = 6378137.0
		 
		scale = 1.0#np.cos(np.radians(lat))
		x = scale * np.radians(lon) * EARTH_RADIUS_EQUA
		y = scale * EARTH_RADIUS_EQUA * np.log(np.tan((90.0 + lat) * np.pi/ 360.0))
		# return in right hand coordinates
		return [0, y, -x]

	def gpsToWorld(self, gps):
		#################################################################################################
		######################################## GPS ####################################################
		#################################################################################################
		#utm_data = LLtoUTM(23, gps[1], gps[0])
		utm_data = self.geodesicToMercator(gps[0], gps[1])
		# self.all_ll.append([gps[1], gps[0]]) 
		# np.save('/home/carla/ll_ll.npy', self.all_ll)
		return np.array([utm_data[1], utm_data[2], gps[2]])
		#utm_data = utm.from_latlon(gps[1][0], gps[1][1])
		#return np.array([utm_data[0], utm_data[1], gps[1][2]])

	def find_road_and_lane(self,root, x, y):
	    for road in root.iter("road"):
	        lane_level = LaneLevel(root, road, ds=0.2)
	        road_attr = lane_level.get_road_attributes()

	        for key in road_attr.keys():
	            lanes = road_attr[key]
	            for lane in lanes:
	                lane_geom = lane["geometry"]
	                x_geom = lane_geom["x"]
	                y_geom = lane_geom["y"]
	                min_dist = np.amin(np.sqrt( (x_geom - x)**2 + (y_geom - y)**2 ) )
	                if min_dist < 1.0:
	                    return str(road.attrib["id"]), str(lane_geom["id"]), str(lane_geom["travel_dir"])


	def update_map(self, sensor_data):
		stamp=self.stamp
		self.OpenDRIVE_loaded=True

		# text_file = open("/home/luis/carla/opendrive.txt", "w")


		# dict = input_data['OpenDRIVE'][1]
		# with open('/home/luis/carla/opendrive.txt', 'w') as fp:
		#   json.dump(dict, fp)

		# print(input_data)
		# print('loading ...')
		# print(self.OpenDRIVE_loaded)
		# print(self.tree)
			# od=input_data['OpenDRIVE']
			# print('loading opendrive map ...')
			# print(od)
		# self.waypoints, self.global_plan_msg = self.routeToWaypoints(self._global_plan)
		#self.waypoints = np.concatenate(([[pose_ekf[0], pose_ekf[1], pose[2]]], self.waypoints), axis=0)


		# self.waypoints_raw = np.concatenate((interp_points, self.waypoints_raw), axis=0)
		# print(self.waypoints_raw)
		try:
			self.tree = ET.ElementTree(ET.fromstring(sensor_data['OpenDRIVE'][1]['opendrive']))

			# print('tree loaded!')
			waypoints_od = self.waypoints_raw[:,:2]#np.load("Town03.npy")

			# print(waypoints_od)
			waypoints_od_x = self.waypoints_raw[:,1]#np.load("Town03.npy")
			waypoints_od_y = self.waypoints_raw[:,0]#np.load("Town03.npy")
			waypoints_od=np.column_stack((-waypoints_od_x,waypoints_od_y))
			# print(waypoints_od)

			# waypoints_od = np.concatenate(([[-pose[1], pose[0]]], waypoints_od), axis=0)

			# print(waypoints_od)
			root = self.tree.getroot()
			road_network = RoadNetwork(root)

			# for road in root.iter("road"):
			#     if road.attrib["id"] == "60":
			#         lane_level = LaneLevel(root, road, ds=0.2)
			#         road_attr = lane_level.get_road_attributes()
			#
			#         for key in road_attr.keys():
			#             lanes = road_attr[key]
			#             for lane in lanes:
			#                 lane_geom = lane["geometry"]
			#                 plt.plot(lane_geom["x"], lane_geom["y"], 'k.-')
			# for road in root.iter("road"):
			#     lane_level = LaneLevel(root, road, ds=0.2)
			#     road_attr = lane_level.get_road_attributes()

			#     for key in road_attr.keys():
			#         lanes = road_attr[key]
			#         for lane in lanes:
			#             lane_geom = lane["geometry"]
			            # plt.plot(lane_geom["x"], lane_geom["y"], 'k.-')

			            # for lane_tl in lane["traffic_light"]:
			            #     plt.plot(lane_tl["x"], lane_tl["y"], 'ys')
			            # for lane_sl in lane["speed_limit"]:
			            #     plt.plot(lane_sl["x"], lane_sl["y"], 'gs')
			            # for lane_st in lane["stencil_stop"]:
			            #     plt.plot(lane_st["x"], lane_st["y"], 'bs')
			            # for lane_sline in lane["stop_line"]:
			            #     plt.plot(lane_sline["x"], lane_sline["y"], 'rs')


			# plt.plot(waypoints_od[:, 0],waypoints_od[:, 1], 'ro')
			# plt.axis('equal')
			# plt.show()
			used_roads = []
			x_way = []
			y_way = []
			hdg_way = []
			curv_way = []

			for w in range(len(waypoints_od)-1):
			    waypoint_ox = waypoints_od[w][0]
			    waypoint_oy = waypoints_od[w][1]
			    waypoint_gx = waypoints_od[w+1][0]
			    waypoint_gy = waypoints_od[w+1][1]
			    # print("Waypoint: ",w)
			    # if w < 2:
			    #     continue

			    #Find first point
			    road_ido, lane_ido, travelDiro = self.find_road_and_lane(root, waypoint_ox, waypoint_oy)

			    #Find second point
			    road_idg, lane_idg, travelDirg = self.find_road_and_lane(root, waypoint_gx, waypoint_gy)

			    current_road_id, travel_dir = road_ido, travelDiro
			    next_roads = road_network.get_next_roads( current_road_id, travel_dir )

			    while current_road_id != road_idg:
			        # print(next_roads)
			        # next_road_id, travel_dir = road_network.get_random_next_road(next_roads)
			        if len(next_roads) > 1:
			            dist = np.inf
			            for nr in next_roads:
			                next_road_id_temp = nr[0]
			                travel_dir_temp = nr[1]
			                for road in root.iter("road"):
			                    if road.attrib["id"] == next_road_id_temp:
			                        lane_level = LaneLevel(root, road, ds=0.2)
			                        road_attr = lane_level.get_road_attributes()
			                        lanes = road_attr[travel_dir_temp]
			                        for lane in lanes:
			                            lane_geom = lane["geometry"]
			                            if np.amin( np.sqrt((waypoint_gx-lane_geom["x"][-1])**2 + (waypoint_gy-lane_geom["y"][-1])**2 )) < dist:
			                                dist = np.amin( np.sqrt((waypoint_gx-lane_geom["x"][-1])**2 + (waypoint_gy-lane_geom["y"][-1])**2 ))
			                                next_road_id = next_road_id_temp
			                                travel_dir = travel_dir_temp
			        else:
			            if len(next_roads) == 0:
			                pass
			            else:
			                next_road_id, travel_dir = road_network.get_random_next_road(next_roads)

			        #TODO fix issue with town 6
			        if len(next_roads) == 0:
			            break

			        lanes_id = road_network.get_lanes(current_road_id, next_road_id, travel_dir)
			        previous_road = current_road_id
			        current_road_id = next_road_id
			        # print(current_road_id)
			        next_roads = road_network.get_next_roads( current_road_id, travel_dir )

			        dist = np.inf
			        for road in root.iter("road"):
			            if road.attrib["id"] == current_road_id and previous_road != current_road_id:
			                lane_level = LaneLevel(root, road, ds=1.0)
			                road_attr = lane_level.get_road_attributes()
			                lanes = road_attr[travel_dir]
			                for lane in lanes:
			                    lane_geom = lane["geometry"]
			                    min_dist = np.amin( np.sqrt((lane_geom["x"] - waypoint_ox)**2+(lane_geom["y"] - waypoint_oy)**2))
			                    if min_dist < dist:
			                        dist = min_dist
			                        x_geom = lane_geom["x"]
			                        y_geom = lane_geom["y"]

			                idx_o = np.argmin( np.sqrt((x_geom - waypoint_ox)**2+(y_geom - waypoint_oy)**2))
			                idx_g = np.argmin( np.sqrt((x_geom - waypoint_gx)**2+(y_geom - waypoint_gy)**2))

			                for x, y in zip(x_geom, y_geom):
			                    if len(x_way) > 0:
			                        dist = np.amin( np.sqrt((x - x_way)**2+(y - y_way)**2))
			                        if dist > 0:
			                            x_way.append(x)
			                            y_way.append(y)
			                    else:
			                        x_way.append(x)
			                        y_way.append(y)


			# plt.plot(x_way, y_way, 'g.-')
			# plt.show()
			z_way=np.zeros_like(x_way)
			mode_way=np.ones_like(x_way)*6
			new_waypoints=np.column_stack((y_way,x_way,z_way,mode_way))
			new_waypoints[:,1] = -new_waypoints[:,1]
			# self.waypoints=new_waypoints
			# print(interp_points[:,:2])
			# print('self.waypoints')
			# print(self.waypoints)


			# if (self.create_dataset):
				# self.waypoints = np.concatenate(([[pose_gt[0], pose_gt[1], pose_gt[2],6]], self.waypoints), axis=0)
			# else:
			# new_waypoints = np.concatenate(([[self.pose[0], self.pose[1], self.pose[2],6]], new_waypoints), axis=0)

			new_way=[]

			for i in range(len(new_waypoints)-1):

				new_way.append(new_waypoints[i])

				dst = np.sqrt( (new_waypoints[i][0] - new_waypoints[i+1][0])**2 + (new_waypoints[i][1] - new_waypoints[i+1][1])**2 )

				if dst>2:
					# print('dst > 10')
					line = LineString(([new_waypoints[i][0], new_waypoints[i][1]], [ new_waypoints[i+1][0], new_waypoints[i+1][1] ]))

					distance_delta = 0.9
					distances = np.arange(0, line.length, distance_delta)
					# or alternatively without NumPy:
					# points_count = int(line.length // distance_delta) + 1
					# distances = (distance_delta * i for i in range(points_count))
					points = [line.interpolate(distance) for distance in distances] + [line.boundary[1]]
					# multipoint = unary_union(points)  # or new_line = LineString(points)
					# print('points')
					# interp_points=[]
					for p in points:
						# print(np.array(p))
						new_way.append([p.x,p.y,0,6])
					# interp_points=np.array(interp_points)
					# print(interp_points)
			new_way.append(new_waypoints[-1])

			# print(self.waypoints_raw)



			way = np.array(new_way)


				# self.waypoints = np.concatenate((interp_points, self.waypoints), axis=0)
				# print(self.waypoints)

		except Exception as e:
			print(e)
			print("An exception occurred. OpenDRIVE map could not be loaded")
			pass


		if not self.convert_route_to_way:
			print ("[Carina Agent] send new route!")

			# if (self.create_dataset):
			# 	way = np.concatenate(([[self.pose_gt[0], self.pose_gt[1], self.pose_gt[2],6]], way), axis=0)
			# else:
			# way = np.concatenate(([[self.poseini[0], self.poseini[1], self.poseini[2],6]], way), axis=0)
			# self.waypoints, self.global_plan_msg = self.routeToWaypoints(self._global_plan)
			#self.waypoints = np.concatenate(([[pose_ekf[0], pose_ekf[1], pose[2]]], self.waypoints), axis=0)
			# if (self.create_dataset):
			# 	self.waypoints = np.concatenate(([[pose_gt[0], pose_gt[1], pose_gt[2],6]], self.waypoints), axis=0)
			# else:
			# 	self.waypoints = np.concatenate(([[pose[0], pose[1], pose[2],6]], self.waypoints), axis=0)

			# plt.plot(self.waypoints[:,0],self.waypoints[:,1],'ro')
			# plt.show()


			stamp=self.stamp
			self.route = Path()
			self.route.header.stamp = stamp#rospy.Time().now()
			self.route.header.frame_id = 'map'
			self.route.poses = []
			for p in way:
				ps = PoseStamped()
				ps.header.frame_id = 'map'
				ps.header.stamp = stamp#rospy.Time().now()
				ps.pose.position.x = p[0]
				ps.pose.position.y = p[1]
				ps.pose.position.z = p[2]
				self.route.poses.append(ps)

		# if self.create_dataset:
		# 	self.route_raw_pub.publish(self.route_raw)
		# 	self.global_plan_raw_pub.publish(self.global_plan_raw_msg)

		self.convert_route_to_way = True

		self.route_pub.publish(self.route)

		# signal_shutdown = Bool()
		# signal_shutdown.data = True
		# self.shutdown_using_map.publish(signal_shutdown)



	def run_step(self, input_data, timestamp):
		# print(input_data)
		# t = time.time()
		#stamp=self.stamp
		#################################################################################################
		#################################### UPDATE CLOCK ###############################################
		#################################################################################################
		#clock_msg = Clock()
		#clock_msg.clock = rospy.Time()
		#clock_msg.clock.secs = int(timestamp)
		#clock_msg.clock.nsecs = int((timestamp - int(timestamp))*1e9)
		#self.clock_pub.publish(clock_msg)

		clock_msg = Clock()
		# self.clock_msg.clock = rospy.Time()
		# clock_msg.clock = self.clock_ini + rospy.Time.from_sec(timestamp)
		self.stamp = self.clock_ini + rospy.Duration.from_sec(timestamp)#rospy.Time.Duration(timestamp)
		# print('stamp ',self.stamp )
		clock_msg.clock = self.stamp


		# clock_msg.clock.secs = int(timestamp)
		# clock_msg.clock.nsecs = int((timestamp - int(timestamp))*1e9)
		self.clock_pub.publish(clock_msg)

        # self.timestamp = timestamp
        # self.clock_publisher.publish(Clock(rospy.Time.from_sec(timestamp)))

		# self.stamp = rospy.Time.from_sec(timestamp)#
		stamp = self.stamp#rospy.Time().now()

		#################################################################################################
		####################################### ROUTE ###################################################
		#################################################################################################
		if not self.convert_route_to_way:
			if self.create_dataset: 
				# self.waypoints_raw, self.global_plan_raw_msg = self.routeToWaypoints(self._global_plan)
				self.waypoints, self.global_plan_msg = self.routeToWaypoints(self._global_plan_gt)
			else:
				self.waypoints, self.global_plan_msg = self.routeToWaypoints(self._global_plan)

		# if not self.convert_route_to_way:
		#       print ("[Carina Agent] send new route!")
		#       self.convert_route_to_way = True
		#       self.waypoints, self.global_plan_msg = self.routeToWaypoints(self._global_plan)

		#       self.route = Path()
		#       self.route.header.stamp = rospy.Time().now()
		#       self.route.header.frame_id = 'map'
		#       self.route.poses = []
		#       for p in self.waypoints:
		#           ps = PoseStamped()
		#           ps.header.frame_id = 'map'
		#           ps.header.stamp = rospy.Time().now()
		#           ps.pose.position.x = p[0]
		#           ps.pose.position.y = p[1]
		#           ps.pose.position.z = p[2]
		#           self.route.poses.append(ps)

		# self.route_pub.publish(self.route)
		# self.global_plan_pub.publish(self.global_plan_msg)

		
		#################################################################################################
		######################################## imu ####################################################
		#################################################################################################   
		yaw_gt=None

		if self.create_dataset:       
			m_imu_gt = Imu()
			m_imu_gt.header.stamp=stamp#rospy.Time().now()
			m_imu_gt.header.frame_id="imu"

			yaw = -input_data['imu_gt'][1][-1]
			yaw_gt=yaw


			#m_imu.orientation=-input_data['IMU'][1][-1]
			m_imu_gt_ori= tf.transformations.quaternion_from_euler(0.0, 0.0, yaw)

			m_imu_gt.orientation.x = m_imu_gt_ori[0]
			m_imu_gt.orientation.y = m_imu_gt_ori[1]
			m_imu_gt.orientation.z = m_imu_gt_ori[2]
			m_imu_gt.orientation.w = m_imu_gt_ori[3]
			#print(input_data['IMU'])
			# m_imu.orientation_covariance=[-1.0, 0.0, 0.0,
			# 							 0.0, 0.0, 0.0,
	  #                                    0.0, 0.0, 0.0]
			m_imu_gt.linear_acceleration=Vector3(input_data['imu_gt'][1][0], -input_data['imu_gt'][1][1], input_data['imu_gt'][1][2])
			#m_imu.linear_acceleration = Vector3(x, y, z)
			# m_imu.angular_velocity_covariance=[-1.0, 0.0, 0.0,
	  #                                         0.0, 0.0, 0.0,
	  #                                         0.0, 0.0, 0.0]
			m_imu_gt.angular_velocity =Vector3(-input_data['imu_gt'][1][3], input_data['imu_gt'][1][4], -input_data['imu_gt'][1][5])
			#m_imu.linear_acceleration = Vector3(x, y, z)

			# m_imu.linear_acceleration_covariance=[0.0, 0.0, 0.0,
	  #                                            0.0, 0.0, 0.0,
	  #                                            0.0, 0.0, 0.0]
			#print(m_imu.orientation)
			self.imu_gt_pub.publish(m_imu_gt)



		m_imu = Imu()
		m_imu.header.stamp=stamp#rospy.Time().now()
		m_imu.header.frame_id="imu"

		yaw = -input_data['IMU'][1][-1]

		# print('yaw: ',yaw)
		#m_imu.orientation=-input_data['IMU'][1][-1]
		m_imu_ori= tf.transformations.quaternion_from_euler(0.0, 0.0, yaw)

		m_imu.orientation.x = m_imu_ori[0]
		m_imu.orientation.y = m_imu_ori[1]
		m_imu.orientation.z = m_imu_ori[2]
		m_imu.orientation.w = m_imu_ori[3]
		#print(input_data['IMU'])
		#m_imu.orientation_covariance=[-1.0, 0.0, 0.0,
		# 							   0.0, 0.0, 0.0,
		#                               0.0, 0.0, 0.0]
		m_imu.linear_acceleration=Vector3(input_data['IMU'][1][0], -input_data['IMU'][1][1], input_data['IMU'][1][2])
		#m_imu.linear_acceleration = Vector3(x, y, z)
		m_imu.linear_acceleration_covariance=[0.001**2, 0.0, 0.0,
                                              0.0, 0.001**2, 0.0,
		                                      0.0, 0.0, 0.015**2]

		m_imu.angular_velocity =Vector3(-input_data['IMU'][1][3], input_data['IMU'][1][4], -input_data['IMU'][1][5])
		#m_imu.linear_acceleration = Vector3(x, y, z)
		m_imu.angular_velocity_covariance=[0.001**2, 0.0, 0.0,
	                                        0.0, 0.001**2, 0.0,
                                            0.0, 0.0, 0.001**2]

		#print(m_imu.orientation)
		self.imu_pub.publish(m_imu)
		#################################################################################################
		######################################## GPS ####################################################
		#################################################################################################
		#yaw = -input_data['IMU'][1][-1]

		if self.create_dataset and input_data['gps_gt'][0] > self.last_gps_gt_index:
			self.last_gps_gt_index = input_data['gps_gt'][0]

			nav_sat_fix_gt = NavSatFix()
			nav_sat_fix_gt.header.stamp=stamp#rospy.Time().now()
			nav_sat_fix_gt.header.frame_id="gps"
			nav_sat_fix_gt.latitude= input_data['gps_gt'][1][0]
			nav_sat_fix_gt.longitude= input_data['gps_gt'][1][1]
			nav_sat_fix_gt.altitude = input_data['gps_gt'][1][2]
			self.nav_sat_fix_gt_pub.publish(nav_sat_fix_gt)
			
			pose_gt = self.gpsToWorld(input_data['gps_gt'][1])
			# pose_front = self.gpsToWorld(input_data['GPS_front'][1])

			# print(input_data['GPS_back'])

			# print('self.last_pose: ',self.last_pose)
			if self.last_pose_gt is None:
				# yaw = 0.0#np.arctan2(pose_front[1]-pose[1] , pose_front[0]-pose[0])

				self.origin_gt.pose.position.x = self.waypoints[0,0]
				self.origin_gt.pose.position.y = self.waypoints[0,1]
				self.origin_gt.pose.position.z = 0
				self.origin_gt_yaw = yaw_gt
				# print('self.origin.pose.position ',self.origin.pose.position)

			# else:
			#   yaw = 0.0#['IMU'][1][5]#0#np.arctan2(pose_front[1] - pose[1], pose_front[0] - pose[0])

			# pose_ekf = self.ekf_gps.fit(pose[0], pose[1], yaw, -self.steering_angle*self.max_steering, abs(input_data['SPEED'][1]['speed']))
			# pose = pose_ekf
			# pose_ekf = pose

			self.last_pose_gt=pose_gt
			self.last_yaw_gt=yaw_gt

			# ori = tf.transformations.quaternion_from_euler(0.0, 0.0, pose_ekf[2])
			ori_gt = tf.transformations.quaternion_from_euler(0.0, 0.0, yaw_gt)
												
			#stamp = rospy.Time().now()
			pose_gt_msg = PoseWithCovarianceStamped()
			pose_gt_msg.header.stamp = stamp
			pose_gt_msg.header.frame_id = 'map'
			pose_gt_msg.pose.pose.position.x = pose_gt[0]#pose_ekf[0]
			pose_gt_msg.pose.pose.position.y = pose_gt[1]#pose_ekf[1]
			pose_gt_msg.pose.pose.position.z = pose_gt[2]#pose[2]
			pose_gt_msg.pose.pose.orientation.x = ori_gt[0]#ori[0]
			pose_gt_msg.pose.pose.orientation.y = ori_gt[1]#ori[1]
			pose_gt_msg.pose.pose.orientation.z = ori_gt[2]#ori[2]
			pose_gt_msg.pose.pose.orientation.w = ori_gt[3]#ori[3]
			self.gps_gt_pub.publish(pose_gt_msg)

		if input_data['GPS'][0] > self.last_gps_index:
			self.last_gps_index = input_data['GPS'][0]


			nav_sat_fix = NavSatFix()
			nav_sat_fix.header.stamp=stamp#rospy.Time().now()
			nav_sat_fix.header.frame_id="gps"
			nav_sat_fix.latitude= input_data['GPS'][1][0]
			nav_sat_fix.longitude= input_data['GPS'][1][1]
			nav_sat_fix.altitude = input_data['GPS'][1][2]


			nav_sat_fix.position_covariance=[10.0,   0.0,   0.0,
											  0.0,  10.0,   0.0,
											  0.0,   0.0,  10.0
												]



			self.nav_sat_fix_pub.publish(nav_sat_fix)

			
			pose = self.gpsToWorld(input_data['GPS'][1])
			error_gps = self.gpsToWorld([0.000005, 0.000005, 0.000005])
			#print(error_gps)

			# pose_front = self.gpsToWorld(input_data['GPS_front'][1])

			# print(input_data['GPS_back'])

			# print('self.last_pose: ',self.last_pose)
			if self.last_pose is None:
				# yaw = 0.0#np.arctan2(pose_front[1]-pose[1] , pose_front[0]-pose[0])

				self.origin.pose.position.x = self.waypoints[0,0]
				self.origin.pose.position.y = self.waypoints[0,1]
				self.origin.pose.position.z = 0
				self.origin_yaw = yaw
				# print('self.origin.pose.position ',self.origin.pose.position)

			# else:
			#   yaw = 0.0#['IMU'][1][5]#0#np.arctan2(pose_front[1] - pose[1], pose_front[0] - pose[0])

			try:

				pose_ekf = self.ekf_gps.fit(pose[0], pose[1], yaw, -self.steering_angle*self.max_steering, abs(input_data['SPEED'][1]['speed']))
				pose = pose_ekf
			except Exception as e:
				print(e, 'in Kalman Filter')



			

			if self.first_tick:
				self.poseini=pose


			self.last_pose=pose

			# if self.create_dataset :
			# 	yaw=yaw_gt

			self.last_yaw=yaw

			# ori = tf.transformations.quaternion_from_euler(0.0, 0.0, pose_ekf[2])
			yaw_n=yaw

			collect_depth=True

			if self.create_dataset:
				if not collect_depth:
					noise = np.random.normal(0,0.5,1)[0]
					yaw_n=yaw+noise
				pose = pose_gt

			ori = tf.transformations.quaternion_from_euler(0.0, 0.0, yaw_n)
												
			#stamp = rospy.Time().now()
			pose_msg = PoseWithCovarianceStamped()
			pose_msg.header.stamp = stamp
			pose_msg.header.frame_id = 'map'
			pose_msg.pose.pose.position.x = pose[0]#pose_ekf[0]
			pose_msg.pose.pose.position.y = pose[1]#pose_ekf[1]
			pose_msg.pose.pose.position.z = pose[2]
			pose_msg.pose.pose.orientation.x = ori[0]
			pose_msg.pose.pose.orientation.y = ori[1]
			pose_msg.pose.pose.orientation.z = ori[2]
			pose_msg.pose.pose.orientation.w = ori[3]
			# if !sel.clean_gps:
			# if not self.create_dataset :
			pose_msg.pose.covariance=[error_gps[0], 0.0, 0.0, 0.0, 0.0, 0.0,
												0.0, error_gps[0], 0.0, 0.0, 0.0, 0.0,
												0.0, 0.0, error_gps[0], 0.0, 0.0, 0.0,
												0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
												0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
												0.0, 0.0, 0.0, 0.0, 0.0, 0.0
												]			

			self.gps_pub.publish(pose_msg)

			
			if self.create_dataset==True :
				pose = pose_gt
				ori=ori_gt
				self.origin=self.origin_gt
			# uncoment if use robot-localization + rtab
			odom_msg = Odometry()
			odom_msg.header = pose_msg.header
			odom_msg.header.stamp = stamp
			odom_msg.header.frame_id = 'odom'#'map'#'odom'
			odom_msg.child_frame_id = 'base_link'#'gps'#'base_link'#'gps'#'base_link'
			odom_msg.pose.pose.position.x = pose[0] - self.origin.pose.position.x#pose_ekf[0] - self.origin.pose.position.x #+ 1.0#pose[0]
			odom_msg.pose.pose.position.y = pose[1] - self.origin.pose.position.y#pose_ekf[1] - self.origin.pose.position.y
			odom_msg.pose.pose.position.z = pose[2] - 2 - self.origin.pose.position.z#0
			odom_msg.pose.pose.orientation.x = ori[0]
			odom_msg.pose.pose.orientation.y = ori[1]
			odom_msg.pose.pose.orientation.z = ori[2]
			odom_msg.pose.pose.orientation.w = ori[3]
			self.odometry_pub.publish(odom_msg)

			tmsg = TransformStamped()
			tmsg.header.stamp = stamp
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
			tmsg.header.stamp = stamp
			tmsg.header.frame_id = 'map'
			tmsg.child_frame_id = 'odom'
			tmsg.transform.translation.x = self.origin.pose.position.x #+ 1.0
			tmsg.transform.translation.y = self.origin.pose.position.y
			tmsg.transform.translation.z = self.origin.pose.position.z
			tmsg.transform.rotation.x = 0.0#self.origin.pose.orientation.x
			tmsg.transform.rotation.y = 0.0#self.origin.pose.orientation.y
			tmsg.transform.rotation.z = 0.0#self.origin.pose.orientation.z
			tmsg.transform.rotation.w = 1.0#self.origin.pose.orientation.w
			self.tf_br_map_odom.sendTransform(tmsg)

			# ori2 = tf.transformations.quaternion_from_euler(0.0, 0.0, self.norm_ang(pose_ekf[2] - np.deg2rad(input_data['hdmap'][1]['transform']['yaw']) ))
			# tmsg = TransformStamped()
			# tmsg.header.stamp = stamp
			# tmsg.header.frame_id = 'odom'
			# tmsg.child_frame_id = 'hdmap'
			# tmsg.transform.translation.x =  odom_msg.pose.pose.position.y  -  input_data['hdmap'][1]['transform']['x'] 
			# tmsg.transform.translation.y =  odom_msg.pose.pose.position.x  - input_data['hdmap'][1]['transform']['y']
			# tmsg.transform.translation.z = odom_msg.pose.pose.position.z - input_data['hdmap'][1]['transform']['z']
			# tmsg.transform.rotation.x = ori2[0]
			# tmsg.transform.rotation.y = ori2[1]
			# tmsg.transform.rotation.z = ori2[2]
			# tmsg.transform.rotation.w = ori2[3]
			# self.tf_br_hdmap_odom.sendTransform(tmsg)



		#################################################################################################
		################################### UPDATE SENSORS ##############################################
		#################################################################################################
		thread_cameras = Thread(target=self.update_cameras, args=[input_data])
		thread_cameras.start()
		thread_lidar = Thread(target=self.update_lidar, args=[input_data])
		thread_lidar.start()

		# thread_lidar = Thread(target=self.update_radar, args=[input_data])
		# thread_lidar.start()



		#################################################################################################
		####################################### STATE ###################################################
		#################################################################################################
		#vehicle state
		vehicle_state_msg = VehicleState()
		vehicle_state_msg.header.stamp = stamp#rospy.Time().now()
	
		vehicle_state_msg.drive.speed=abs(input_data['SPEED'][1]['speed'])
		self.speed_temp=vehicle_state_msg.drive.speed
		vehicle_state_msg.drive.steering_angle = -self.steering_angle*self.max_steering
		vehicle_state_msg.handbrake=0
		vehicle_state_msg.brake=0
		self.vehicle_state_pub.publish(vehicle_state_msg)




		############################# global plan ################################################3


		self.waypoints_raw, self.global_plan_raw_msg = self.routeToWaypoints(self._global_plan)


		self.route_raw = Path()
		self.route_raw.header.stamp = stamp#rospy.Time().now()
		self.route_raw.header.frame_id = 'map'
		self.route_raw.poses = []
		for p in self.waypoints_raw:
			ps = PoseStamped()
			ps.header.frame_id = 'map'
			ps.header.stamp = stamp#rospy.Time().now()
			ps.pose.position.x = p[0]
			ps.pose.position.y = p[1]
			ps.pose.position.z = p[2]
			self.route_raw.poses.append(ps)

		if self.create_dataset and self.publis_gt_waypoints:
			self.route = Path()
			self.route.header.stamp = stamp#rospy.Time().now()
			self.route.header.frame_id = 'map'
			self.route.poses = []

			for p in self.waypoints:
				ps = PoseStamped()
				ps.header.frame_id = 'map'
				ps.header.stamp = stamp#rospy.Time().now()
				ps.pose.position.x = p[0]
				ps.pose.position.y = p[1]
				ps.pose.position.z = p[2]
				self.route.poses.append(ps)

			self.route_pub.publish(self.route)
			self.publis_gt_waypoints=False


		self.global_plan_raw_pub.publish(self.global_plan_raw_msg)
		self.route_vis_pub.publish(self.route)

		#################################################################################################
		####################################### HDMAP ###################################################
		#################################################################################################
		self.pose = pose

		if self.track == Track.MAP and self.OpenDRIVE_loaded==False:
			thread_map = Thread(target=self.update_map, args=[input_data])
			thread_map.start()

			
		# self.OpenDRIVE_loaded=True



		self.global_plan_pub.publish(self.global_plan_msg)


		# if self.first_tick:
		# 	print ("[Carina Agent] send provisional new route!")
		# 	self.first_tick=False

		# 	# way = np.concatenate(([[self.pose[0], self.pose[1], self.pose[2],6]], self.waypoints_raw), axis=0)
		# 	# self.waypoints, self.global_plan_msg = self.routeToWaypoints(self._global_plan)
		# 	#self.waypoints = np.concatenate(([[pose_ekf[0], pose_ekf[1], pose[2]]], self.waypoints), axis=0)
		# 	# if (self.create_dataset):
		# 	# 	self.waypoints = np.concatenate(([[pose_gt[0], pose_gt[1], pose_gt[2],6]], self.waypoints), axis=0)
		# 	# else:
		# 	# 	self.waypoints = np.concatenate(([[pose[0], pose[1], pose[2],6]], self.waypoints), axis=0)

		# 	# plt.plot(self.waypoints[:,0],self.waypoints[:,1],'ro')
		# 	# plt.show()

		# 	# self.convert_route_to_way = True


		# 	self.route = Path()
		# 	self.route.header.stamp = stamp#rospy.Time().now()
		# 	self.route.header.frame_id = 'map'
		# 	self.route.poses = []
		# 	for p in way:
		# 		ps = PoseStamped()
		# 		ps.header.frame_id = 'map'
		# 		ps.header.stamp = stamp#rospy.Time().now()
		# 		ps.pose.position.x = p[0]
		# 		ps.pose.position.y = p[1]
		# 		ps.pose.position.z = p[2]
		# 		self.route.poses.append(ps)
		# 	self.route_pub.publish(self.route)

		# if self.create_dataset:
		# 	self.route_raw_pub.publish(self.route_raw)
		# 	self.global_plan_raw_pub.publish(self.global_plan_raw_msg)



		
		#################################################################################################
		###################################### CONTROL ##################################################
		#################################################################################################

		# dtt = (time.time() - self.start_time)
		# if dtt > 60 and  dtt<80:
		#   print ("add error")
		#   self.steering_angle += np.random.normal(0, 0.1)
		# RETURN CONTROL
		control = carla.VehicleControl()
		control.steer = self.steering_angle 
		control.throttle = self.throttle if not self.hand_brake else 0.0
		control.brake = int(self.brake)
		control.hand_brake = self.hand_brake

		# dt = time.time() - t
		# self.last_time = time.time()
		# print ("dt:", dt)
		# time.sleep(max(0.025 - dt, 0))
		return control


	def throttle_cb(self, msg):
			self.throttle = msg.value
			
	def brake_cb(self, msg):
			self.brake = msg.value

	def hand_brake_cb(self, msg):
		self.hand_brake = msg.data

	def steer_cb(self, msg):
			if msg.angle > self.max_steering:
					msg.angle = self.max_steering
			elif msg.angle < -self.max_steering:
					msg.angle = -self.max_steering
			
			self.steering_angle = -msg.angle/self.max_steering

	def norm_ang(self, theta):
		if theta > np.pi:
			theta_n = theta - 2*np.pi
		elif theta < -np.pi:
			theta_n = 2*np.pi + theta
		else:
			theta_n = theta
		return theta_n

	def destroy(self):
			"""
			Destroy (clean-up) the agent
			:return:
			"""
			signal_shutdown = Bool()
			signal_shutdown.data = True
			self.shutdown_pub.publish(signal_shutdown)

			self.update_sensors_running = False

			time.sleep(2.0)

			del self.gps_pub 
			#del self.img_f_pub 
			#del self.img_f_info_pub 
			del self.lidar_pub_front 
			# del self.radar_pub_front 
			# del self.pub_marker_radar 
			# del self.pub_obj_radar

			del self.lidar_pub_back
			del self.vehicle_state_pub 
			del self.route_pub 
			del self.route_raw_pub 
			del self.shutdown_pub

			#subscriber
			del self.throttle_sub
			del self.brake_sub 
			del self.steer_sub 

			del self.ekf_gps 

			self.convert_route_to_way = False
			self.throttle = 0.0
			self.brake = 0
			self.steering_angle = 0.0
			self.last_pose = None
			self.last_gps = None
			self.last_yaw = 0.0
			self.last_gps_index = -1.0
			self.first_y = 0.0

			self.route = Path()

			#rospy.signal_shutdown("finished route")

