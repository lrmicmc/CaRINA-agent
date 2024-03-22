#!/usr/bin/env python3
import os
import carla
import numpy as np
import time
import sys
from threading import Thread
from leaderboard.autoagents.autonomous_agent import AutonomousAgent, Track
from agents.navigation.local_planner import RoadOption
#ros
import rospy
import tf
#messages
from geometry_msgs.msg import PoseWithCovarianceStamped, PoseStamped, Point, Vector3 #TransformStamped,
from std_msgs.msg import Header, Bool #Float64
from sensor_msgs.msg import Image, PointCloud2, CameraInfo, Imu, NavSatFix
from msgs_action.msg import VehicleState, Throttle, Brake, SteeringAngle
from nav_msgs.msg import Path, Odometry#, OccupancyGrid
from rosgraph_msgs.msg import Clock
from msgs_navigation.msg import GlobalPlan
from msgs_mapping.msg import HDMap as HDMapMsg
from msgs_perception.msg import Obstacle, ObstacleArray
from visualization_msgs.msg import Marker, MarkerArray
#pointcloud
from sensor_msgs.point_cloud2 import create_cloud, PointField #create_cloud_xyz32
import xml.etree.ElementTree as ET
import math
import cv2
import yaml

def get_entry_point():
	return 'CarinaAgent'

class CarinaAgent(AutonomousAgent):
	def setup(self, path_to_conf_file):
		print('Setup')
		with open(path_to_conf_file) as f:
			parameters = yaml.full_load(f)
		print("Configuration parameters: ")
		for k, v in parameters.items():
			print(k, v)

		########## stereo camera parameters ##########
		self.fov=parameters["cameras"]["stereo"]["fov"]#122#cameras fov
		# self.fov=30.6             #cameras fov  argo
		self.baseline=parameters["cameras"]["stereo"]["baseline"]#0.24
		self.image_width=parameters["cameras"]["stereo"]["width"]#1600 
		self.image_height=parameters["cameras"]["stereo"]["height"]#1600

		# self.back_image_width=parameters["cameras"]["mono"]["cam_back"]["width"]#1600 
		# self.back_image_height=parameters["cameras"]["mono"]["cam_back"]["height"]#1600
		# self.baseline=0.2986      #argo
		##############################################
		simulator_version = os.environ.get('CARLA_VERSION')
		print('simulator_version',simulator_version)
		track_env = os.environ.get('CHALLENGE_TRACK_CODENAME')
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

		# print('Leaderboard Cersion:', leaderboard)
		print('CHALLENGE_TRACK_CODENAME: ', track_env)
		print('Configured Track:', self.track)

		########## stereo camera parameters ##########
		# self.fov=122#cameras fov
		# # self.fov=30.6             #cameras fov  argo
		# self.baseline=0.24
		# self.image_width=1600 
		# self.image_height=1600
		# self.baseline=0.2986      #argo
		##############################################

		#running nodes
		command_path='/'
		for c in os.path.realpath(__file__).split('/')[0:-1]:
			command_path = os.path.join(command_path, c)
		command_path=os.path.join(command_path, 'execute_carina.sh')

		os.system("bash " + command_path)

		rospy.set_param("/use_sim_time", True)

		rospy.init_node('carina_agent_node', anonymous=True)

		self.clock_ini = rospy.Time.now()#rospy.Time()
		self.clock_publisher = rospy.Publisher('clock', Clock, queue_size=10, latch=True)
		self.clock_publisher.publish(Clock(self.clock_ini))
		self.stamp=None

		self.speed_temp=0.0

		#route
		self.converted_route_to_way = False               
		self.route_raw = Path()
		self.global_plan_msg = GlobalPlan()
  
		#control
		self.throttle = 0.0
		self.brake = 0
		self.steering_angle = 0.0
		self.max_steering = np.deg2rad(30.)
		self.hand_brake = False
		
		#gps
		self.pose = None
		self.last_pose = None

		self.last_gps_index = -1.0
		self.last_gps_gt_index = -1.0

		#sensors
		self.last_image_index = -1.0
		self.last_pc_index = -1.0
 
		self.tree=None
		self.OpenDRIVE_loaded=False

		self.half_pc_old =None

		#publisher
		self.gps_pub = rospy.Publisher('/carina/localization/pose', PoseWithCovarianceStamped, queue_size=1)
		self.lidar_pub_front = rospy.Publisher('/carina/sensor/lidar/front/point_cloud', PointCloud2, queue_size=1)

		self.pub_marker_radar = rospy.Publisher('/carina/perception/radar/obstacles_marker_array', MarkerArray, queue_size=1)
		self.pub_obj_radar = rospy.Publisher('/carina/perception/radar/front/obstacles_array', ObstacleArray, queue_size=1)

		self.vehicle_state_pub = rospy.Publisher('/carina/vehicle/state', VehicleState, queue_size=1)
		
		self.route_pub = rospy.Publisher('/carina/navigation/waypoints', Path, queue_size=1, latch=True)
		self.global_plan_raw_pub = rospy.Publisher('/carina/navigation/global_plan_raw', GlobalPlan, queue_size=1, latch=True)

		self.shutdown_pub = rospy.Publisher('/carina/vehicle/shutdown', Bool, queue_size=1)

		self.odometry_pub = rospy.Publisher('/carina/localization/odom', Odometry, queue_size=1)

		if self.track == Track.MAP:
			self.hdmap_pub = rospy.Publisher('/carina/map/hdmap', HDMapMsg, queue_size=1, latch=True)

		self.clock_pub = rospy.Publisher('/clock', Clock, queue_size=1)
		self.imu_pub = rospy.Publisher('/carina/sensor/imu', Imu, queue_size=1)

		self.nav_sat_fix_pub = rospy.Publisher('/carina/localization/nav_sat_fix', NavSatFix, queue_size=1)
	
		#pub cameras
		self.img_left_pub = rospy.Publisher('/carina/sensor/camera/left/image_raw', Image, queue_size=1)
		self.img_left_rect_color_pub = rospy.Publisher('/carina/sensor/camera/left/image_rect_color', Image, queue_size=1)
		self.img_left_info_pub = rospy.Publisher('/carina/sensor/camera/left/camera_info', CameraInfo,queue_size=1)

		self.img_right_pub = rospy.Publisher('/carina/sensor/camera/right/image_raw', Image, queue_size=1)
		self.img_right_rect_color_pub = rospy.Publisher('/carina/sensor/camera/right/image_rect_color', Image, queue_size=1)
		self.img_right_info_pub = rospy.Publisher('/carina/sensor/camera/right/camera_info', CameraInfo,queue_size=1)

		# self.img_back_rect_color_pub = rospy.Publisher('/carina/sensor/camera/back/image_rect_color', Image, queue_size=1)
		# self.img_back_info_pub = rospy.Publisher('/carina/sensor/camera/back/camera_info', CameraInfo,queue_size=1)

		#subscriber
		self.throttle_sub = rospy.Subscriber('/carina/control/throttle_cmd', Throttle, self.throttle_cb)
		self.brake_sub = rospy.Subscriber('/carina/control/brake_cmd', Brake, self.brake_cb)
		self.steer_sub = rospy.Subscriber('/carina/control/steer_cmd', SteeringAngle, self.steer_cb)
		self.hand_brake_sub = rospy.Subscriber('/carina/control/hand_brake_cmd', Bool, self.hand_brake_cb)

		self.pose_corrected_sub = rospy.Subscriber('/localization/ekf/global', PoseWithCovarianceStamped, self.pose_corrected_cb)

	def sensors(self):
		sensors = []

		if self.track == Track.MAP:
			OpenDRIVE={'type': 'sensor.opendrive_map', 'reading_frequency': 1, 'id': 'OpenDRIVE'}
			sensors.append(OpenDRIVE)

		CameraLeft={'type': 'sensor.camera.rgb', 'x':0, 'y':0.0, 'z':2.8, 'roll':0.0, 'pitch':0.0, 'yaw':0.0,
						'width':self.image_width, 'height': self.image_height, 'fov':self.fov, 'sensor_tick': 0.0, 'id': 'CameraLeft'}
		sensors.append(CameraLeft)	

		CameraRight={'type': 'sensor.camera.rgb', 'x':0., 'y':self.baseline, 'z':2.8, 'roll':0.0, 'pitch':0.0, 'yaw':0.0,
						'width':self.image_width, 'height': self.image_height, 'fov':self.fov, 'sensor_tick': 0.0, 'id': 'CameraRight'}
		sensors.append(CameraRight)

		# CameraBack={'type': 'sensor.camera.rgb', 'x':0., 'y':self.baseline, 'z':2.8, 'roll':0.0, 'pitch':0.0, 'yaw':180.0,
		# 				'width': self.back_image_width, 'height': self.back_image_height, 'fov':self.fov, 'sensor_tick': 0.0, 'id': 'CameraBack'}
		# sensors.append(CameraBack)

		LIDAR={'type': 'sensor.lidar.ray_cast', 'x': 0., 'y': 0.0, 'z': 2.25, 'roll': 0.0, 'pitch': 0.0,
						'yaw': 0.0, 'channels':64,'points_per_second':350000, 'upper_fov':5.0,
						 'lower_fov':-20.0, 'range': 10000, 'sensor_tick': 0.0, 'rotation_frequency':30.0, 'id': 'LIDAR'}
		sensors.append(LIDAR)
        #       		 
		SPEED={'type': 'sensor.speedometer',  'reading_frequency': 20, 'id': 'SPEED'}   
		sensors.append(SPEED)		                          

		IMU={'type': 'sensor.other.imu', 'x': 0.0, 'y': 0.0, 'z': 2.0, 'roll': 0.0, 'pitch': 0.0, 'yaw': 0.0, 'id': 'IMU'}
		sensors.append(	IMU)	

		GPS={'type': 'sensor.other.gnss', 'x': 0.0, 'y': 0.0, 'z': 2.0, 'reading_frequency': 20, 'id': 'GPS'}
		sensors.append(GPS)	

		return sensors

	def update_cameras(self, sensor_data):
		#################################################################################################
		###################################### CAMERAS ##################################################
		#################################################################################################
		if sensor_data['CameraLeft'][0] > self.last_image_index:

			self.last_image_index = sensor_data['CameraLeft'][0]
			stamp = self.stamp#rospy.Time().now()

			#LEFT
			img_left = sensor_data['CameraLeft'][1]
			img_left = img_left[:,:,0:3]
			img_left = img_left[:,:,::-1]
			img_left_flat = np.reshape(img_left, (1, img_left.shape[0]*img_left.shape[1]*3))

			img_left_msg = Image()
			img_left_msg.header.stamp = stamp
			img_left_msg.header.frame_id = "stereo"
			img_left_msg.height = img_left.shape[0]
			img_left_msg.width = img_left.shape[1]
			img_left_msg.encoding = 'rgb8'
			img_left_msg.step=img_left.shape[1]*3
			img_left_msg.data=img_left_flat[0].tolist()
	
			camera_info_left= CameraInfo()
			camera_info_left.header.frame_id = "stereo"
			camera_info_left.header.stamp = stamp
			camera_info_left.width = img_left_msg.width
			camera_info_left.height = img_left_msg.height
			camera_info_left.distortion_model='plumb_bob'
			cx = camera_info_left.width/2.0
			cy = camera_info_left.height/2.0
			fx2 = camera_info_left.width / (2.0 * np.tan(self.fov * np.pi / 360.0)) #fov=50 (sensor configuration)
			fy2 = fx2
			camera_info_left.K = [fx2, 0, cx, 0, fy2, cy, 0, 0, 1]
			camera_info_left.D = [0, 0, 0, 0, 0]
			camera_info_left.R = [1.0, 0, 0, 0, 1.0, 0, 0, 0, 1.0]
			camera_info_left.P = [fx2, 0, cx, 0, 0, fy2, cy, 0, 0, 0, 1.0, 0]

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
			camera_info_right.K = [fx2, 0, cx, 0, fy2, cy, 0, 0, 1]
			camera_info_right.D = [0, 0, 0, 0, 0]
			camera_info_right.R = [1.0, 0, 0, 0, 1.0, 0, 0, 0, 1.0]
			camera_info_right.P = [fx2, 0, cx, -fx2*self.baseline, 0, fy2, cy, 0, 0, 0, 1.0, 0]


			# #BACK
			# img_back = sensor_data['CameraBack'][1]
			# img_back = img_back[:,:,0:3]
			# img_back = img_back[:,:,::-1]
			# img_back_flat = np.reshape(img_back, (1, img_back.shape[0]*img_back.shape[1]*3))

			# img_back_msg = Image()
			# img_back_msg.header.stamp = stamp
			# img_back_msg.header.frame_id = "stereo"
			# img_back_msg.height = img_back.shape[0]
			# img_back_msg.width = img_back.shape[1]
			# img_back_msg.encoding = 'rgb8'
			# img_back_msg.step=img_back.shape[1]*3
			# img_back_msg.data=img_back_flat[0].tolist()
	
			# camera_info_back= CameraInfo()
			# camera_info_back.header.frame_id = "stereo"
			# camera_info_back.header.stamp = stamp
			# camera_info_back.width = img_back_msg.width
			# camera_info_back.height = img_back_msg.height
			# camera_info_back.distortion_model='plumb_bob'
			# cx = camera_info_back.width/2.0
			# cy = camera_info_back.height/2.0
			# fx2 = camera_info_back.width / (2.0 * np.tan(self.fov * np.pi / 360.0)) #fov=50 (sensor configuration)
			# fy2 = fx2
			# camera_info_back.K = [fx2, 0, cx, 0, fy2, cy, 0, 0, 1]
			# camera_info_back.D = [0, 0, 0, 0, 0]
			# camera_info_back.R = [1.0, 0, 0, 0, 1.0, 0, 0, 0, 1.0]
			# camera_info_back.P = [fx2, 0, cx, 0, 0, fy2, cy, 0, 0, 0, 1.0, 0]
			
			#pubs
			self.img_left_pub.publish(img_left_msg)     
			self.img_left_rect_color_pub.publish(img_left_msg)     
			self.img_left_info_pub.publish(camera_info_left)
			self.img_right_pub.publish(img_right_msg) 
			self.img_right_rect_color_pub.publish(img_right_msg)             
			self.img_right_info_pub.publish(camera_info_right)

			# self.img_back_rect_color_pub.publish(img_back_msg)             
			# self.img_back_info_pub.publish(camera_info_back)

			# if self.create_dataset_depth==True:

			# 	#depth
			# 	img_depth = sensor_data['depthCameraLeft'][1]
			# 	img_depth = img_depth[:,:,0:3]
			# 	img_depth = img_depth[:,:,::-1]
			# 	img_depth_flat = np.reshape(img_depth, (1, img_depth.shape[0]*img_depth.shape[1]*3))

			# 	img_depth_msg = Image()
			# 	img_depth_msg.header.stamp = stamp
			# 	img_depth_msg.header.frame_id = "stereo"
			# 	img_depth_msg.height = img_depth.shape[0]
			# 	img_depth_msg.width = img_depth.shape[1]
			# 	img_depth_msg.encoding = 'rgb8'
			# 	img_depth_msg.step=img_depth.shape[1]*3
			# 	img_depth_msg.data=img_depth_flat[0].tolist()
		
			# 	camera_info_depth= CameraInfo()
			# 	camera_info_depth.header.frame_id = "stereo"
			# 	camera_info_depth.header.stamp = stamp
			# 	camera_info_depth.width = img_depth_msg.width
			# 	camera_info_depth.height = img_depth_msg.height
			# 	camera_info_depth.distortion_model='plumb_bob'
			# 	cx = camera_info_depth.width/2.0
			# 	cy = camera_info_depth.height/2.0
			# 	fx2 = camera_info_depth.width / (2.0 * np.tan(self.fov * np.pi / 360.0)) #fov=50 (sensor configuration)
			# 	fy2 = fx2
			# 	camera_info_depth.K = [fx2, 0, cx, 0, fy2, cy, 0, 0, 1]
			# 	camera_info_depth.D = [0, 0, 0, 0, 0]
			# 	camera_info_depth.R = [1.0, 0, 0, 0, 1.0, 0, 0, 0, 1.0]
			# 	camera_info_depth.P = [fx2, 0, cx, 0, 0, fy2, cy, 0, 0, 0, 1.0, 0]

			# 	img_depth_r = sensor_data['depthCameraRight'][1]
			# 	img_depth_r = img_depth_r[:,:,0:3]
			# 	img_depth_r = img_depth_r[:,:,::-1]
			# 	img_depth_r_flat = np.reshape(img_depth_r, (1, img_depth_r.shape[0]*img_depth_r.shape[1]*3))

			# 	img_depth_r_msg = Image()
			# 	img_depth_r_msg.header.stamp = stamp
			# 	img_depth_r_msg.header.frame_id = "stereo"
			# 	img_depth_r_msg.height = img_depth_r.shape[0]
			# 	img_depth_r_msg.width = img_depth_r.shape[1]
			# 	img_depth_r_msg.encoding = 'rgb8'
			# 	img_depth_r_msg.step=img_depth_r.shape[1]*3
			# 	img_depth_r_msg.data=img_depth_r_flat[0].tolist()
		
			# 	camera_info_depth_r= CameraInfo()
			# 	camera_info_depth_r.header.frame_id = "stereo"
			# 	camera_info_depth_r.header.stamp = stamp
			# 	camera_info_depth_r.width = img_depth_r_msg.width
			# 	camera_info_depth_r.height = img_depth_r_msg.height
			# 	camera_info_depth_r.distortion_model='plumb_bob'
			# 	cx = camera_info_depth_r.width/2.0
			# 	cy = camera_info_depth_r.height/2.0
			# 	fx2 = camera_info_depth_r.width / (2.0 * np.tan(self.fov * np.pi / 360.0)) #fov=50 (sensor configuration)
			# 	fy2 = fx2
			# 	camera_info_depth_r.K = [fx2, 0, cx, 0, fy2, cy, 0, 0, 1]
			# 	camera_info_depth_r.D = [0, 0, 0, 0, 0]
			# 	camera_info_depth_r.R = [1.0, 0, 0, 0, 1.0, 0, 0, 0, 1.0]
			# 	camera_info_depth_r.P = [fx2, 0, cx, -fx2*self.baseline, 0, fy2, cy, 0, 0, 0, 1.0, 0]


			# 	self.img_depth_pub.publish(img_depth_msg)     
			# 	self.img_depth_info_pub.publish(camera_info_depth)

			# 	self.img_depth_r_pub.publish(img_depth_r_msg)     
			# 	self.img_depth_r_info_pub.publish(camera_info_depth_r)

				# stampnow = rospy.Time().now()
				# # save left disp
				# cv2.imwrite('/mnt/Data2/carla_depth_dataset/CarlaDisp/training/rgb/left/'+str(stampnow)+'.jpg', cv2.cvtColor(img_left, cv2.COLOR_RGB2BGR))
				# cv2.imwrite('/mnt/Data2/carla_depth_dataset/CarlaDisp/training/rgb/right/'+str(stampnow)+'.jpg', cv2.cvtColor(img_right, cv2.COLOR_RGB2BGR))
				# cv2.imwrite('/mnt/Data2/carla_depth_dataset/CarlaDisp/training/semantic/left/'+str(stampnow)+'.png', cv2.cvtColor(img_semantic, cv2.COLOR_RGB2BGR))
				# cv2.imwrite('/mnt/Data2/carla_depth_dataset/depth/'+str(stamp)+'.png',	  cv2.cvtColor(img_depth, cv2.COLOR_RGB2BGR))

				# R=img_depth[:,:,0]
				# G=img_depth[:,:,1]
				# B=img_depth[:,:,2]

				# R=R.astype(np.float32)
				# G=G.astype(np.float32)
				# B=B.astype(np.float32)

				# normalized = (R + G * 256 + B * 256 * 256) / (256 * 256 * 256 - 1)
				# in_meters = 1000 * normalized
				# Z=in_meters
				# B=self.baseline#0.2986
				# fov=self.fov #30.6
				# # print(depth.shape)
				# f=fx2#depth.shape[1]/ (2.0 * np.tan(fov * np.pi / 360.0)) 
				# disparity=B*f/Z
				# flowy=np.zeros_like(disparity)

				# flowxy=np.dstack((disparity,flowy))
				# flowxy=flowxy*-1
				# # print(np.max(flowxy))
				# # print(np.min(flowxy))

				# flowxy=flowxy.astype(np.float32)

				# visualize_flow(flowxy, '/mnt/Data2/carla_depth_dataset/CarlaDisp/training/flow_viz/left/'+str(stamp)+'.png')
				# write_flow(flowxy, '/mnt/Data2/carla_depth_dataset/CarlaDisp/training/flow/left/'+str(stampnow)+'.flo')

				# # save Right disp
				# R=img_depth_r[:,:,0]
				# G=img_depth_r[:,:,1]
				# B=img_depth_r[:,:,2]

				# R=R.astype(np.float32)
				# G=G.astype(np.float32)
				# B=B.astype(np.float32)

				# normalized = (R + G * 256 + B * 256 * 256) / (256 * 256 * 256 - 1)
				# in_meters = 1000 * normalized
				# Z=in_meters
				# B=self.baseline#0.2986
				# fov=self.fov #30.6
				# # print(depth.shape)
				# f=fx2#depth.shape[1]/ (2.0 * np.tan(fov * np.pi / 360.0)) 
				# disparity=B*f/Z
				# flowy=np.zeros_like(disparity)

				# flowxy=np.dstack((disparity,flowy))
				# flowxy=flowxy
				# flowxy=flowxy.astype(np.float32)
		
				# visualize_flow(flowxy, '/mnt/Data2/carla_depth_dataset/CarlaDisp/training/flow_viz/right/'+str(stamp)+'.png')
				# write_flow(flowxy, '/mnt/Data2/carla_depth_dataset/CarlaDisp/training/flow/right/'+str(stampnow)+'.flo')

	def update_radar(self, sensor_data):
		stamp=self.stamp
		if self.pub_obstacles.get_num_connections() >  0 :
			markerArray = MarkerArray()
			obstacle_Array = ObstacleArray()
			markerArray.markers=[]
			obstacle_Array.obstacle=[]
			t = rospy.Duration(0.35)  
			id_obj=0              
			if sensor_data['obstacles'][0] > self.last_radar_index:
				self.last_radar_index = sensor_data['obstacles'][0]
				for o in sensor_data['obstacles'][1]:
					#[depth, azimuth, altitute, velocity] ###for carla leaderboard
					depth=o[0]
					altitude=o[1]
					azimuth=o[2]
					# velocity=o[3]

					x = depth * np.cos(altitude) * np.cos(azimuth)
					y = depth * np.cos(altitude) * np.sin(azimuth)
					z = depth * np.sin(altitude)

					if self.pub_marker_radar.get_num_connections() > 0:

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
						marker.id =id_obj

						markerArray.markers.append(marker)

					obj = Obstacle()
					obj.header.frame_id = "velodyne"
					obj.header.stamp = stamp
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

					obj.color.r = 255
					obj.color.g = 0
					obj.color.b = 0
					obj.color.a = 255

					obj.track_status = 1   ############## important 1
					obj.lifetime = t
					obj.type = -1
					obj.animation_speed = 0.5;

					# markerArray.markers.append(marker)
					obstacle_Array.obstacle.append(obj)

					id_obj+=1
			if self.pub_marker_radar.get_num_connections() > 0:
				self.pub_marker_obstacles.publish(markerArray)
			self.pub_obstacles.publish(obstacle_Array)

	#################################################################################################
	####################################### LIDAR ###################################################
	#################################################################################################
	def update_lidar(self, sensor_data):
		#Lidar                  
		stamp=self.stamp
		if sensor_data['LIDAR'][0] > self.last_pc_index:
			self.last_pc_index = sensor_data['LIDAR'][0]
			pc = None
			if self.lidar_pub_front.get_num_connections() > 0:
				pc = sensor_data['LIDAR'][1]
				header = Header()
				header.stamp = stamp
				header.frame_id='velodyne'
				#pc = np.concatenate((-pc, -pc + np.random.normal(0, 0.0001, pc.shape)), axis=0)                
				pc = pc[..., [0,1,2,3]]
				# we take the opposite of y axis
				# (as lidar point are express in left handed coordinate system, and ros need right handed)
				pc[:,1]=-pc[:,1]
				# point_front=np.array([  
									  # [4.4, 0.5,-2.25, 1], [4.4, 0.4,-2.25, 1], [4.4, 0.3,-2.25, 1], [4.4, 0.2,-2.25, 1], [4.4, 0.1,-2.25, 1],[4.4,0.0,-2.25, 1],
									  # [4.4,-0.5,-2.25, 1], [4.4,-0.4,-2.25, 1], [4.4,-0.3,-2.25, 1], [4.4,-0.2,-2.25, 1], [4.4,-0.1,-2.25, 1],
				            		  # [4.2, 0.4,-2.25, 1], [4.2, 0.3,-2.25, 1], [4.2, 0.2,-2.25, 1], [4.2, 0.1,-2.25, 1], [4.2, 0.0,-2.25, 1], 
									# 				       [4.2,-0.4,-2.25, 1], [4.2,-0.3,-2.25, 1], [4.2,-0.2,-2.25, 1], [4.2,-0.1,-2.25, 1],
									  # [4.0, 0.5,-2.25, 1], [4.0, 0.4,-2.25, 1], [4.0, 0.3,-2.25, 1], [4.0, 0.2,-2.25, 1], [4.0, 0.1,-2.25, 1],[4.0,0.0,-2.25, 1],
									  # [4.0,-0.5,-2.25, 1], [4.0,-0.4,-2.25, 1], [4.0,-0.3,-2.25, 1], [4.0,-0.2,-2.25, 1], [4.0,-0.1,-2.25, 1], 
									  # [3.8, 0.5,-2.25, 1], [3.8, 0.4,-2.25, 1], [3.8, 0.3,-2.25, 1], [3.8, 0.2,-2.25, 1], [3.8, 0.1,-2.25, 1],[3.8,0.0,-2.25, 1],
									  # [3.8,-0.5,-2.25, 1], [3.8,-0.4,-2.25, 1], [3.8,-0.3,-2.25, 1], [3.8,-0.2,-2.25, 1], [3.8,-0.1,-2.25, 1],
				            		  # [3.7, 0.4,-2.25, 1], [3.7, 0.3,-2.25, 1], [3.7, 0.2,-2.25, 1], [3.7, 0.1,-2.25, 1], [3.7, 0.0,-2.25, 1], 
									# 				       [3.7,-0.4,-2.25, 1], [3.7,-0.3,-2.25, 1], [3.7,-0.2,-2.25, 1], [3.7,-0.1,-2.25, 1],
									  # [3.6, 0.5,-2.25, 1], [3.6, 0.4,-2.25, 1], [3.6, 0.3,-2.25, 1], [3.6, 0.2,-2.25, 1], [3.6, 0.1,-2.25, 1],[3.6,0.0,-2.25, 1],
									  # [3.6,-0.5,-2.25, 1], [3.6,-0.4,-2.25, 1], [3.6,-0.3,-2.25, 1], [3.6,-0.2,-2.25, 1], [3.6,-0.1,-2.25, 1]
									  # ])
				# pc = np.concatenate((pc, point_front), axis=0) 

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

	#################################################################################################
	####################################### ROUTE ###################################################
	#################################################################################################				
	def routeToWaypoints(self, route):
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
			utm_data = self.geodesicToMercator(self.lat_ref, self.lon_ref, lat, lon, z)

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

	def routeWorldToWaypoints(self, route):		
		stamp=self.stamp
		waypoints = []
		msg = GlobalPlan()
		msg.header.stamp = stamp#rospy.Time().now()
		msg.header.frame_id = 'map'
		msg.points = []
		msg.road_options = []

		for way in route:
			p = Point()
			p.x = way[0].location.x
			p.y = way[0].location.y
			p.z = way[0].location.z
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
			waypoints.append([p.x, p.y, p.z, road_opt])
		waypoints = np.asarray(waypoints) 
		return np.asarray(waypoints), msg

	#################################################################################################
	######################################## GPS ####################################################
	#################################################################################################
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
		return np.array([utm_data[1], utm_data[2], gps[2]])

	#################################################################################################
	######################################## HD-MAP##################################################
	#################################################################################################
	def processHDMAp(self, sensor_data):
		stamp=self.stamp

		msg = HDMapMsg()
		msg.header.stamp = stamp
		msg.header.frame_id = 'map'
		try:
			msg.XML_HDMap = sensor_data['OpenDRIVE'][1]['opendrive']
		except:
			return
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

		self.OpenDRIVE_loaded=True
		# if self.create_dataset_path == False:
		# print('publish hdmap')
		self.hdmap_pub.publish(msg)


	def run_step(self, input_data, timestamp):
		#################################################################################################
		#################################### UPDATE CLOCK ###############################################
		#################################################################################################
		clock_msg = Clock()
		self.stamp = self.clock_ini + rospy.Duration.from_sec(timestamp)#rospy.Time.Duration(timestamp)
		clock_msg.clock = self.stamp
		self.clock_pub.publish(clock_msg)
		stamp = self.stamp

		#################################################################################################
		####################################### HDMAP ###################################################
		#################################################################################################
		if self.track == Track.MAP and not self.OpenDRIVE_loaded  and  'OpenDRIVE' in 	input_data.keys():  
			self.processHDMAp(input_data)

		#################################################################################################
		####################################### ROUTE ###################################################
		#################################################################################################
		if not self.converted_route_to_way and self.lat_ref is not None:

			self.waypoints, self.global_plan_msg = self.routeToWaypoints(self._global_plan)
			self.route_raw = Path()
			self.route_raw.header.stamp = stamp#rospy.Time().now()
			self.route_raw.header.frame_id = 'map'
			self.route_raw.poses = []
			for p in self.waypoints:
				ps = PoseStamped()
				ps.header.frame_id = 'map'
				ps.header.stamp = stamp#rospy.Time().now()
				ps.pose.position.x = p[0]
				ps.pose.position.y = p[1]
				ps.pose.position.z = p[2]
				self.route_raw.poses.append(ps)
			self.converted_route_to_way=True

		if self.converted_route_to_way:
			# print('publishing route and global plan')
			self.global_plan_raw_pub.publish(self.global_plan_msg)
			self.route_pub.publish(self.route_raw)
		#################################################################################################
		######################################## IMU ####################################################
		#################################################################################################   

		m_imu = Imu()
		m_imu.header.stamp=stamp#rospy.Time().now()
		m_imu.header.frame_id="imu"

		yaw = -input_data['IMU'][1][-1]
		#m_imu.orientation=-input_data['IMU'][1][-1]
		m_imu_ori= tf.transformations.quaternion_from_euler(0.0, 0.0, yaw)

		m_imu.orientation.x = m_imu_ori[0]
		m_imu.orientation.y = m_imu_ori[1]
		m_imu.orientation.z = m_imu_ori[2]
		m_imu.orientation.w = m_imu_ori[3]
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
		self.imu=m_imu                                        
		self.imu_pub.publish(m_imu)
  
		#################################################################################################
		######################################## GPS ####################################################
		#################################################################################################
		yaw_n=yaw

		ori = tf.transformations.quaternion_from_euler(0.0, 0.0, yaw_n)		

		if input_data['GPS'][0] > self.last_gps_index and self.datum is not None:# and stamp.to_sec() > 5: #wait 5 secs to publush pose and transforms
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

		if self.pose is not None:
			self.gps_pub.publish(self.pose)

		if self.lat_ref is None or stamp.to_sec() < 5.0: #wait 20.0 secs to start
				control=carla.VehicleControl()
				control.hand_brake = True
				return control
	
		#################################################################################################
		################################### UPDATE SENSORS ##############################################
		#################################################################################################
		thread_cameras = Thread(target=self.update_cameras, args=[input_data])
		thread_cameras.start()
		thread_lidar = Thread(target=self.update_lidar, args=[input_data])
		thread_lidar.start()

		#################################################################################################
		####################################### STATE ###################################################
		#################################################################################################
		vehicle_state_msg = VehicleState()
		vehicle_state_msg.header.stamp = stamp
		vehicle_state_msg.drive.speed=abs(input_data['SPEED'][1]['speed'])
		self.speed_temp=vehicle_state_msg.drive.speed
		vehicle_state_msg.drive.steering_angle = -self.steering_angle*self.max_steering
		vehicle_state_msg.handbrake=self.hand_brake#0
		vehicle_state_msg.brake=int(self.brake)#0
		self.vehicle_state_pub.publish(vehicle_state_msg)

		#################################################################################################
		###################################### CONTROL ##################################################
		#################################################################################################
		control = carla.VehicleControl()
		control.steer = self.steering_angle
		control.throttle = self.throttle if not self.hand_brake else 0.0
		control.brake = int(self.brake)
		control.hand_brake = self.hand_brake
		return control

	def pose_corrected_cb(self, msg):
		r, p, y = tf.transformations.euler_from_quaternion([self.imu.orientation.x, self.imu.orientation.y, self.imu.orientation.z, self.imu.orientation.w]) 
		pose_msg = PoseWithCovarianceStamped()
		pose_msg.header.stamp = self.stamp#msg.header.stamp
		pose_msg.header.frame_id = 'map'#msg.header.frame_id
		pose_msg.pose.pose.position.x = msg.pose.pose.position.x #- 1.2*math.cos(y)
		pose_msg.pose.pose.position.y = msg.pose.pose.position.y #- 1.2*math.sin(y)
		pose_msg.pose.pose.position.z = msg.pose.pose.position.z

		pose_msg.pose.pose.orientation.x = self.imu.orientation.x
		pose_msg.pose.pose.orientation.y = self.imu.orientation.y
		pose_msg.pose.pose.orientation.z = self.imu.orientation.z
		pose_msg.pose.pose.orientation.w = self.imu.orientation.w

		# pose_msg.pose.pose.orientation.x = msg.pose.pose.orientation.x
		# pose_msg.pose.pose.orientation.y = msg.pose.pose.orientation.y
		# pose_msg.pose.pose.orientation.z = msg.pose.pose.orientation.z
		# pose_msg.pose.pose.orientation.w = msg.pose.pose.orientation.w
	
		if self.stamp.to_sec() >2.0:
			self.pose=pose_msg


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

	def destroy(self):
			"""
			Destroy (clean-up) the agent
			:return:
			"""
			signal_shutdown = Bool()
			signal_shutdown.data = True
			self.shutdown_pub.publish(signal_shutdown)
			stamp=self.stamp
			
			self.route_raw = Path()
			self.route_raw.header.stamp = stamp#rospy.Time().now()
			self.route_raw.header.frame_id = 'map'
			self.route_raw.poses = []

			self.global_plan_msg = GlobalPlan()
			self.global_plan_msg.header.stamp = stamp#rospy.Time().now()
			self.global_plan_msg.header.frame_id = 'map'
			self.global_plan_msg.points = []

			self.global_plan_raw_pub.publish(self.global_plan_msg)
			self.route_pub.publish(self.route_raw)

			if self.track == Track.MAP:
				stamp=self.stamp
				msg = HDMapMsg()
				msg.header.stamp = stamp
				msg.header.frame_id = 'map'
				msg.XML_HDMap = ""
				self.hdmap_pub.publish(msg)

			self.update_sensors_running = False

			time.sleep(2.0)

			self.converted_route_to_way = False
			self.throttle = 0.0
			self.brake = 0
			self.steering_angle = 0.0
			self.last_pose = None
			self.last_gps_index = -1.0

		#publisher
			del self.pub_marker_radar
			del self.pub_obj_radar 
			del self.lidar_pub_front 
			del self.vehicle_state_pub 
			del self.route_pub 
			del self.shutdown_pub
			del self.gps_pub 
			del self.odometry_pub 
			del self.global_plan_raw_pub 

			if self.track == Track.MAP:
				del self.hdmap_pub 

			del self.clock_pub 
			del self.imu_pub 
			del self.nav_sat_fix_pub 

			#pub cameras
			del self.img_left_pub 
			del self.img_left_rect_color_pub 
			del self.img_left_info_pub 
			del self.img_right_pub 
			del self.img_right_rect_color_pub 
			del self.img_right_info_pub 
			# del self.img_back_rect_color_pub 
			# del self.img_back_info_pub 
			#subscriber
			del self.throttle_sub
			del self.brake_sub 
			del self.steer_sub 
			del self.hand_brake_sub 
			del self.pose_corrected_sub 



