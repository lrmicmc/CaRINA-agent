#!/usr/bin/env python3
from __future__ import print_function, division

from argparse import ArgumentParser

# from mmseg.apis import inference_segmentor, init_segmentor, show_result_pyplot
from inference_fpn import inference_segmentor, init_segmentor, show_result_pyplot

from mmseg.core.evaluation import get_palette




# from typing import Dict

# from tempfile import gettempdir
import matplotlib.pyplot as plt
import numpy as np
import torch
from torch import nn, optim
from torch.utils.data import DataLoader
# from torchvision.models.resnet import resnet50
from torchvision.models.resnet import resnet18

#from tqdm import tqdm

# from prettytable import PrettyTable
# from pathlib import Path


import os
import torch
# import pandas as pd 
from skimage import io, transform
import numpy as np 
import matplotlib.pyplot as plt 
from torch.utils.data import Dataset, DataLoader 
from torchvision import transforms, utils 
import numpy as np

import os

import argparse
from torch.optim.lr_scheduler import StepLR

import math



import numpy as np
import rospy
import time
from geometry_msgs.msg import PoseWithCovarianceStamped, Twist, PoseStamped, TransformStamped
from msgs_action.msg import VehicleState, Throttle, Brake, SteeringAngle

# from msgs_perception.msg import Obstacle, ObstacleArray
from std_msgs.msg import Float64, Bool
# from msgs_navigation.msg import Path
from nav_msgs.msg import Path as RosPath
from msgs_navigation.msg import Path as NavPath
from msgs_navigation.msg import TrajectoryPoint

from nav_msgs.msg import OccupancyGrid

# from msgs_traffic.msg import TrafficSign, TrafficSignArray

# from abt import *
# from model_POMDP_v5 import *
from sensor_msgs.msg import Image, PointCloud2, CameraInfo

# from sensor_msgs.msg import PointCloud2
import ros_numpy

from threading import Thread

import tf2_ros
from tf2_geometry_msgs import *
from tf.transformations import *
import tf
from std_msgs.msg import Header
from geometry_msgs.msg import Transform, Vector3, Quaternion

import cv2

from collections import deque 

from cv_bridge import CvBridge, CvBridgeError
# from sensor_msgs.msg import Image

from msgs_navigation.msg import GlobalPlan

from visualization_msgs.msg import Marker
from visualization_msgs.msg import MarkerArray

from pathlib import Path
import os

from image_geometry import PinholeCameraModel

from path_optimizer import *

from scipy import interpolate

import rospkg


import time
class CreateDatasetCarla(object):
	def __init__ (self):
		# img=

		rospack = rospkg.RosPack()
		dir_pkg=rospack.get_path('fpn_planner')#+'/weights/yolov3-tiny_140000.weights'
		# config='/home/luis/mmsegmentation/configs/AD_fpn/fpn_r50_700X700_.py'
		# checkpoint='/home/luis/mmsegmentation/work_dirs/fpn_r50_700X700_/latest.pth'
		# model_name='fpn_r50_700X700_'
		checkpoint = os.path.join(dir_pkg,'work_dirs','latest.pth')
		# folder_config='AD_fpn'
		config=      os.path.join(dir_pkg,'work_dirs','fpn_r50_700X700_.py')
		
		device='cuda:0'
		# palette='cityscapes'
		opacity=0.5
		self.palette = [
	     [70, 70, 70],  # 'Building'
	    [102, 102, 156],  #   'Fence'
	    [220, 20, 60],    # 'Pedestrian'
	     [6, 255, 0],     #'RoadLine'
	     [128, 64, 128],  #   'Road'
	    [244, 35, 232],   #  'SideWalk'
	    [107, 142, 35],   #   'Vegetation'
	    [0, 0, 142],     #'Vehicles'
	    [102, 102, 156],  #   'Wall'
	    [14, 10, 0],   #  'Ground'
	    [17, 255, 0],   #  'GuardRail'
	    [191, 255, 0],   #  'Static'
	    [201, 0, 26],    # 'Dynamic'
	    [152, 251, 152]]    #'Terrain'

		self.tf2_buffer_blink2odom = tf2_ros.Buffer()
		self.tf2_listener_blink2odom = tf2_ros.TransformListener(self.tf2_buffer_blink2odom)

		# build the model from a config file and a checkpoint file
		self.model = init_segmentor(config, checkpoint, device=device)
		# test a single image

		self.pose_sub = rospy.Subscriber('/carina/localization/pose', PoseWithCovarianceStamped, self.pose_cb, queue_size=1)
		self.state_sub = rospy.Subscriber('/carina/vehicle/state', VehicleState, self.vehicle_state_cb, queue_size=1)

		self.image_bev_sub =        rospy.Subscriber('/carina/sensor/lidar/bev_point_cloud', Image, self.lidar_bev_imageCallback, queue_size=1)
		self.image_bev_stereo_sub = rospy.Subscriber('/carina/sensor/stereo/bev_rgb_point_cloud', Image, self.stereo_bev_imageCallback, queue_size=1)

		self.global_plan_raw_sub = rospy.Subscriber('/carina/navigation/global_plan_raw', GlobalPlan, self.global_plan_cb, queue_size=1)

		self.shutdown_sub = rospy.Subscriber('/carina/vehicle/shutdown', Bool, self.shutdown_cb, queue_size=1)


		### using map and cnn planning### self.shutdown_using_map_sub = rospy.Subscriber('/carina/map/shutdown_using_map', Bool, self.shutdown_using_map_cb, queue_size=1)



		# self.shutdown_shutdown_first_point = rospy.Subscriber('/carina/map/shutdown_first_point', Bool, self.shutdown_first_point, queue_size=1)

		self.first_point_achived = rospy.Publisher('/carina/map/first_point_achived', Bool, queue_size=1)		# self.radar_pub_front = rospy.Publisher('/carina/sensor/radar/front/obstacles_array', ObstacleArray, queue_size=1)

		self.marker_route_pub = rospy.Publisher('/carina/route/points/route_points_array', MarkerArray, queue_size=1)

		# self.path_ros_pub = rospy.Publisher('/carina/navigation/cnn_path_ros_fpn', RosPath, queue_size=1)#same than local_path_planing_node
		# self.path_pub = rospy.Publisher("/carina/navigation/cnn_path_fpn", NavPath,queue_size=1)#same than local_path_planing_node

		self.path_ros_pub = rospy.Publisher('/carina/navigation/cnn_path_ros', RosPath, queue_size=1)#same than local_path_planing_node
		self.path_pub = rospy.Publisher("/carina/navigation/cnn_path", NavPath,queue_size=1)#same than local_path_planing_node

		self.pub_intersections_obj = rospy.Publisher('/carina/navigation/next_intersection', MarkerArray, queue_size=1)

		self.seg_image_pub = rospy.Publisher("/carina/navigation/planner/seg_image",Image, queue_size=1)
		self.occupation_map_pub = rospy.Publisher('/carina/map/OccupancyGrid', OccupancyGrid, queue_size=10, latch=True)

		# self.path_pub = rospy.Publisher('/carina/navigation/path', Path, queue_size=1)
		# self.path_ros_pub = rospy.Publisher('/carina/navigation/path_ros', RosPath, queue_size=1)
		self.stamp=rospy.Time.now()

		self.steering_angle=None
		self.speed=None

		self.throttle=None
		self.brake=None
		self.hand_brake=None
		self.points_lidar=None
		self.points_stereo=None
		self.current_pose=None
		self.old_pose=None
		self.old_poses_list=None
		self.cvbridge = CvBridge()
		self.rgb_image =None
		self.depth_image =None

		self.rgb_cam_info=None
		self.stereo_bev_image =None
		self.lidar_bev_image =None
		self.seg_image=None
		self.global_plan=None

		self.last_path_time=rospy.Time.now()#rospy.to_sec()

		self.max_steering = np.deg2rad(30.)

		self.next_index_goal_plan=0
		self.len_global_plan=None
		self.visited_points_global_plan=[]


		self.LANEFOLLOW=0
		self.STRAIGHT=1
		self.RIGHT=2
		self.LEFT=3
		self.CHANGELANELEFT=4
		self.CHANGELANERIGHT=5
		self.UNKNOWN=6

		self.losses_train = []
		self.first_frame=True
		self.waypoints = np.array([])
		self.precision = 0.2

		self.window = 50

		self.index = 0

		self.msg_path_p = RosPath()
		self.msg_path_p.header.frame_id='map'
		self.msg_path_p.poses = []

		self.path_msg = NavPath()
		self.path_msg.header.frame_id = 'map'
		self.path_msg.path = []

		self.markerArray = MarkerArray()
		self.markerArray.markers=[]

		self.intersections_markerArray = MarkerArray()
		self.intersections_markerArray.markers=[]


		channels=11
		# predicted_points=200

	def global_plan_cb(self,msg):
		self.global_plan=msg
		# print('len ',len(self.global_plan.points))
		self.len_global_plan=len(self.global_plan.points)

	def vehicle_state_cb(self,msg):
		self.speed = msg.drive.speed
		self.steering_angle=msg.drive.steering_angle
		
	def pose_cb(self,msg):
		# self.old_pose=self.current_pose
		self.stamp=msg.header.stamp
		self.current_pose = msg

		if self.old_poses_list==None:
			# print('self.old_poses_list none')
			self.old_poses_list = [[self.current_pose] for i in range(200)]
		else:

			deque_old_poses_list = deque(self.old_poses_list) 
			deque_old_poses_list.rotate(1) 
			self.old_poses_list = list(deque_old_poses_list)
			self.old_poses_list[0] = [self.current_pose]

	def lidar_bev_imageCallback(self,im):
		try:
			self.lidar_bev_image = self.cvbridge.imgmsg_to_cv2(im, "mono8")
		except CvBridgeError as e:
			print (e)

	def improve_waypoints(self, msg):
		self.waypoints = np.array([])
		self.index=0

		# print ("[CNN Local Path Planning] route inference: new route! ")

		way = [[p.pose.position.x, p.pose.position.y, p.pose.position.z] for p in msg.poses]
		# way=way[0::5]
		# print("len(way) ",len(way))
		way = np.asarray(way)

		new_way = []
		_p = way[0,0:2]
		_i = 0
		wx =[]
		wy = []
		for i in np.arange(0, len(way)):
			d = np.sqrt(np.power(_p - way[_i:min(_i+50, len(way)), 0:2], 2).sum(axis=1))
			closest = d.argmin() 
			
			if d[closest]<1.1:
				way = np.delete(way, closest, axis=0)
				continue

			new_way.append(way[closest])
			_i = closest 
			_p = way[closest, 0:2]
			way = np.delete(way, closest, axis=0)

		way = np.asarray(new_way)			

		dist = np.sqrt(np.power(way[0,:] - way[1,:], 2).sum())

		s, _ = initial_guess(way[:, 0:2])
		
		interp_x = interpolate.interp1d(s, way[:, 0], fill_value='extrapolate')
		interp_y = interpolate.interp1d(s, way[:, 1], fill_value='extrapolate')
		
		x_new = interp_x(np.arange(0, s[-1],self.precision))

		y_new = interp_y(np.arange(0, s[-1],self.precision))

		stamp = rospy.Time().now()
		
		while(self.index < len(x_new)):
			
			max_index = self.index

			if self.index + self.window > len(x_new):
				max_index = len(x_new)
			else:
				max_index = self.index + self.window

			ways = generate_trajectory(np.array([x_new[self.index:max_index], y_new[self.index:max_index]]).T)
			self.index=max_index

			if len(self.waypoints) == 0:
				self.waypoints =  ways
			else:
				ways[:, 4] = ways[:,4] + self.waypoints[-1, 4]
				self.waypoints  = np.concatenate((self.waypoints , ways), axis=0)
			
			# print ("Progress: " + str(float(self.index)/float(len(x_new))))

		# print ("Done!")
		return self.waypoints

	def stereo_bev_imageCallback(self,im):

		try:
			self.stereo_bev_image = self.cvbridge.imgmsg_to_cv2(im, "bgr8")
		except CvBridgeError as e:
			print (e)

		# print(self.current_pose is None , self.stereo_bev_image is None , self.lidar_bev_image is None , self.global_plan is None ,
		# 	self.old_poses_list is None)
		if self.current_pose is None or self.stereo_bev_image is None or self.lidar_bev_image is None or self.global_plan is None or self.old_poses_list is None:
			return
		scale=8
		size_image=700


		current_pose=self.current_pose

		old_poses_list=self.old_poses_list
		global_plan=self.global_plan

		lidar_bev_image=self.lidar_bev_image
		stereo_bev_image=self.stereo_bev_image 

		# stamp = msg.header.stamp#rospy.Time().now()
		stamp = self.stamp#im.header.stamp#rospy.Time().now()

		################inverse transform map to velodyne
		try:
			trans_rot= self.tf2_buffer_blink2odom.lookup_transform('map', 'velodyne', rospy.Time())
			# stamp = msg.header.stamp#rospy.Time().now()
			# because it is not the tf::Transform, there is no reverse member function call, do it manually
			trans=(trans_rot.transform.translation.x, trans_rot.transform.translation.y,  trans_rot.transform.translation.z)
			rot  =(trans_rot.transform.rotation.x,    trans_rot.transform.rotation.y,     trans_rot.transform.rotation.z,   trans_rot.transform.rotation.w)
			transform = concatenate_matrices(translation_matrix(trans), quaternion_matrix(rot))
			transform_to_world=trans_rot#transform
			inversed_trans = inverse_matrix(transform)

		except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as exept:
			print(exept)
			return

		header = Header()
		# header.stamp = msg.header.stamp#rospy.Time.now()
		header.stamp = stamp#im.header.stamp#rospy.Time.now()

		header.frame_id = 'velodyne'
		tr_inv=translation_from_matrix(inversed_trans)
		rot_inv=quaternion_from_matrix(inversed_trans)
		trans = Transform(translation=Vector3(tr_inv[0], tr_inv[1], tr_inv[2]),rotation=Quaternion(rot_inv[0], rot_inv[1], rot_inv[2], rot_inv[3])  )#Quaternion(rot_inv[0], rot_inv[1], rot_inv[2], rot_inv[3])  )

		trans_stamp_inv = TransformStamped(header, 'map_vel', trans)

		
		if self.old_pose==None: #or global_plan==None:
			self.old_pose=current_pose
			dist=10000
		else:
			dist=numpy.linalg.norm(np.array([self.old_pose.pose.pose.position.x, self.old_pose.pose.pose.position.y]) - np.array([current_pose.pose.pose.position.x, current_pose.pose.pose.position.y]))

		global_plan_points_png= numpy.zeros([size_image,size_image,3],dtype=np.uint8)
		global_plan_line_png= numpy.zeros([size_image,size_image,1],dtype=np.uint8)
		gps_backward_png= numpy.zeros([size_image,size_image,1],dtype=np.uint8)
		path_pred_png= numpy.zeros([size_image,size_image,1],dtype=np.uint8)


		if self.first_frame:
			self.first_frame=False
			dist=10000 
			# print('first frame')
			# return
		# if dist>0.2 or self.first_frame:
		# else:
		# print('gen path')

		# if dist>0.05 :
		# print('dist ',dist)
		# print(stamp.secs,self.last_path_time.secs)
		# print((stamp-self.last_path_time).to_sec())

		self.msg_path_p.poses = []

		self.path_msg.path = []

		self.markerArray.markers=[]

		self.intersections_markerArray.markers=[]

		poses_back=[]
		poses_pred_gt=[]
		poses_pred_gt_to_txt=[]
		
		for p in old_poses_list:
			new_tfmed_gps = tf2_geometry_msgs.do_transform_pose(p[0].pose, trans_stamp_inv)
			poses_back.append( [(new_tfmed_gps.pose.position.y*scale)+(size_image/2), (new_tfmed_gps.pose.position.x*scale)+(size_image/3)] )

		im_color_lidar = cv2.applyColorMap(lidar_bev_image, cv2.COLORMAP_JET)
		im_color_lidar = cv2.bitwise_and(im_color_lidar, im_color_lidar, mask=lidar_bev_image)

		raw_path_points=[]
		color_plan=[]

		next_intersection=None

		for index, (pose, option) in enumerate(zip(global_plan.points, global_plan.road_options)):

			old_pose = PoseStamped()
			old_pose.header.frame_id='map'
			old_pose.header.stamp = stamp
			old_pose.pose.position = pose

			new_pose = tf2_geometry_msgs.do_transform_pose(old_pose, trans_stamp_inv)
			raw_path_points.append([new_pose.pose.position.y*scale+(size_image/2), new_pose.pose.position.x*scale+(size_image/3)])

			marker = Marker()
			marker.header.frame_id = "velodyne"
			marker.type = marker.SPHERE
			marker.action = marker.ADD
			marker.ns = "my_namespace";

			# marker scale
			marker.scale.x = 5.3
			marker.scale.y = 5.3
			marker.scale.z = 5.3

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
			marker.pose.position.x = new_pose.pose.position.x
			marker.pose.position.y = new_pose.pose.position.y
			marker.pose.position.z = -20

			t = rospy.Duration(5.0) 
			marker.lifetime = t
			self.markerArray.markers.append(marker)

			if self.STRAIGHT==option or self.RIGHT==option or self.LEFT==option :
				self.intersections_markerArray.markers.append(marker)

			thickness=-1

			if index==self.next_index_goal_plan:          
				dist_to_goal =numpy.linalg.norm(np.array([pose.x, pose.y]) - 
														np.array([current_pose.pose.pose.position.x, current_pose.pose.pose.position.y]))#dist between ego and the next goal
				if dist_to_goal<10:  ## if the ego achieve the goal
					self.next_index_goal_plan=self.next_index_goal_plan+1  #next goal

			if self.next_index_goal_plan>=2:
				signal_first_point_achived = Bool()
				signal_first_point_achived.data = True
				self.first_point_achived.publish(signal_first_point_achived)

		init_interval=self.next_index_goal_plan-2
		end_interval=self.next_index_goal_plan+3

		if init_interval<0:
			init_interval=0
		if end_interval>=len(raw_path_points):
			end_interval=len(raw_path_points)-1

		raw_local_points=[]
		for i in range(init_interval, end_interval):
			raw_local_points.append(raw_path_points[i])
			color_point=color_plan[i]
			point_local=raw_path_points[i]

			cv2.circle(global_plan_points_png, (int(point_local[0]), int(point_local[1])), 10, color_point, thickness)
		#global_plan_points_png = cv2.cvtColor(global_plan_points_png, cv2.COLOR_BGR2RGB)

		thickness=4
		pts = np.array(raw_local_points, np.int32)
		pts = pts.reshape((-1,1,2))
		cv2.polylines(global_plan_line_png,[pts],False,(255),thickness)	

		pts = np.array(poses_back, np.int32)
		pts = pts.reshape((-1,1,2))
		cv2.polylines(gps_backward_png,[pts],False,(255),thickness)

		pts = np.array(poses_pred_gt, np.int32)
		pts = pts.reshape((-1,1,2))	
		cv2.polylines(path_pred_png,[pts],False,(255))	

		# cv2.imshow('lidar_bev_image', im_color_lidar)
		# cv2.waitKey(1)
		# cv2.imshow('stereo_bev_image', stereo_bev_image)
		# cv2.waitKey(1)
		# cv2.imshow('global_plan_line_png', global_plan_line_png)
		# cv2.waitKey(1)
		# cv2.imshow('global_plan_points_png', global_plan_points_png)
		# cv2.waitKey(1)
		# cv2.imshow('global_plan_line_png', global_plan_line_png)
		# cv2.waitKey(1)
		# cv2.imshow('gps_backward_png', gps_backward_png)
		# cv2.waitKey(1)
		# cv2.imshow('path_pred_png', path_pred_png)
		# cv2.waitKey(1)



		id = 0
		for m in self.markerArray.markers:
		   m.id = id
		   id += 1

		id = 0
		for m in self.intersections_markerArray.markers:
		   m.id = id
		   id += 1


		# print('outputin')
		# print(global_plan_line_png.shape)

		global_plan_line_png=np.stack((global_plan_line_png[:,:,0],)*3, axis=-1)
		# print(global_plan_line_png.shape)
		gps_backward_png=np.stack((gps_backward_png[:,:,0],)*3, axis=-1)

		seg, output = inference_segmentor(self.model, stereo_bev_image, im_color_lidar, 
														global_plan_points_png, global_plan_line_png,
														gps_backward_png)#args.img)
		# print('seg',seg[0].shape[0],seg[0].shape[1])
		# print(self.model.CLASSES)
		# print(self.model.PALETTE)
		# print(self.palette)
		img_seg=self.model.show_result(stereo_bev_image,seg, self.palette, opacity=1.0)

		try:
			self.seg_image_pub.publish(self.cvbridge.cv2_to_imgmsg(img_seg, "bgr8"))
		except CvBridgeError as e:
			print(e)
		# cv2.imwrite('/home/luis/Desktop/segfpn.png',img_seg)
		# print('seg',seg)
		# print('seg shape',seg[0].shape)
		# seg=seg[0]
		# seg_car=np.where(seg==7 ,255,0)
		# seg_pedestrian=np.where(seg==4 ,255,0)



		########################################################################################################
		####################################################ocuppancy map ###############################################
		########################################################################################################
		self.map_msg = map_msg = OccupancyGrid()
		map_msg.header.frame_id='map'
		# extract red channel, transform, scale to range 0..100, convert to int8
		map_img = (seg[0]).astype(np.int8)

		map_img = np.rot90(map_img)
		map_img = np.flipud(map_img)

		map_img[map_img==4]=100
		# map_img[map_img==6]=100
		# map_img[map_img==5]=100

		map_img[map_img<100]=0
		# map_img = (img_aerialseg[..., 0] * 100.0 / 7).astype(np.int8)
		map_msg.data = map_img.ravel().tolist()

		# set up general info
		map_msg.info.resolution = 0.125#self.carla_map._pixel_density
		map_msg.info.width = map_img.shape[1]
		map_msg.info.height = map_img.shape[0]

		# set up origin orientation
		map_msg.info.origin.orientation.x = self.current_pose.pose.pose.orientation.x#quat[0]
		map_msg.info.origin.orientation.y = self.current_pose.pose.pose.orientation.y#quat[1]
		map_msg.info.origin.orientation.z = self.current_pose.pose.pose.orientation.z#quat[2]
		map_msg.info.origin.orientation.w = self.current_pose.pose.pose.orientation.w#quat[3]

		# set up origin position
		top_right_corner = float(map_img.shape[1]/2), 0.0
		# to_world = self.carla_map.convert_to_world(top_right_corner)
		xp=-float(map_img.shape[0]/3)*map_msg.info.resolution  #* np.cos(self.yaw_gt)
		yp=-float(map_img.shape[1]/2)*map_msg.info.resolution  #* np.sin(self.yaw_gt)

		r, p, yaw = tf.transformations.euler_from_quaternion([self.current_pose.pose.pose.orientation.x, 
			self.current_pose.pose.pose.orientation.y, self.current_pose.pose.pose.orientation.z, 
			self.current_pose.pose.pose.orientation.w]) 

		map_msg.info.origin.position.x = self.current_pose.pose.pose.position.x + xp*np.cos(yaw) - yp*np.sin(yaw)  #to_world[0]
		map_msg.info.origin.position.y = self.current_pose.pose.pose.position.y + xp*np.sin(yaw) + yp*np.cos(yaw)  #-to_world[1]

		# FIXME: remove hardcoded values from convert_to_world
		map_msg.info.origin.position.z = self.current_pose.pose.pose.position.z#-self.carla_map._converter._worldoffset[2]

		self.occupation_map_pub.publish(self.map_msg)

		# cv2.imwrite('/home/luis/Desktop/obstcar.png',seg_car)
		# cv2.imwrite('/home/luis/Desktop/obstpedestrian.png',seg_pedestrian)

		# cv2.waitKey(10)

		# print('seg',seg)
		# print('seg shape',seg[0].shape)

		msg_path = RosPath()
		msg_path.header.frame_id='map'
		msg_path.poses = []




		for p in output[0]:
			# print(p)
			# print(p[0])
			# print(p[1])


			ps = PoseStamped()
			ps.header.stamp = rospy.Time().now()#stamp#
			ps.pose.position.x = p[0] #- self.init_point_odom[0]
			ps.pose.position.y = p[1] #- self.init_point_odom[1]
			to_word_point_path = tf2_geometry_msgs.do_transform_pose(ps, transform_to_world)
			msg_path.poses.append(to_word_point_path)

		msg_path.header.stamp =  rospy.Time().now()#stamp#

		# print('len path ',len(msg_path.poses))

		try:
			path_imp=self.improve_waypoints(msg_path)
		except Exception as e:
			self.first_frame=True
			print(e)
			return

		count = 0

		# print(path_imp)
		for w in path_imp:
			t = TrajectoryPoint()
			# print(w)
			t.point = w
			t.point_number = count
			t.end_track = False
			count = count + 1
			self.path_msg.path.append(t)

			ps = PoseStamped()
			ps.header.stamp = stamp#rospy.Time().now()#stamp#
			ps.pose.position.x = w[0] #- self.init_point_odom[0]
			ps.pose.position.y = w[1] #- self.init_point_odom[1]
			# to_word_point_path = tf2_geometry_msgs.do_transform_pose(ps, transform_to_world)
			self.msg_path_p.poses.append(ps)#to_word_point_path)



		self.path_msg.path[-1].end_track = False#True

		if dist>1.0 or (stamp - self.last_path_time).to_sec() > 10:
			self.old_pose=current_pose

			self.last_path_time=stamp

			self.marker_route_pub.publish(self.markerArray)

			self.msg_path_p.header.stamp =  stamp#rospy.Time().now()
			self.path_msg.header.stamp = rospy.Time().now()#stamp#
			self.path_pub.publish(self.path_msg)
			self.path_ros_pub.publish(self.msg_path_p)

		self.pub_intersections_obj.publish(self.intersections_markerArray)


	def shutdown_cb(self, msg):
		if msg.data:
			self.path = np.array([])
			self.last_path_stamp = 0.0

			print ("Bye!")
			rospy.signal_shutdown("finished route")

	def shutdown_using_map_cb(self, msg):
		if msg.data:
			self.path = np.array([])
			self.last_path_stamp = 0.0

			print ("Bye!")
			rospy.signal_shutdown("shutdown cnn planner because new opendrive map")



if __name__ == '__main__':
	rospy.init_node("fpn_panner", anonymous=True)
	print ("[Create fpn planner running...")
	dataset=CreateDatasetCarla()
	rospy.spin()



