#!/usr/bin/env python3
import numpy as np
import rospy
import time
from geometry_msgs.msg import PoseWithCovarianceStamped, Twist, PoseStamped, TransformStamped
from msgs_action.msg import VehicleState, Throttle, Brake, SteeringAngle

# from msgs_perception.msg import Obstacle, ObstacleArray
from std_msgs.msg import Float64, Bool
# from msgs_navigation.msg import Path
from nav_msgs.msg import Path as RosPath

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
from std_msgs.msg import Header
from geometry_msgs.msg import Transform, Vector3, Quaternion

import cv2

from collections import deque 

from cv_bridge import CvBridge, CvBridgeError
# from sensor_msgs.msg import Image
from msgs_perception.msg import ObstacleArray
from msgs_perception.msg import Obstacle

from msgs_navigation.msg import GlobalPlan

from visualization_msgs.msg import Marker
from visualization_msgs.msg import MarkerArray

from pathlib import Path
import os

from image_geometry import PinholeCameraModel

import time
import pickle

class CreateDatasetCarla(object):
	def __init__ (self):
		self.cvbridge = CvBridge()
		self.points_lidar=None
		

		self.n_folder=0
		self.string_folder='{:03d}'.format(self.n_folder)

		self.old_pose=None 
		self.current_pose=None

		self.tf2_buffer_blink2odom = tf2_ros.Buffer()
		self.tf2_listener_blink2odom = tf2_ros.TransformListener(self.tf2_buffer_blink2odom)
		self.frames=[]

		#subscriber
	

		self.lidar_sub = rospy.Subscriber("/carina/sensor/lidar/front/point_cloud", PointCloud2, self.lidar_point_cloud_cb)

		self.image_rgb_sub = rospy.Subscriber('/carina/sensor/camera/left/image_raw', Image, self.rgb_imageCallback, queue_size=1)
		self.img_left_info_sub = rospy.Subscriber('/carina/sensor/camera/left/camera_info', CameraInfo, self.rgb_camera_infoCallback, queue_size=1)

		self.shutdown_sub = rospy.Subscriber('/carina/vehicle/shutdown', Bool, self.shutdown_cb, queue_size=1)



		self.pose_sub = rospy.Subscriber('/carina/localization/pose', PoseWithCovarianceStamped, self.pose_gps_cb, queue_size=1)

		self.gt_obstacles_sub = rospy.Subscriber('/carina/perception/dataset/obstacles_array', ObstacleArray, self.obstacles_dataset_gt_cb, queue_size=1)




	def rgb_imageCallback(self,im):
		try:
			self.rgb_image = self.cvbridge.imgmsg_to_cv2(im, "bgr8")
		except CvBridgeError as e:
			print (e)

	def rgb_camera_infoCallback(self,msg):
		self.rgb_cam_info=msg



	def pose_gps_cb(self,msg):
		# self.old_pose=self.current_pose
		self.current_pose_gps = msg



	def obstacles_dataset_gt_cb(self,msg):
		self.obstacles_dataset_gt=msg.obstacle
		# print('len ',len(self.global_plan.points))


	def lidar_point_cloud_cb(self,data):
		# print('lidar')
		# self.points_lidar_msg=data
		timestamp=rospy.Time.now()

		if self.old_pose==None or self.current_pose==None: #or global_plan==None:
			dist=10000
		else:
			dist=numpy.linalg.norm(np.array([self.old_pose.pose.pose.position.x, self.old_pose.pose.pose.position.y]) - np.array([current_pose.pose.pose.position.x, current_pose.pose.pose.position.y]))

		pc = ros_numpy.numpify(data)
		# self.points_lidar=np.zeros((pc.shape[0],44))
		# points=np.zeros((pc.shape[0],3))

		x=pc['x']
		y=pc['y']
		z=pc['z']
		intensity=pc['intensity']


		arr = np.zeros(x.shape[0] + y.shape[0] + z.shape[0] + intensity.shape[0], dtype=np.float32)
		arr[::4] = x
		arr[1::4] = y
		arr[2::4] = z
		arr[3::4] = intensity
		arr.astype('float32').tofile('/home/luis/Desktop/filename.bin')

		classes_list=[]


		for obs in self.obstacles_dataset_gt:
			for c in obs.classes:
				classes_list.append(c)

		# if 'CLASSIFICATION_BIKE' in classes_list:
		# 	print('CLASSIFICATION_BIKE')

		# if 'CLASSIFICATION_MOTORCYCLE' in classes_list:
		# 	print('CLASSIFICATION_MOTORCYCLE')

		# if 'CLASSIFICATION_TRUCK' in classes_list:
		# 	print('CLASSIFICATION_TRUCK')

		# if 'CLASSIFICATION_OTHER_VEHICLE' in classes_list:
		# 	print('CLASSIFICATION_OTHER_VEHICLE')

		# if 'CLASSIFICATION_PEDESTRIAN' in classes_list:
		# 	print('CLASSIFICATION_PEDESTRIAN')		


		# dic_frame={'image': {'image_idx': 0, 'image_path': 'training/image_2/000000.png', 'image_shape': np.array([ 370, 1224], dtype=np.int32)}, \
		# 	'point_cloud': {'num_features': 4, 'velodyne_path': 'training/velodyne/000000.bin'}, \
		# 	'calib': {'P0': np.array([[707.0493,   0.    , 604.0814,   0.    ], \
		# 	[  0.    , 707.0493, 180.5066,   0.    ], \
		# 	[  0.    ,   0.    ,   1.    ,   0.    ], \
		# 	[  0.    ,   0.    ,   0.    ,   1.    ]]), \
		# 	'P1': np.array([[ 707.0493,    0.    ,  604.0814, -379.7842], \
		# 	[   0.    ,  707.0493,  180.5066,    0.    ], \
		# 	[   0.    ,    0.    ,    1.    ,    0.    ], \
		# 	[   0.    ,    0.    ,    0.    ,    1.    ]]), \
		# 	'P2': np.array([[ 7.070493e+02,  0.000000e+00,  6.040814e+02,  4.575831e+01], \
		# 	[ 0.000000e+00,  7.070493e+02,  1.805066e+02, -3.454157e-01], \
		# 	[ 0.000000e+00,  0.000000e+00,  1.000000e+00,  4.981016e-03], \
		# 	[ 0.000000e+00,  0.000000e+00,  0.000000e+00,  1.000000e+00]]), \
		# 	'P3': np.array([[ 7.070493e+02,  0.000000e+00,  6.040814e+02, -3.341081e+02], \
		# 	[ 0.000000e+00,  7.070493e+02,  1.805066e+02,  2.330660e+00], \
		# 	[ 0.000000e+00,  0.000000e+00,  1.000000e+00,  3.201153e-03], \
		# 	[ 0.000000e+00,  0.000000e+00,  0.000000e+00,  1.000000e+00]]), \
		# 	'R0_rect': np.array([[ 0.9999128 ,  0.01009263, -0.00851193,  0.        ], \
		# 	[-0.01012729,  0.9999406 , -0.00403767,  0.        ], \
		# 	[ 0.00847068,  0.00412352,  0.9999556 ,  0.        ], \
		# 	[ 0.        ,  0.        ,  0.        ,  1.        ]]), \
		# 	'Tr_velo_to_cam': np.array([[ 0.00692796, -0.9999722 , -0.00275783, -0.02457729], \
		# 	[-0.00116298,  0.00274984, -0.9999955 , -0.06127237], \
		# 	[ 0.9999753 ,  0.00693114, -0.0011439 , -0.3321029 ], \
		# 	[ 0.        ,  0.        ,  0.        ,  1.        ]]), \
		# 	'Tr_imu_to_velo': np.array([[ 9.999976e-01,  7.553071e-04, -2.035826e-03, -8.086759e-01], \
		# 	[-7.854027e-04,  9.998898e-01, -1.482298e-02,  3.195559e-01], \
		# 	[ 2.024406e-03,  1.482454e-02,  9.998881e-01, -7.997231e-01], \
		# 	[ 0.000000e+00,  0.000000e+00,  0.000000e+00,  1.000000e+00]])}, \
		# 	'annos': {'name': np.array(['Pedestrian'], dtype='<U10'), 'truncated': np.array([0.]), 'occluded': np.array([0]), \
		# 	'alpha': np.array([-0.2]), 'bbox': np.array([[712.4 , 143.  , 810.73, 307.92]]), 'dimensions': np.array([[1.2 , 1.89, 0.48]]), \
		# 	'location': np.array([[1.84, 1.47, 8.41]]), 'rotation_y': np.array([0.01]), 'score': np.array([0.]), \
		# 	'index': np.array([0], dtype=np.int32), 'group_ids': np.array([0], dtype=np.int32), 'difficulty': np.array([0], dtype=np.int32), \
		# 	'num_points_in_gt': np.array([377], dtype=np.int32)}}
			

		# print(self.frames)

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

		filename=int(time.time_ns())
		# print(self.string_folder)
		# Path(os.path.join(root_dataset,train_test,inputs,rgb_img,self.string_folder)).mkdir(parents=True, exist_ok=True)
		# cv2.imwrite(os.path.join(root_dataset,train_test,inputs,rgb_img,self.string_folder,filename+'.jpg'), rgb_image)
		# print(classes_list)
		if ('CLASSIFICATION_BIKE' in classes_list) or ('CLASSIFICATION_MOTORCYCLE' in classes_list) or ('CLASSIFICATION_TRUCK' in classes_list) or \
								('CLASSIFICATION_OTHER_VEHICLE' in classes_list) or ('CLASSIFICATION_PEDESTRIAN' in classes_list):
			# print(True)
			# print(classes_list)

			for obs in self.obstacles_dataset_gt:

				x = obs.pose.position.x
				y = obs.pose.position.y
				z = obs.pose.position.z

				angle_ob = euler_from_quaternion([obs.pose.orientation.x,	obs.pose.orientation.y, obs.pose.orientation.z, obs.pose.orientation.w])
				# print(angle_ob[2])
				# print(obs.classes)
				# for c in obs.classes:
				# 	classes_list.append(c)

				w=obs.scale.x
				l=obs.scale.y
				h=obs.scale.z

				if obs.classes[0]=='CLASSIFICATION_BIKE' or obs.classes[0]=='CLASSIFICATION_MOTORCYCLE':
					c='cyclist'

				if obs.classes[0]=='CLASSIFICATION_PEDESTRIAN':
					c='pedestrian'

				if obs.classes[0]=='CLASSIFICATION_TRUCK' or obs.classes[0]=='CLASSIFICATION_OTHER_VEHICLE'or obs.classes[0]=='CLASSIFICATION_CAR':
					c='car'
				gt_names.append(c)
				# name.append(obs.classes[0])
				# truncated.append(0.)
				# occluded.append(0)
				# alpha.append(-0.2)
				# bbox.append(  [0. , 0.  , 0., 0.])
				gt_bboxes_3d.append([x,y,z,w,l,h,angle_ob[2]])#(x, y, z, x_size, y_size, z_size, yaw, ...)
				# dimensions.append([w , l, h])
				# location.append([x, y, z])
				# rotation_y.append(angle_ob[2])
				# score.append(1.)
				index.append(obs.id)
				# sample_idx
				# group_ids.append(0)
				# difficulty.append(0)
				# num_points_in_gt.append(337)

			    # Attributes:
			    #     tensor (torch.Tensor): Float matrix of N x box_dim.
			    #     box_dim (int): Integer indicating the dimension of a box.
			    #         Each row is (x, y, z, x_size, y_size, z_size, yaw, ...).
			    #     with_yaw (bool): If True, the value of yaw will be set to 0 as minmax
			    #         boxes.
    
			dic_frame={
				'sample_idx': filename, \
				'lidar_points': {'lidar_path': 'training/velodyne/'+str(filename)+'.bin', 'num_pts_feats': 4 }, \
				'annos': {'box_type_3d':  'LiDAR', 'gt_bboxes_3d':  np.array(gt_bboxes_3d), 'gt_names':  np.array(gt_names, \
																 dtype='<U10'), 'gt_index': np.array(index, dtype=np.int32)}, \
				'calib': {}, \
				'images': {}}

			# filename=int(time.time_ns())
			root_dataset='/mnt/Data1/dataset_carla_tracking'
			train_test='training'
			velodyne='velodyne'
			labels='labels'

			path_labels  =os.path.join(root_dataset,train_test,labels)
			path_velodyne=os.path.join(root_dataset,train_test,velodyne)

			Path(path_labels).mkdir(parents=True, exist_ok=True)
			Path(path_velodyne).mkdir(parents=True, exist_ok=True)

			with open(os.path.join(path_labels, str(filename)+'.pkl'),  'wb') as handle:
				pickle.dump(dic_frame, handle, protocol=pickle.HIGHEST_PROTOCOL)

			self.frames.append(dic_frame)

			# print(self.frames)

			
			arr.astype('float32').tofile(os.path.join(path_velodyne,str(filename)+'.bin'))



	def shutdown_cb(self, msg):
		if msg.data:
			self.path = np.array([])
			self.last_path_stamp = 0.0

			print ("Bye!")
			# rospy.signal_shutdown("finished route")
			self.n_folder=self.n_folder+1
			self.string_folder='{:03d}'.format(self.n_folder)



if __name__ == '__main__':
	rospy.init_node("tracking_pointclud", anonymous=True)
	print ("[tracking_pointclud] running...")
	dataset=CreateDatasetCarla()
	rospy.spin()



