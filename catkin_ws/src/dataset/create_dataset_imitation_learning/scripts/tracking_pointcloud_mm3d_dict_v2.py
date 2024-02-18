#!/usr/bin/env python3
import numpy as np
import rospy
import time
from geometry_msgs.msg import PoseWithCovarianceStamped, Twist, PoseStamped, TransformStamped
from msgs_action.msg import VehicleState, Throttle, Brake, SteeringAngle

from std_msgs.msg import Float64, Bool
from nav_msgs.msg import Path as RosPath

from sensor_msgs.msg import Image, PointCloud2, CameraInfo

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
from msgs_perception.msg import ObstacleArray
from msgs_perception.msg import Obstacle

from msgs_navigation.msg import GlobalPlan

from visualization_msgs.msg import Marker
from visualization_msgs.msg import MarkerArray

from msgs_perception.msg import BoundingBox, BoundingBoxArray

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
		self.current_pose_gps=None

		self.rgb_image=''
		self.seg_image=''
		self.ins_image=''
		self.classes=[]

		self.tf2_buffer_blink2odom = tf2_ros.Buffer()
		self.tf2_listener_blink2odom = tf2_ros.TransformListener(self.tf2_buffer_blink2odom)
		self.frames=[]
		self.obstacles_dataset_gt=[]

		#subscriber
		self.stereo_sub = rospy.Subscriber("/carina/perception/stereo/point_cloud", PointCloud2, self.stereo_point_cloud_cb)
		self.lidar_sub = rospy.Subscriber("/carina/sensor/lidar/front/point_cloud", PointCloud2, self.lidar_point_cloud_cb)
		self.image_rgb_sub = rospy.Subscriber('/carina/sensor/camera/left/image_raw', Image, self.rgb_imageCallback, queue_size=1)
		self.img_left_info_sub = rospy.Subscriber('/carina/sensor/camera/left/camera_info', CameraInfo, self.rgb_camera_infoCallback, queue_size=1)

		# self.image_seg_sub = rospy.Subscriber('/carina/sensor/camera/left/sem/image_raw', Image, self.seg_imageCallback, queue_size=1)
		self.image_seg_sub = rospy.Subscriber('/carla/hero/CameraSemantic/image', Image, self.seg_imageCallback, queue_size=1)
		self.image_instance_sub = rospy.Subscriber('/carla/hero/CameraInstances/image', Image, self.instance_imageCallback, queue_size=1)

		self.shutdown_sub = rospy.Subscriber('/carina/vehicle/shutdown', Bool, self.shutdown_cb, queue_size=1)
		self.pose_sub = rospy.Subscriber('/carina/localization/pose', PoseWithCovarianceStamped, self.pose_gps_cb, queue_size=1)
		self.gt_obstacles_sub = rospy.Subscriber('/carina/perception/dataset/obstacles_array', ObstacleArray, self.obstacles_dataset_gt_cb, queue_size=1)
		self.traffic_signs_sub = rospy.Subscriber("/carina/perception/camera/signs_bb", BoundingBoxArray, self.traffic_signs_cb, queue_size=1)

	def traffic_signs_cb(self,data):
		classes=[]
		for obj in data.objects:
			print(obj.classe)
			classes.append(obj.classe.data)
		self.classes=classes
		print(self.classes)

	def rgb_imageCallback(self,im):
		try:
			self.rgb_image = self.cvbridge.imgmsg_to_cv2(im, "bgr8")
		except CvBridgeError as e:
			print (e)

	def rgb_camera_infoCallback(self,msg):
		self.rgb_cam_info=msg



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


	def pose_gps_cb(self,msg):
		self.current_pose_gps = msg

	def obstacles_dataset_gt_cb(self,msg):
		self.obstacles_dataset_gt=msg.obstacle

	def stereo_point_cloud_cb(self,data):

		rgb_image=self.rgb_image.copy()
		seg_image=self.seg_image.copy()
		ins_image=self.ins_image.copy()


		timestamp=rospy.Time.now()

		if self.old_pose==None or self.current_pose_gps==None: #or global_plan==None:
			dist=10000
			self.old_pose=self.current_pose_gps
		else:
			dist=numpy.linalg.norm(np.array([self.old_pose.pose.pose.position.x, self.old_pose.pose.pose.position.y]) - np.array([self.current_pose_gps.pose.pose.position.x, self.current_pose_gps.pose.pose.position.y]))

		# if dist < 2.0:
		# 	return
		
		self.old_pose=self.current_pose_gps
		pc = ros_numpy.numpify(data)
		# print(pc.dtype)
		pc = ros_numpy.point_cloud2.split_rgb_field(pc)
		# print(pc.dtype)

		# self.points_lidar=np.zeros((pc.shape[0],44))
		# points=np.zeros((pc.shape[0],3))
		# print(pc['x'].shape)
		x=pc['z']+0.55
		x=np.reshape(x, (x.shape[0]*x.shape[1]))

		y=-pc['x']
		y=np.reshape(y, (y.shape[0]*y.shape[1]))

		z=-pc['y']
		z=np.reshape(z, (z.shape[0]*z.shape[1]))

		r=pc['r']
		r=np.reshape(r, (r.shape[0]*r.shape[1]))

		g=pc['g']
		g=np.reshape(g, (g.shape[0]*g.shape[1]))

		b=pc['b']
		b=np.reshape(b, (b.shape[0]*b.shape[1]))

		arr_stereo = np.zeros(x.shape[0] + y.shape[0] + z.shape[0] + r.shape[0]  + g.shape[0] + b.shape[0], dtype=np.float32)
		arr_stereo[::6] = x
		arr_stereo[1::6] = y
		arr_stereo[2::6] = z
		arr_stereo[3::6] = r
		arr_stereo[4::6] = g
		arr_stereo[5::6] = b


		classes_list=[]

		for obs in self.obstacles_dataset_gt:
			for c in obs.classes:
				classes_list.append(c)


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

		METAINFO = { 'classes':   ('Car', 'Cyclist', 'Pedestrian'),    }#,'Van','Motorcycle','Truck'),    }

		# if ('CLASSIFICATION_VAN' in classes_list) or ('CLASSIFICATION_BIKE' in classes_list) or ('CLASSIFICATION_MOTORCYCLE' in classes_list) or  \
		# 	('CLASSIFICATION_TRUCK' in classes_list) or ('CLASSIFICATION_OTHER_VEHICLE' in classes_list) or ('CLASSIFICATION_PEDESTRIAN' in classes_list):
		# if ('CLASSIFICATION_PEDESTRIAN' in classes_list) or 'traffic_light_red' in self.classes or 'traffic_light_green' in self.classes or 'traffic_light_yellow' in self.classes or 'stop' in self.classes:
		if ('CLASSIFICATION_PEDESTRIAN' in classes_list):

			instance_list_lidar = []
			instance_list_stereo = []

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
				# gt_names.append(c)
				empty_instance['bbox_label'] = METAINFO['classes'].index(c)
				empty_instance['bbox_label_3d'] = copy.deepcopy( empty_instance['bbox_label'])
				# name.append(obs.classes[0])
				# truncated.append(0.)
				# occluded.append(0)
				# alpha.append(-0.2)
				# bbox.append(  [0. , 0.  , 0., 0.])
				# gt_bboxes_3d.append([x,y,z,w,l,h,angle_ob[2]])#(x, y, z, x_size, y_size, z_size, yaw, ...)
				empty_instance['bbox'] = [0,0,0,0]
				empty_instance['bbox_3d'] = [x,y,z,w,l,h,angle_ob[2]]
				# dimensions.append([w , l, h])
				# location.append([x, y, z])
				# rotation_y.append(angle_ob[2])
				# score.append(1.)
				# index.append(obs.id)
				empty_instance['id'] = [obs.id]
				# sample_idx
				# group_ids.append(0)
				# difficulty.append(0)
				# num_points_in_gt.append(337)


				empty_instance['truncated']=0.
				empty_instance['occluded']=0
				empty_instance['alpha']=0.
				empty_instance['score']=1.

				instance_list_lidar.append(empty_instance)

				if x>0:
					xyangle=np.arctan2(x,y)
					xyangle=np.rad2deg(xyangle)

					if (xyangle>30 and xyangle<(180-30)):
						instance_list_stereo.append(empty_instance)


			temp_data_info = self.get_empty_standard_data_info()

			temp_data_info['sample_idx']=filename
			temp_data_info['lidar_points']['num_pts_feats']=4
			temp_data_info['lidar_points']['lidar_path']= 'training/velodyne/points/'+str(filename)+'.bin'
			temp_data_info['instances'] = instance_list_lidar

			root_dataset='/mnt/Data1/dataset_carla_tracking'
			prefix='pointcloud'
			train_test='training'
			velodyne='velodyne'
			stereo='stereo'
			labels='labels'
			rgb='rgb'
			segmentation='segmentation'
			instances='instances'

			path_labels_velodyne  =os.path.join(root_dataset,train_test,prefix,velodyne,labels)
			path_labels_stereo  =os.path.join(root_dataset,train_test,prefix,stereo,labels)

			path_velodyne=os.path.join(root_dataset,train_test,prefix,velodyne,'points')
			path_stereo=os.path.join(root_dataset,train_test,prefix,stereo,'points')

			Path(path_labels_velodyne).mkdir(parents=True, exist_ok=True)
			Path(path_labels_stereo).mkdir(parents=True, exist_ok=True)

			Path(path_velodyne).mkdir(parents=True, exist_ok=True)
			Path(path_stereo).mkdir(parents=True, exist_ok=True)

			with open(os.path.join(path_labels_velodyne,str(filename)+'.pkl'),  'wb') as handle:
				pickle.dump(temp_data_info, handle, protocol=pickle.HIGHEST_PROTOCOL)

			temp_data_info['lidar_points']['lidar_path']= 'training/stereo/points/'+str(filename)+'.bin'
			temp_data_info['instances'] = instance_list_stereo
			temp_data_info['lidar_points']['num_pts_feats']=6


			# with open(os.path.join(path_labels_stereo,str(filename)+'.pkl'),  'wb') as handle:
			# 	pickle.dump(temp_data_info, handle, protocol=pickle.HIGHEST_PROTOCOL)

			self.arr.astype('float32').tofile(os.path.join(path_velodyne,str(filename)+'.bin'))
			# arr_stereo.astype('float32').tofile(os.path.join(path_stereo,str(filename)+'.bin'))

			Path(os.path.join(root_dataset,train_test,'images',rgb)).mkdir(parents=True, exist_ok=True)
			cv2.imwrite(os.path.join(root_dataset,train_test,'images',rgb, str(filename)+'.jpg'), rgb_image)

			Path(os.path.join(root_dataset,train_test,'images','gt',segmentation)).mkdir(parents=True, exist_ok=True)
			cv2.imwrite(os.path.join(root_dataset,train_test,'images','gt',segmentation, str(filename)+'.png'), seg_image)

			Path(os.path.join(root_dataset,train_test,'images','gt',instances)).mkdir(parents=True, exist_ok=True)
			cv2.imwrite(os.path.join(root_dataset,train_test,'images','gt',instances, str(filename)+'.png'), ins_image)

	def lidar_point_cloud_cb(self,data):
		timestamp=rospy.Time.now()

		if self.old_pose==None or self.current_pose_gps==None: #or global_plan==None:
			dist=10000
		else:
			dist=numpy.linalg.norm(np.array([self.old_pose.pose.pose.position.x, self.old_pose.pose.pose.position.y]) - np.array([current_pose.pose.pose.position.x, current_pose.pose.pose.position.y]))

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


	def shutdown_cb(self, msg):
		if msg.data:
			self.path = np.array([])
			self.last_path_stamp = 0.0

			print ("Bye!")
			rospy.signal_shutdown("finished route")
			self.n_folder=self.n_folder+1
			self.string_folder='{:03d}'.format(self.n_folder)



if __name__ == '__main__':
	rospy.init_node("tracking_pointclud", anonymous=True)
	print ("[tracking_pointclud] running...")
	dataset=CreateDatasetCarla()
	rospy.spin()



