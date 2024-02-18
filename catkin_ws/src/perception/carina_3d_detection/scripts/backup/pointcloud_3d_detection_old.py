#!/usr/bin/env python3
import numpy as np
import rospy
# import time
# from geometry_msgs.msg import PoseWithCovarianceStamped, Twist, PoseStamped, TransformStamped
# from msgs_action.msg import VehicleState, Throttle, Brake, SteeringAngle

# # from msgs_perception.msg import Obstacle, ObstacleArray
# from std_msgs.msg import Float64, Bool
# # from msgs_navigation.msg import Path
# from nav_msgs.msg import Path as RosPath

# # from msgs_traffic.msg import TrafficSign, TrafficSignArray

# # from abt import *
# # from model_POMDP_v5 import *
from sensor_msgs.msg import PointCloud2
import ros_numpy

# from threading import Thread

# import tf2_ros
# from tf2_geometry_msgs import *
# from tf.transformations import *
# from std_msgs.msg import Header
# from geometry_msgs.msg import Transform, Vector3, Quaternion

# import cv2

# from collections import deque 

# from cv_bridge import CvBridge, CvBridgeError
# from sensor_msgs.msg import Image

# from msgs_navigation.msg import GlobalPlan

from visualization_msgs.msg import Marker
from visualization_msgs.msg import MarkerArray


# from mmdet3d.apis import init_detector, show_result_meshlab#inference_detector, 
from inference import inference_detector, init_model
# from mmdet3d.apis import inference_detector_nusc, init_detector_nusc, show_result_meshlab_nusc
# from inference_nusc import  inference_detector_nusc, init_detector_nusc, show_result_meshlab_nusc
from mmdet3d.core import Box3DMode
# from pathlib import Path
# import os
import tf

import time
class PointCloud3Ddetection(object):

	def __init__ (self):
		self.model=None
		#subscriber
		self.lidar_sub = rospy.Subscriber("/carina/sensor/lidar/front/point_cloud", PointCloud2, self.lidar_point_cloud_cb)


		self.marker_obst_pub = rospy.Publisher('/carina/sensor/lidar/pillars_obst_array', MarkerArray, queue_size=1)

		self.points_lidar=None

		device='cuda:0'
		# checkpoint_file='/home/luis/mmdetection3d/work_dirs/second/hv_second_secfpn_6x8_80e_kitti-3d-car_20200620_230238-393f000c.pth'
		# checkpoint_file='/home/luis/catkin_ws/src/carina_3d_detection/models/centerpoint_0075voxel_second_secfpn_circlenms_4x8_cyclic_20e_nus_20200925_230905-358fbe3b.pth'
		# checkpoint_file='/home/luis/catkin_ws/src/carina_3d_detection/models/hv_pointpillars_fpn_sbn-all_4x8_2x_nus-3d_20200620_230405-2fa62f3d.pth'
		# checkpoint_file='/home/luis/catkin_ws/src/carina_3d_detection/models/hv_ssn_secfpn_sbn-all_2x16_2x_nus-3d_20201023_193737-5fda3f00.pth'
		# checkpoint_file='/home/luis/carla/TRACK/team_code/catkin_ws/src/perception/carina_3d_detection/models/hv_pointpillars_fpn_sbn-all_4x8_2x_nus-3d_20210826_104936-fca299c1.pth'
		checkpoint_file='/home/luis/carla/TRACK/team_code/catkin_ws/src/perception/carina_3d_detection/checkpoints/hv_pointpillars_secfpn_6x8_160e_kitti-3d-3class_20220301_150306-37dc2420.pth'


		config_file ='/home/luis/mmdetection3d/configs/pointpillars/hv_pointpillars_secfpn_6x8_160e_kitti-3d-3class.py'
		# config_file ='/home/luis/mmdetection3d/configs/second/hv_second_secfpn_6x8_80e_kitti-3d-car.py'
		# config_file ='/home/luis/mmdetection3d/configs/centerpoint/centerpoint_0075voxel_second_secfpn_circlenms_4x8_cyclic_20e_nus.py'
		# config_file ='/home/luis/mmdetection3d/configs/pointpillars/hv_pointpillars_fpn_sbn-all_4x8_2x_nus-3d.py'
		# config_file ='/home/luis/mmdetection3d/configs/ssn/hv_ssn_secfpn_sbn-all_2x16_2x_nus-3d.py'
		print("loading model: ",config_file)

		# build the model from a config file and a checkpoint file
		self.model = init_model(config_file, checkpoint_file, device=device)
		print("init detector ... ")

		
		self.score_thr=0.3
		self.out_dir="/home/luis/catkin_ws/src/carina_3d_detection/demo"


		
	def lidar_point_cloud_cb(self,data):
		if self.model==None:
			return
		# print('lidar')
		pc = ros_numpy.numpify(data)
		# print("cloud")
		# print(pc.dtype)
		self.points_lidar=np.zeros((pc.shape[0],4))
		# points=np.zeros((pc.shape[0],3))

		self.points_lidar[:,0]=pc['x']
		self.points_lidar[:,1]=pc['y']
		self.points_lidar[:,2]=pc['z']
		self.points_lidar[:,3]= 0.0 #pc['intensity']
		# print('max intensity: ',np.max(self.points_lidar[:,3]))
		# print('min intensity: ',np.min(self.points_lidar[:,3]))

		# self.points_lidar[:,4]= 0 #pc['ring']

		# print(self.points_lidar)


		# test a single image
		time_ini=time.time()
		result, data = inference_detector(self.model, self.points_lidar.astype(np.float32))
		time_fin=time.time()
		print(time_ini-time_fin)
		# print(result)
		# print('data ',data)



# [{'boxes_3d': LiDARInstance3DBoxes(
#     tensor([[ 15.8835,   3.7046,  -2.1156,   3.8823,   1.5996,   1.5752,   0.7677],
#         [ 51.4273,   3.9935,  -2.1078,   4.3792,   1.7143,   1.5807,   2.9961],
#         [ 11.5407, -17.3038,  -1.6115,   3.8997,   1.6417,   1.5939,   3.7797],
#         [ 28.3417,   0.5213,  -1.7855,   3.8298,   1.6445,   1.6122,   0.4652],
#         [ 12.2130, -23.8129,  -1.6055,   3.7613,   1.6518,   1.6955,   3.2269],
#         [ 41.5364,   0.1312,  -1.7323,   3.9835,   1.6676,   1.5552,   0.2923],
#         [  0.4556,  12.8126,  -1.7848,   3.9143,   1.6213,   1.6165,  -1.5243],
#         [ 59.2706,  36.4589,  -1.8547,   4.3567,   1.7589,   1.7091,  -1.5641]])), 
# 'scores_3d': tensor([0.5355, 0.4652, 0.3075, 0.2910, 0.2873, 0.2279, 0.2263, 0.2218]), 
# 'labels_3d': tensor([2, 2, 2, 2, 2, 2, 2, 2])}]


		if 'pts_bbox' in result[0].keys():
			pred_bboxes = result[0]['pts_bbox']['boxes_3d'].tensor.numpy()
			pred_scores = result[0]['pts_bbox']['scores_3d'].numpy()
		else:
			pred_bboxes = result[0]['boxes_3d'].tensor.numpy()
			pred_scores = result[0]['scores_3d'].numpy()
			labels_3d = result[0]['labels_3d'].numpy()
			# print(pred_bboxes,pred_bboxes)

	    # filter out low score bboxes for visualization
		if self.score_thr > 0:
			inds = pred_scores > self.score_thr
			pred_bboxes = pred_bboxes[inds]
			labels_3d = labels_3d[inds]



		markerArray = MarkerArray()
		markerArray.markers=[]

		for box,label in zip(pred_bboxes,labels_3d):
			# print(box,label)

			marker = Marker()
			marker.header.frame_id = "velodyne"
			marker.type = marker.CUBE
			marker.action = marker.ADD
			marker.ns = "my_namespace";

			# marker scale
			marker.scale.x = box[3]
			marker.scale.y = box[4]
			marker.scale.z = box[5]

			# marker color
			r=1.0
			g=0.0
			b=0.0
			# if self.LANEFOLLOW==option:
			# 	g=1.0
			# if self.STRAIGHT==option:
			# 	r=1.0
			# if self.RIGHT==option:
			# 	b=1.0
			# if self.LEFT==option:
			# 	r=1.0
			# 	g=1.0
			# if self.CHANGELANELEFT==option:
			# 	r=1.0
			# 	b=1.0
			# if self.CHANGELANERIGHT==option:
			# 	g=1.0
			# 	b=1.0
			# if self.UNKNOWN==option:
			# 	r=1.0
			# 	g=1.0
			# 	b=1.0
			yaw=box[6]
			ori= tf.transformations.quaternion_from_euler(0.0, 0.0, yaw)#+np.deg2rad(-90))

			marker.color.a = 0.5
			marker.color.r = r
			marker.color.g = g
			marker.color.b = b

			# marker orientaiton
			marker.pose.orientation.x = ori[0]
			marker.pose.orientation.y = ori[1]
			marker.pose.orientation.z = ori[2]
			marker.pose.orientation.w = ori[3]

			# marker position
			marker.pose.position.x = box[0]
			marker.pose.position.y = box[1]
			marker.pose.position.z = box[2]

			t = rospy.Duration(0.5) 
			marker.lifetime = t
			markerArray.markers.append(marker)

		id = 0
		for m in markerArray.markers:
		   m.id = id
		   id += 1
		self.marker_obst_pub.publish(markerArray)





if __name__ == '__main__':
	rospy.init_node("pointcloud_3d_detection", anonymous=True)
	print ("[3D detection pointcloud] running...")
	dataset=PointCloud3Ddetection()
	rospy.spin()



