#!/usr/bin/env python3
import numpy as np
import rospy

from sensor_msgs.msg import PointCloud2
import ros_numpy
from std_msgs.msg import Bool
import cv2
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
from visualization_msgs.msg import Marker
from visualization_msgs.msg import MarkerArray
from geometry_msgs.msg import Pose
from geometry_msgs.msg import PoseArray
from msgs_perception.msg import ObstacleArray, Obstacle
from inference import inference_detector, init_model
import tf
import rospkg
import os
import time
import math

class PointCloud3Ddetection(object):

	def __init__ (self):
		self.result=None 
		self.data_inf=None
		self.model=None
		self.cvbridge = CvBridge()

		rospack = rospkg.RosPack()
		dir_pkg=rospack.get_path('carina_3d_detection')
		# device='cuda:0'
		gpu_device = rospy.get_param("/gpu_device_stack_param")
		print('gpu_device stack ', gpu_device)
		# device='cuda:0'
		device=gpu_device
		# checkpoint_file='/home/luis/mmdetection3d/work_dirs/second/hv_second_secfpn_6x8_80e_kitti-3d-car_20200620_230238-393f000c.pth'
		# checkpoint_file='/home/luis/catkin_ws/src/carina_3d_detection/models/centerpoint_0075voxel_second_secfpn_circlenms_4x8_cyclic_20e_nus_20200925_230905-358fbe3b.pth'
		# checkpoint_file='/home/luis/catkin_ws/src/carina_3d_detection/models/hv_pointpillars_fpn_sbn-all_4x8_2x_nus-3d_20200620_230405-2fa62f3d.pth'
		# checkpoint_file='/home/luis/catkin_ws/src/carina_3d_detection/models/hv_ssn_secfpn_sbn-all_2x16_2x_nus-3d_20201023_193737-5fda3f00.pth'
		# checkpoint_file='/home/luis/carla/TRACK/team_code/catkin_ws/src/perception/carina_3d_detection/models/hv_pointpillars_fpn_sbn-all_4x8_2x_nus-3d_20210826_104936-fca299c1.pth'
		# checkpoint_file='/home/luis/carla/TRACK/team_code/catkin_ws/src/perception/carina_3d_detection/checkpoints/hv_pointpillars_secfpn_6x8_160e_kitti-3d-3class_20220301_150306-37dc2420.pth'
		# checkpoint_file='/home/luis/mmdetection3d/work_dirs/hv_pointpillars_secfpn_6x8_160e_carla-3d-2class/epoch_10.pth'
		self.checkpoint_file = os.path.join(dir_pkg,'checkpoints','epoch_40.pth')

		# config_file ='/home/luis/mmdetection3d/configs/pointpillars/hv_pointpillars_secfpn_6x8_160e_kitti-3d-3class.py'
		# config_file ='/home/luis/mmdetection3d/configs/pointpillars/hv_pointpillars_secfpn_6x8_160e_carla-3d-2class.py'
		self.config_file = os.path.join(dir_pkg,'checkpoints','hv_pointpillars_secfpn_6x8_160e_carla-3d-2class.py')
		# config_file ='/home/luis/mmdetection3d/configs/second/hv_second_secfpn_6x8_80e_kitti-3d-car.py'
		# config_file ='/home/luis/mmdetection3d/configs/centerpoint/centerpoint_0075voxel_second_secfpn_circlenms_4x8_cyclic_20e_nus.py'
		# config_file ='/home/luis/mmdetection3d/configs/pointpillars/hv_pointpillars_fpn_sbn-all_4x8_2x_nus-3d.py'
		# config_file ='/home/luis/mmdetection3d/configs/ssn/hv_ssn_secfpn_sbn-all_2x16_2x_nus-3d.py'
		print("loading model: ",self.config_file)
		self.model = init_model(self.config_file, self.checkpoint_file, device=device)
		print("init detector ... ")
		self.score_thr=0.3
		self.points_lidar=None

		self.marker_obst_pub = rospy.Publisher('/carina/sensor/lidar/obst_3d_marker_array', MarkerArray, queue_size=1)
		self.obst_3d_pub = rospy.Publisher('/carina/sensor/lidar/obst_3d_array', ObstacleArray, queue_size=1)
		self.poses_obst_pub = rospy.Publisher('/carina/sensor/lidar/poses_3d_array', PoseArray, queue_size=1)
		# self.pub_bev_detections = rospy.Publisher('/carina/sensor/lidar/bev/obst_3d_array_img', Image, queue_size=1)


		self.lidar_sub = rospy.Subscriber("/carina/sensor/lidar/front/point_cloud", PointCloud2, self.lidar_point_cloud_cb)
		self.shutdown_sub = rospy.Subscriber('/carina/vehicle/shutdown', Bool, self.shutdown_cb, queue_size=1)

	def normalize_angle(self, angle):
		return math.atan2(np.sin(angle), np.cos(angle))

	def lidar_point_cloud_cb(self,data):
		start_time = time.time()
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
		self.points_lidar[:,3]=pc['intensity']
		# print('max intensity: ',np.max(self.points_lidar[:,3]))
		# print('min intensity: ',np.min(self.points_lidar[:,3]))

		# self.points_lidar[:,4]= 0 #pc['ring']

		# print(self.points_lidar)
		# self.dict_points_lidar = {'points': self.points_lidar.astype(np.float32)}


		# test a single image
		time_ini=time.time()
		self.result, self.data_inf = inference_detector(self.model, self.points_lidar.astype(np.float32))
		self.data_inf=None
		time_fin=time.time()
		# print('time inference', time_ini-time_fin)
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

		pred_scores = self.result.pred_instances_3d.scores_3d.to('cpu')
		pred_bboxes = self.result.pred_instances_3d.bboxes_3d.tensor.to('cpu')
		labels_3d = self.result.pred_instances_3d.labels_3d.to('cpu')
		# print(labels_3d)
		# if 'pts_bbox' in result[0].keys():
		# 	pred_bboxes = result[0]['pts_bbox']['boxes_3d'].tensor.numpy()
		# 	pred_scores = result[0]['pts_bbox']['scores_3d'].numpy()
		# else:
		# 	pred_bboxes = result[0]['boxes_3d'].tensor.numpy()
		# 	pred_scores = result[0]['scores_3d'].numpy()
		# 	labels_3d = result[0]['labels_3d'].numpy()
		# 	# print(pred_bboxes,pred_bboxes)

	    # filter out low score bboxes for visualization
		if self.score_thr > 0:
			inds = pred_scores > self.score_thr
			pred_bboxes = pred_bboxes[inds]
			labels_3d = labels_3d[inds]



		markerArray = MarkerArray()
		markerArray.markers=[]

		ob_array = ObstacleArray()
		ob_array.obstacle = []

		ps = PoseArray()
		ps.header.frame_id = "velodyne"
		ps.header.stamp = rospy.Time.now()
		ps.poses=[]

		# map_scale=8
		# size_image=700

		# bev_detections= np.zeros([size_image,size_image,3],dtype=np.uint8)
		# thickness=4

		# img_map=np.zeros((size_image,size_image,1), dtype=np.uint8)
		# img_map_dir=np.zeros((size_image,size_image,1), dtype=np.uint8)

		for box,label in zip(pred_bboxes,labels_3d):
			# print(box,label)

			marker = Marker()
			marker.header.frame_id = "velodyne"
			marker.type = marker.CUBE
			marker.action = marker.ADD
			marker.ns = "my_namespace";

			ob = Obstacle()
			ob.header.frame_id = "velodyne"
			ob.header.stamp = rospy.Time().now()#o.header.stamp#msg.header.stamp
			# ob.id = o.id
			ob.classes=[str(label)]
			ob.ns = 'signs_carla_tk4'
			ob.class_id=0

			ob.type = -1

			# marker scale
			marker.scale.x = box[4]
			marker.scale.y = box[3]
			marker.scale.z = box[5]

			# obstacle scale
			ob.scale.x = box[4]
			ob.scale.y = box[3]
			ob.scale.z = box[5]

			# marker color
			r=0.0
			g=1.0
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
			ori= tf.transformations.quaternion_from_euler(0.0, 0.0, yaw)

			marker.color.a = 0.7
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

			# ob orientaiton
			ob.pose.orientation.x = float(ori[0])
			ob.pose.orientation.y = float(ori[1])
			ob.pose.orientation.z = float(ori[2])
			ob.pose.orientation.w = float(ori[3])
			# ob position
			ob.pose.position.x = float(box[0])
			ob.pose.position.y = float(box[1])
			ob.pose.position.z = float(box[2])

			t = rospy.Duration(0.5) 
			marker.lifetime = t
			markerArray.markers.append(marker)

			ob.lifetime = rospy.Duration(0.5)
			ob.track_status = 1	
			ob_array.obstacle.append(ob)

			pose = Pose()
			pose.position.x = box[0]
			pose.position.y = box[1]
			pose.position.z = box[2]
			pose.orientation.x = ori[0]
			pose.orientation.y = ori[1]
			pose.orientation.z = ori[2]
			pose.orientation.w = ori[3]
			ps.poses.append( pose )

			# x_point=pose.position.y
			# y_point=pose.position.x

			# px = int(size_image/2)+int(x_point*map_scale)
			# py = int(size_image/3)+int(y_point*map_scale)





			# point_center=(int(px), int(py))
			# l_ag=ob.scale.x
			# w_ag=ob.scale.y

			# angle=180-np.rad2deg(self.normalize_angle(yaw))
			# # angle=angle+180
			# # print('angle ', angle)

			# if angle >= 90:
			# 	# angle= -angle
			# 	l_ag=ob.scale.x
			# 	w_ag=ob.scale.y
			# if angle > 0 and angle<90:
			# 	angle = angle
			# 	l_ag=ob.scale.y
			# 	w_ag=ob.scale.x
			# if angle >=-90 and angle<=0:
			# 	angle = angle
			# 	l_ag=ob.scale.x
			# 	w_ag=ob.scale.y
			# if angle<-90:
			# 	angle=-angle
			# 	l_ag=ob.scale.y
			# 	w_ag=ob.scale.x



			# rect=((point_center), (int(l_ag*map_scale/2),int(w_ag*map_scale/2)),  angle)

			# box = cv2.boxPoints(rect) # cv2.boxPoints(rect) for OpenCV 3.x
			# box = np.int0(box)
			# cv2.drawContours(img_map,[box],-1,color_past_ag,-1)

			# cv2.drawContours(img_map_agents[i],[box],-1,color_past_ag,-1)
			# cv2.drawContours(bev_detections,[box],-1,(255,0,0),-1)
			# cv2.drawContours(bev_detections[17-3],[box],-1,(int(v)),-1)


			# dir_value_pixel= ((self.normalize_angle(yaw) + (math.pi)) *255)/(2*math.pi)

			# print(dir_value_pixel)

			# cv2.drawContours(img_map,[box],-1,255,-1)
			# cv2.drawContours(img_map_dir,[box], -1, int(dir_value_pixel), -1)

			# cv2.circle(img_map,(px, py), 10, 255, -1)
			# cv2.circle(img_map_dir,(px, py), 10, int(dir_value_pixel), -1)





		id = 0
		for m in markerArray.markers:
		   m.id = id
		   id += 1

		id = 0
		for o in ob_array.obstacle:
		   o.id = id
		   id += 1


		# img_map_mask=cv2.cvtColor(img_map, cv2.COLOR_BGR2GRAY)
		# img_map_mask=img_map
		# ret,img_map_mask = cv2.threshold(img_map_mask,1,255,cv2.THRESH_BINARY)


		# img_map_dir_color=cv2.applyColorMap(img_map_dir, cv2.COLORMAP_HSV)
		# img_map_dir_color = cv2.bitwise_and(img_map_dir_color,img_map_dir_color, mask= img_map_mask)

			# cv2.circle(bev_detections, (px, py), 10, (255,0,0), thickness)


		self.marker_obst_pub.publish(markerArray)
		self.obst_3d_pub.publish(ob_array)

		self.poses_obst_pub.publish( ps )

		# if self.pub_bev_detections.get_num_connections() != 0:
		# 	try:
		# 		# self.pub_bev_detections.publish(self.cvbridge.cv2_to_imgmsg(bev_detections, "bgr8"))
		# 		self.pub_bev_detections.publish(self.cvbridge.cv2_to_imgmsg(img_map_dir_color, "bgr8"))

		# 	except CvBridgeError as e:
		# 		print (e)
		print("--- %s seconds ---" % (time.time() - start_time))

	def shutdown_cb(self, msg):
		if msg.data:

			self.model=None
			self.points_lidar=None
			self.checkpoint_file = None
			self.config_file = None
			self.score_thr=None

			self.result=None 
			self.data_inf=None 

			self.lidar_sub = None
			self.marker_obst_pub = None
			self.obst_3d_pub = None
			self.poses_obst_pub = None
			self.shutdown_sub = None
			# torch.cuda.empty_cache()
			# print('empty_cache')

			print ("Bye!")
			rospy.signal_shutdown("finished route")



if __name__ == '__main__':
	rospy.init_node("pointcloud_3d_detection", anonymous=True)
	print ("[3D detection pointcloud] running...")
	dataset=PointCloud3Ddetection()
	rospy.spin()



