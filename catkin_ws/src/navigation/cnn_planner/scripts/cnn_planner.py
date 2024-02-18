#!/usr/bin/env python3
import torch
from torch import nn#, optim
from torch.utils.data import DataLoader
from torchvision.models.resnet import resnet18#, ResNet18_Weights
from torchvision import transforms#, utils 

import rospkg
import rospy
from geometry_msgs.msg import PoseWithCovarianceStamped, PoseStamped#, TransformStamped, Transform, Vector3, Quaternion#, Twist
from msgs_action.msg import VehicleState#, SteeringAngle, Throttle, Brake
from nav_msgs.msg import Path as RosPath
from msgs_navigation.msg import Path as NavPath
from msgs_navigation.msg import TrajectoryPoint, GlobalPlan
from sensor_msgs.msg import Image#, PointCloud2, CameraInfo
from std_msgs.msg import Bool, Header# Float64, 
from visualization_msgs.msg import Marker, MarkerArray
import tf2_ros
from tf2_geometry_msgs import *
import tf
from cv_bridge import CvBridge, CvBridgeError
from msgs_perception.msg import ObstacleArray, Obstacle


from scipy import interpolate
import numpy as np
import os
import cv2
from collections import deque
from path_optimizer import *

from shapely.geometry import LineString#, get_point
import copy
from enum import Enum

from torchvision.utils import save_image
class CNNPlanner(object):
	def __init__ (self):

		self.tf2_buffer_vel2map = tf2_ros.Buffer()
		self.tf2_listener_vel2map = tf2_ros.TransformListener(self.tf2_buffer_vel2map)

		self.tf2_buffer_map2vel = tf2_ros.Buffer()
		self.tf2_listener_map2vel = tf2_ros.TransformListener(self.tf2_buffer_map2vel)

		self.time_threshold_to_pub_path=8 #publish path each 8 secs

		self.stamp=rospy.Time.now()

		# self.steering_angle=None
		self.speed=0.
		self.current_pose=None
		self.old_pose=None
		self.old_poses_list=None
		self.cvbridge = CvBridge()
		self.rgb_image =None
		self.stereo_bev_image =None
		self.lidar_bev_image =None
		self.global_plan=None
		self.next_index_goal_plan=0
		# self.len_global_plan=None
		self.foot_print_objects_image=None

		self.last_time_run=rospy.Time.now().to_sec()
		self.publishing_replanning=False
		self.time_last_path=-100#rospy.Time.now().to_sec()
		self.time_last_replanning=rospy.Time.now().to_sec()

		self.LANEFOLLOW=0
		self.STRAIGHT=1
		self.RIGHT=2
		self.LEFT=3
		self.CHANGELANELEFT=4
		self.CHANGELANERIGHT=5
		self.UNKNOWN=6

		# self.first_frame=True
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

		self.msg_replan_path_p = RosPath()
		self.msg_replan_path_p.header.frame_id='map'
		self.msg_replan_path_p.poses = []

		self.replan_path_msg = NavPath()
		self.replan_path_msg.header.frame_id = 'map'
		self.replan_path_msg.path = []

		self.raw_path_plan_points=[]
		# self.markerArray = MarkerArray()
		# self.markerArray.markers=[]
		# self.intersections_markerArray = MarkerArray()
		# self.intersections_markerArray.markers=[]

		self.last_path_published=None
		self.last_path_published_ros=None

		self.len_tfl_stop=0
		self.stoped_threshold=100

		self.obstacles_frame_0 =[]


		rospack = rospkg.RosPack()
		dir_pkg=rospack.get_path('cnn_planner')

		# ==== INIT MODEL ====
		# self.device = torch.device("cuda:0" if torch.cuda.is_available() else "cpu")
		gpu_device = rospy.get_param("/gpu_device_stack_param")
		self.do_replanning=rospy.get_param("/navigation/replanning")#parameters["dataset"]["create_dataset_path"]#False


		# gpu_device='cuda:0'
		print('gpu_device stack ', gpu_device)
		self.device=gpu_device

		use_cuda = torch.cuda.is_available()#not args.no_cuda and torch.cuda.is_available()
		# torch.manual_seed(10)#args.seed)

		self.test_kwargs = {'batch_size': 1}#args.test_batch_size}
		if use_cuda:
		    cuda_kwargs = {'num_workers': 1,
		                   'pin_memory': True,
		                   'shuffle': False}
		                    # 'shuffle': True}
		    # train_kwargs.update(cuda_kwargs)
		    self.test_kwargs.update(cuda_kwargs)




		self.data=None 
		self.output=None
		channels=14
		predicted_points=200
		# self.model = Resnet18Model(predicted_points,channels)
		fusion_strategy="none"
		self.model = Resnet18Model(predicted_points,channels, fusion_strategy=fusion_strategy, device=self.device)
		self.model.to(self.device)
		# model_name='resnet18'
		model_name='cnn-planner-resnet18'
		weight_path = os.path.join(dir_pkg,'checkpoints',model_name,model_name+'_none_100.pt')		
		if weight_path:
			print('weight_path: ',weight_path)
			checkpoint=torch.load(weight_path)
			self.model.load_state_dict(checkpoint['model_state_dict'])
			# self.model.load_state_dict(weight_path['state_dict'], strict=False)
		self.model.eval()

		self.scale=8
		self.size_image=700


		# self.global_plan_points_png= numpy.zeros([self.size_image, self.size_image,3],dtype=np.uint8)
		# self.global_plan_line_png= numpy.zeros([self.size_image, self.size_image,1],dtype=np.uint8)
		# self.gps_backward_png= numpy.zeros([self.size_image, self.size_image,1],dtype=np.uint8)
			# path_pred_png= numpy.zeros([size_image,size_image,1],dtype=np.uint8)
		self.pose_sub = rospy.Subscriber('/carina/localization/pose', PoseWithCovarianceStamped, self.pose_cb, queue_size=1)
		self.state_sub = rospy.Subscriber('/carina/vehicle/state', VehicleState, self.vehicle_state_cb, queue_size=1)
		self.image_bev_sub =        rospy.Subscriber('/carina/sensor/lidar/bev_point_cloud', Image, self.lidar_bev_imageCallback, queue_size=1)
		self.image_bev_stereo_sub = rospy.Subscriber('/carina/sensor/stereo/bev_rgb_point_cloud', Image, self.stereo_bev_imageCallback, queue_size=1)
		self.global_plan_raw_sub = rospy.Subscriber('/carina/navigation/global_plan_raw', GlobalPlan, self.global_plan_cb, queue_size=1)
		self.shutdown_sub = rospy.Subscriber('/carina/vehicle/shutdown', Bool, self.shutdown_cb, queue_size=1)
		self.shutdown_using_map_sub = rospy.Subscriber('/carina/map/shutdown_using_map', Bool, self.shutdown_using_map_cb, queue_size=1)
		self.odom_objs_tfsign_sub = rospy.Subscriber("/carina/perception/stereo/traffic_sign_odom", ObstacleArray, self.obj_traffic_light_stop_sub, queue_size = 1)

		self.odom_objs_tfsign_sub = rospy.Subscriber("/carina/sensor/lidar/obst_3d_array", ObstacleArray, self.dinamic_objects_sub, queue_size = 1)

		self.first_point_achived = rospy.Publisher('/carina/map/first_point_achived', Bool, queue_size=1)		# self.radar_pub_front = rospy.Publisher('/carina/sensor/radar/front/obstacles_array', ObstacleArray, queue_size=1)
		# self.marker_route_pub = rospy.Publisher('/carina/route/points/route_points_array', MarkerArray, queue_size=1)
		self.path_ros_pub = rospy.Publisher('/carina/navigation/cnn_path_ros', RosPath, queue_size=1)#same than local_path_planing_node
		self.path_pub = rospy.Publisher("/carina/navigation/cnn_path", NavPath,queue_size=1)#same than local_path_planing_node
		self.pub_intersections_obj = rospy.Publisher('/carina/navigation/next_intersection', MarkerArray, queue_size=1)
		self.pub_global_plan_img = rospy.Publisher('/carina/debug/cnn/global_plan_img', Image, queue_size=1)
		self.global_plan_line_img = rospy.Publisher('/carina/debug/cnn/global_plan_line_img', Image, queue_size=1)

		self.foot_print_objects_pub = rospy.Publisher('/carina/sensor/lidar/bev/foot_print_objects', Image, queue_size=1)


	def normalize_angle(self, angle):
		return math.atan2(np.sin(angle), np.cos(angle))


	def dinamic_objects_sub(self, msg):

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
			euler = tf.transformations.euler_from_quaternion(quaternion)
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
					# print('ocupped')
					continue


			quaternion = ( velo_obs.pose.orientation.x, velo_obs.pose.orientation.y, velo_obs.pose.orientation.z, velo_obs.pose.orientation.w )
			euler = tf.transformations.euler_from_quaternion(quaternion)
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



		self.foot_print_objects_image=img_map_dir_color


		if self.foot_print_objects_pub.get_num_connections() != 0:
			try:
				# self.pub_bev_detections.publish(self.cvbridge.cv2_to_imgmsg(bev_detections, "bgr8"))
				self.foot_print_objects_pub.publish(self.cvbridge.cv2_to_imgmsg(self.foot_print_objects_image, "bgr8"))

			except CvBridgeError as e:
				print (e)



		if len(self.obstacles_frame_0) > 2000:
			self.obstacles_frame_0=self.obstacles_frame_0[:2000]

		# print('obstacles_frame_0' , len(self.obstacles_frame_0))



	def obj_traffic_light_stop_sub(self,msg):
		obj_tfl_stop=msg
		self.len_tfl_stop=len(obj_tfl_stop.obstacle)
		if self.len_tfl_stop > 0:
			self.stoped_threshold=120
		else:
			self.stoped_threshold=100

	def global_plan_cb(self,msg):
		self.global_plan=msg
		# self.len_global_plan=len(self.global_plan.points)

	def vehicle_state_cb(self,msg):
		self.speed = msg.drive.speed
		# self.steering_angle=msg.drive.steering_angle
		
	def pose_cb(self,msg):
		self.stamp=msg.header.stamp
		self.current_pose = msg

		if self.old_poses_list==None:
			self.old_poses_list = [[self.current_pose] for i in range(200)]
		else:
			deque_old_poses_list = deque(self.old_poses_list) 
			deque_old_poses_list.rotate(1) 
			self.old_poses_list = list(deque_old_poses_list)
			self.old_poses_list[0] = [self.current_pose]

		try:
			trans_map2vel = self.tf2_buffer_map2vel.lookup_transform('velodyne', 'map', rospy.Time())
		except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as exept:
			print(exept)
			return

		global_plan=self.global_plan

		raw_path_plan_points=[]
		color_plan=[]
		# next_intersection=None

		# print('pose')
		for index, (pose, option) in enumerate(zip(global_plan.points, global_plan.road_options)):
			old_pose = PoseStamped()
			old_pose.header.frame_id='map'
			old_pose.header.stamp = self.stamp
			old_pose.pose.position = pose

			new_pose = tf2_geometry_msgs.do_transform_pose(old_pose, trans_map2vel)
			raw_path_plan_points.append([new_pose.pose.position.y*self.scale+(self.size_image/2), new_pose.pose.position.x*self.scale+(self.size_image/3)])
			# marker = Marker()
			# marker.header.frame_id = "velodyne"
			# marker.type = marker.SPHERE
			# marker.action = marker.ADD
			# marker.ns = "my_namespace";
			# # marker scale
			# marker.scale.x = 5.3
			# marker.scale.y = 5.3
			# marker.scale.z = 5.3
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
			# marker.pose.position.z = -20
			# t = rospy.Duration(5.0) 
			# marker.lifetime = t
			# self.markerArray.markers.append(marker)
			# if self.STRAIGHT==option or self.RIGHT==option or self.LEFT==option :
			# 	self.intersections_markerArray.markers.append(marker)
			if index==self.next_index_goal_plan:          
				dist_to_goal =np.linalg.norm(np.array([pose.x, pose.y]) - 
														np.array([self.current_pose.pose.pose.position.x, self.current_pose.pose.pose.position.y]))#dist between ego and the next goal
				if dist_to_goal<7.:  ## if the ego achieve the goal// the vehicle is in a distance < 10 meter of the point
					self.next_index_goal_plan=self.next_index_goal_plan+1  #next goal
			if self.next_index_goal_plan>=2:
				signal_first_point_achived = Bool()
				signal_first_point_achived.data = True
				self.first_point_achived.publish(signal_first_point_achived)

		self.raw_path_plan_points = raw_path_plan_points
		self.color_plan=color_plan
		# id = 0
		# for m in self.markerArray.markers:
		#    m.id = id
		#    id += 1
		# if self.pub_intersections_obj.get_num_connections() != 0:
		# 	id = 0
		# 	for m in self.intersections_markerArray.markers:
		# 	   m.id = id
		# 	   id += 1
		# 	self.pub_intersections_obj.publish(self.intersections_markerArray)


		if self.stereo_bev_image is None or self.current_pose is None or self.lidar_bev_image is None or self.global_plan is None or self.global_plan.points==[] or \
									self.old_poses_list is None and len(self.raw_path_plan_points or self.foot_print_objects_image is None)>0:
			print('return')
			return

		current_pose=copy.deepcopy(self.current_pose)


		if self.old_pose==None: #or global_plan==None:
			self.old_pose=current_pose
			dist=10000
		else:
			dist=np.linalg.norm(np.array([self.old_pose.pose.pose.position.x, self.old_pose.pose.pose.position.y]) - np.array([current_pose.pose.pose.position.x, current_pose.pose.pose.position.y]))
		
		# if self.first_frame:
		# 	self.first_frame=False
		# 	dist=10000 





		if self.speed > 0.5:
			self.last_time_run = rospy.Time().now().to_sec()#msg.header.stamp.to_sec()

		time_stoped = (rospy.Time().now().to_sec() - self.last_time_run)
		time_from_last_path = (rospy.Time().now().to_sec() - self.time_last_path)
		time_from_last_replanning = (rospy.Time().now().to_sec() - self.time_last_replanning)



		# print('cnn planner: time_stoped ',time_stoped, 'time_from_last_path ', time_from_last_path, 'time_from_last_replanning ',time_from_last_replanning)


		if time_stoped > 10.:
			self.time_threshold_to_pub_path=1.

		# print( dist, time_from_last_path, self.time_threshold_to_pub_path, time_from_last_replanning)

		if dist>11.0 or (time_from_last_path > self.time_threshold_to_pub_path \
			 and time_from_last_replanning > 12.0):


			stamp = rospy.Time().now()#self.stamp#im.header.stamp#rospy.Time().now()
			try:
				trans_map2vel = self.tf2_buffer_map2vel.lookup_transform('velodyne', 'map', rospy.Time())
			except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as exept:
				print(exept)
				return


			global_plan_points_png= np.zeros([self.size_image, self.size_image,3],dtype=np.uint8)
			global_plan_line_png= np.zeros([self.size_image, self.size_image,1],dtype=np.uint8)
			gps_backward_png= np.zeros([self.size_image, self.size_image,1],dtype=np.uint8)


			poses_back=[]
			poses_pred_gt=[]
			old_poses_list=copy.deepcopy(self.old_poses_list)
			for p in old_poses_list:
				new_tfmed_gps = tf2_geometry_msgs.do_transform_pose(p[0].pose, trans_map2vel)
				poses_back.append( [(new_tfmed_gps.pose.position.y*self.scale)+(self.size_image/2), (new_tfmed_gps.pose.position.x*self.scale)+(self.size_image/3)] )
			pts = np.array(poses_back, np.int32)
			pts = pts.reshape((-1,1,2))
			cv2.polylines(gps_backward_png,[pts],False,(255),thickness=4)


			# thickness=-1

			init_interval=self.next_index_goal_plan-2
			end_interval=self.next_index_goal_plan+3
			if init_interval<0:
				init_interval=0
			if end_interval>=len(self.raw_path_plan_points):
				end_interval=len(self.raw_path_plan_points)
			raw_local_points=[]
			for i in range(init_interval, end_interval):
				raw_local_points.append(self.raw_path_plan_points[i])
				color_point=self.color_plan[i]
				point_local=self.raw_path_plan_points[i]
				# if self.pub_global_plan_img.get_num_connections() != 0:
				cv2.circle(global_plan_points_png, (int(point_local[0]), int(point_local[1])), 10, color_point, thickness=-1)

			global_plan_points_png_rgb = global_plan_points_png[...,[2,1,0]]

			# thickness=4
			pts = np.array(raw_local_points, np.int32)
			pts = pts.reshape((-1,1,2))
			cv2.polylines(global_plan_line_png,[pts],False,(255),thickness=4)	


			# pts = np.array(poses_pred_gt, np.int32)
			# pts = pts.reshape((-1,1,2))	
			# cv2.polylines(path_pred_png,[pts],False,(255))	

			stereo_bev_image=copy.deepcopy(self.stereo_bev_image)
			lidar_bev_image=copy.deepcopy(self.lidar_bev_image)
			im_color_lidar = cv2.applyColorMap(lidar_bev_image, cv2.COLORMAP_JET)
			im_color_lidar = cv2.bitwise_and(im_color_lidar, im_color_lidar, mask=lidar_bev_image)
			im_color_lidar = cv2.cvtColor(im_color_lidar, cv2.COLOR_BGR2RGB)
			stereo_bev_image = cv2.cvtColor(stereo_bev_image, cv2.COLOR_BGR2RGB)
			foot_print_objects_image= copy.deepcopy(self.foot_print_objects_image)

			foot_print_objects_image_rgb = foot_print_objects_image[...,[2,1,0]]





			test_dataset = CarlaDatset(im_color_lidar , foot_print_objects_image_rgb, stereo_bev_image , 
				global_plan_points_png_rgb , global_plan_line_png, gps_backward_png, root_dir='', train=False)


			test_dataloader = DataLoader(test_dataset, **self.test_kwargs)
			self.output=None
			try:
				with torch.no_grad():
					for data in test_dataloader:
						data_test=data["image_inputs"]
						self.data = data_test.to(self.device)#, target.to(device)
						self.output = self.model(self.data).reshape(-1,2)#.reshape(targets.shape)
			except Exception as e:
				print(e)
				return
			if self.output is None or len(self.output)<30:
				print('except: ', len(self.output))
				# self.first_frame=True
				return
			try:
				transform_to_world= self.tf2_buffer_vel2map.lookup_transform('map', 'velodyne', rospy.Time())
			except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as exept:
				print(exept)
				return


			msg_path = RosPath()
			msg_path.header.frame_id='map'
			msg_path.poses = []

			for p in self.output:
				p=p.tolist()
				ps = PoseStamped()
				ps.header.stamp = rospy.Time().now()#stamp#
				ps.pose.position.x = p[0] 
				ps.pose.position.y = p[1] 
				to_word_point_path = tf2_geometry_msgs.do_transform_pose(ps, transform_to_world)
				msg_path.poses.append(to_word_point_path)
			msg_path.header.stamp =  rospy.Time().now()#stamp#
			try:
				path_imp=self.improve_waypoints(msg_path)
			except Exception as e:
				# print('msg_path ', msg_path.poses)
				# print('improve_waypoints Exception')
				# self.first_frame=True
				# print(e)
				return
			count = 0


			self.msg_path_p.poses = []
			self.path_msg.path = []
			# self.intersections_markerArray.markers=[]

			for w in path_imp:
				t = TrajectoryPoint()
				t.point = w
				t.point_number = count
				t.end_track = False
				count = count + 1
				self.path_msg.path.append(t)

				ps = PoseStamped()
				ps.header.stamp = rospy.Time().now()#stamp#
				ps.pose.position.x = w[0] 
				ps.pose.position.y = w[1] 
				ps.pose.position.z = current_pose.pose.pose.position.z
				self.msg_path_p.poses.append(ps)#to_word_point_path)

			self.path_msg.path[-1].end_track = False#True

			self.old_pose=current_pose
			# self.last_path_time=rospy.Time().now()

			self.msg_path_p.header.stamp =  rospy.Time().now()
			self.path_msg.header.stamp = rospy.Time().now()#stamp#
			self.path_pub.publish(self.path_msg)
			self.path_ros_pub.publish(self.msg_path_p)

			self.last_path_published=copy.deepcopy(self.path_msg)
			# self.last_path_published_ros=self.msg_path_p
			self.time_last_path=rospy.Time().now().to_sec()

			self.publishing_replanning=False




			if self.pub_global_plan_img.get_num_connections() != 0:
				try:
					self.pub_global_plan_img.publish(self.cvbridge.cv2_to_imgmsg(global_plan_points_png, "bgr8"))
				except CvBridgeError as e:
					print (e)
			if self.global_plan_line_img.get_num_connections() != 0:
				try:
					self.global_plan_line_img.publish(self.cvbridge.cv2_to_imgmsg(global_plan_line_png, "mono8"))
				except CvBridgeError as e:
					print (e)
		######################################################################
		##############################replanning##############################
		######################################################################

		# if self.speed > 0.5:
		# 	self.last_time_run = rospy.Time().now()#msg.header.stamp.to_sec()

		# time_stoped = (rospy.Time().now() - self.last_time_run).to_sec()
		time_from_last_path = rospy.Time().now().to_sec() - self.time_last_path
		# time_from_last_replanning = (rospy.Time().now() - self.time_last_replanning).to_sec()

		# print('cnn planner: time_stoped ',time_stoped, 'time_from_last_path ', time_from_last_path, 'time_from_last_replanning ',time_from_last_replanning, 'do_replanning ', self.do_replanning)
		if time_stoped > self.stoped_threshold and not(self.publishing_replanning) and time_from_last_path > 2. \
			and time_from_last_replanning > 200 and self.last_path_published is not None \
			and self.do_replanning:
			
			self.publishing_replanning=True
			self.replan_path_msg.path = []
			# self.path_msg.path  = []
			path=[]
			for p in self.last_path_published.path:
				path.append(p.point[:2])

			path_shapely = LineString(path)
			offset_path = path_shapely.parallel_offset(2., 'left', join_style=1)
			x, y = offset_path.xy
			# print(x,y)
			msg_path = RosPath()
			msg_path.header.frame_id='map'
			msg_path.poses = []
			self.msg_replan_path_p.poses = []

			ps = PoseStamped()
			ps.header.stamp = rospy.Time().now()#stamp#
			ps.pose.position.x = current_pose.pose.pose.position.x
			ps.pose.position.y = current_pose.pose.pose.position.y
			msg_path.poses.append(ps)

			psi = PoseStamped()
			psi.header.stamp = rospy.Time().now()#stamp#
			psi.pose.position.x = (current_pose.pose.pose.position.x + x[10])/2
			psi.pose.position.y = (current_pose.pose.pose.position.y + y[10])/2
			msg_path.poses.append(psi)

			for x,y in zip(x[10:-1],y[10:-1]):
				ps = PoseStamped()
				ps.header.stamp = rospy.Time().now()#stamp#
				ps.pose.position.x = x 
				ps.pose.position.y = y

				msg_path.poses.append(ps)
			msg_path.header.stamp =  rospy.Time().now()#stamp#

			try:
				path_imp=self.improve_waypoints(msg_path)
			except Exception as e:
				# self.first_frame=True
				# print('exception improve_waypoints')
				print(e)
				return

			count = 0
			for w in path_imp:
				t = TrajectoryPoint()
				t.point = w
				t.point_number = count
				t.end_track = False
				count = count + 1
				self.replan_path_msg.path.append(t)

				ps = PoseStamped()
				ps.header.stamp = rospy.Time().now()
				ps.pose.position.x = w[0] 
				ps.pose.position.y = w[1]
				ps.pose.position.z = current_pose.pose.pose.position.z
				self.msg_replan_path_p.poses.append(ps)

			self.time_last_replanning=rospy.Time().now().to_sec()
			self.replan_path_msg.path[-1].end_track = False

			self.replan_path_msg.header.stamp = rospy.Time().now()#stamp#
			self.path_pub.publish(self.replan_path_msg)

			self.msg_replan_path_p.header.stamp =  rospy.Time().now()
			self.path_ros_pub.publish(self.msg_replan_path_p)





	def lidar_bev_imageCallback(self,im):
		try:
			self.lidar_bev_image = self.cvbridge.imgmsg_to_cv2(im, "mono8")
		except CvBridgeError as e:
			print (e)

	def stereo_bev_imageCallback(self,im):
		try:
			self.stereo_bev_image = self.cvbridge.imgmsg_to_cv2(im, "bgr8")
		except CvBridgeError as e:
			print (e)


	def improve_waypoints(self, msg):
		self.waypoints = np.array([])
		self.index=0
		way = [[p.pose.position.x, p.pose.position.y, p.pose.position.z] for p in msg.poses]
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
		return self.waypoints


	def shutdown_cb(self, msg):
		if msg.data:

			self.stamp=None
			# self.steering_angle=None
			self.speed=None
			self.current_pose=None
			self.old_pose=None
			self.old_poses_list=None
			self.cvbridge = None
			self.rgb_image =None
			self.stereo_bev_image =None
			self.lidar_bev_image =None
			self.global_plan=None
			self.next_index_goal_plan=None
			# self.len_global_plan=None
			# self.first_frame=None
			self.waypoints = None
			self.precision =None
			self.window = None
			self.index = None
			self.msg_path_p = None
			self.msg_replan_path_p = None
			self.path_msg = None
			self.replan_path_msg = None

			# self.intersections_markerArray = None
			self.model = None
			self.data=None 
			self.output=None
			self.device = None

			self.tf2_buffer_map2vel = None
			self.tf2_listener_map2vel = None

			self.tf2_buffer_vel2map = None
			self.tf2_listener_vel2map = None

			self.pose_sub = None
			self.state_sub = None
			self.image_bev_sub =None
			self.image_bev_stereo_sub = None
			self.global_plan_raw_sub = None
			self.shutdown_sub = None
			self.shutdown_using_map_sub = None

			self.first_point_achived = None	
			self.path_ros_pub = None
			self.path_pub = None
			self.pub_intersections_obj = None
			self.pub_global_plan_img = None
			self.global_plan_line_img = None

			torch.cuda.empty_cache()
			print ("Bye!")
			rospy.signal_shutdown("finished route")

	def shutdown_using_map_cb(self, msg):
		if msg.data:
			self.stamp=None
			# self.steering_angle=None
			self.speed=None
			self.current_pose=None
			self.old_pose=None
			self.old_poses_list=None
			self.cvbridge = None
			self.rgb_image =None
			self.stereo_bev_image =None
			self.lidar_bev_image =None
			self.global_plan=None
			self.next_index_goal_plan=None
			# self.len_global_plan=None
			# self.first_frame=None
			self.waypoints = None
			self.precision =None
			self.window = None
			self.index = None
			self.msg_path_p = None
			self.msg_replan_path_p = None
			self.replan_path_msg = None
			# self.intersections_markerArray = None
			self.device = None
			self.model = None

			self.tf2_buffer_map2vel = None
			self.tf2_listener_map2vel = None

			self.tf2_buffer_vel2map = None
			self.tf2_listener_vel2map = None

			self.pose_sub = None
			self.state_sub = None
			self.image_bev_sub =None
			self.image_bev_stereo_sub = None
			self.global_plan_raw_sub = None
			self.shutdown_sub = None
			self.shutdown_using_map_sub = None

			self.first_point_achived = None	
			self.path_ros_pub = None
			self.path_pub = None
			self.pub_intersections_obj = None
			self.pub_global_plan_img = None
			self.global_plan_line_img = None

			print ("Bye!")
			rospy.signal_shutdown("shutdown cnn planner because new opendrive map")

class Resnet18Model(nn.Module):
    
    class FusionStrategy(Enum):
        NONE:int = 1
        MOE :int = 2
        MOEA_LINEAR:int=3
        MOEA_HEURISTIC:int=4

        @staticmethod
        def from_str(gate_type:str):
            for _typ, names in Resnet18Model.FusionStrategy.supported_dict().items():
                if gate_type.lower().strip() in names:
                    return _typ
            
            raise NotImplementedError
        
        @staticmethod
        def supported_dict():
            return {
                Resnet18Model.FusionStrategy.NONE:["none", "n"],
                Resnet18Model.FusionStrategy.MOE:["moe", "m"],
                Resnet18Model.FusionStrategy.MOEA_LINEAR:["linear", "moea-linear", "l"],
                Resnet18Model.FusionStrategy.MOEA_HEURISTIC:["heuristic", "moea-heuristic", "h"]
            }
        
    def __init__(self, 
                 points_planner:int,
                 num_in_channels:int,
                 fusion_strategy:str,
                 device:torch.device=torch.device("cpu"),
                 **kwargs
    ):
        
        super(Resnet18Model, self).__init__(**kwargs)

        self.fusion_strategy:Resnet18Model.FusionStrategy=\
            Resnet18Model.FusionStrategy.from_str(fusion_strategy)
        
        self.device = device
        
        # self.backbone = resnet18(weights=ResNet18_Weights.DEFAULT, progress=True)
        self.backbone = resnet18(pretrained=False, progress=False)

        num_in_channels = num_in_channels#

        self.backbone.conv1 = nn.Conv2d(
            num_in_channels,
            self.backbone.conv1.out_channels,
            kernel_size=self.backbone.conv1.kernel_size,
            stride=self.backbone.conv1.stride,
            padding=self.backbone.conv1.padding,
            bias=False,
        )
        # This is 512 for resnet18 and resnet34;
        # And it is 2048 for the other resnets
        backbone_out_features = 512
        # X, Y coords for the future positions (output shape: Bx50x2)
        num_targets = 2 * points_planner#cfg["model_params"]["future_num_frames"]
        # You can add more layers here.
        self.head = nn.Sequential(
            # nn.Dropout(0.2),
            nn.Linear(in_features=backbone_out_features, out_features=4096),
        )
        self.logit = nn.Linear(4096, out_features=num_targets)

        if self.fusion_strategy == Resnet18Model.FusionStrategy.MOEA_LINEAR:
            self.fusion_model = MixtureOfExpertsAttention(
                                    name = "MOEA_linear",
                                    num_experts = 6,
                                    num_heads=4,
                                    key_dim = backbone_out_features,
                                    embed_dim= backbone_out_features,
                                    dropout=0.3,
                                    gate_type="linear",
                                    gate_units=2*backbone_out_features,
                                    seq_dim=(points_planner, 2),
                                    device=self.device
                                )
            self.fusion_model.to(self.device)
        elif self.fusion_strategy == Resnet18Model.FusionStrategy.MOEA_HEURISTIC:
            self.fusion_model = MixtureOfExpertsAttention(
                            name = "MOEA_heuristic",
                            num_experts = 6,
                            num_heads=4,
                            key_dim = backbone_out_features,
                            embed_dim= backbone_out_features,
                            dropout=0.3,
                            gate_type="heuristic",
                            gate_units=2*backbone_out_features,
                            seq_dim=(points_planner, 2),
                            device=self.device
                        )
            self.fusion_model.to(self.device)
        elif self.fusion_strategy == Resnet18Model.FusionStrategy.MOE:
            self.fusion_model = MixtureOfExpertsNetwork(
                                    name="MOE",
                                    num_experts=6,
                                    input_dim=backbone_out_features,
                                    units = backbone_out_features,
                                    dropout=0.3,
                                    device=self.device
                                 )
            self.fusion_model.to(self.device)

    def feature_fusion(self,
                       x:torch.Tensor,
                       command:torch.Tensor=None
    )-> torch.Tensor:
        
        batch_size = x.size(0)

        if self.fusion_strategy == Resnet18Model.FusionStrategy.NONE:
            return x

        elif self.fusion_strategy == Resnet18Model.FusionStrategy.MOE:
            x = self.fusion_model(x)
            return x
        
        elif self.fusion_strategy == Resnet18Model.FusionStrategy.MOEA_LINEAR:
            x = self.fusion_model(
                    query=x,
                    value=x,
                    key=x,
                    attention_mask=None,
                    return_attention_scores=False,
                    prior_trajectory=None,
                    command = None
                )
            
            return x
        elif self.fusion_strategy == Resnet18Model.FusionStrategy.MOEA_HEURISTIC:
            x_prior = self.head(x)
            x_prior = self.logit(x_prior)
            x_prior = x_prior.reshape(batch_size, -1, 2)

            x = self.fusion_model(
                    query=x,
                    value=x,
                    key=x,
                    attention_mask=None,
                    return_attention_scores=False,
                    prior_trajectory=x_prior,
                    command = command
                )
            
            return x
        else:
            raise RuntimeError(f"Fusion strategy not found: {self.fusion_strategy}!")
    
    def forward(self, x, command=None):

        x = self.backbone.conv1(x)
        x = self.backbone.bn1(x)
        x = self.backbone.relu(x)
        x = self.backbone.maxpool(x)

        x = self.backbone.layer1(x)
        x = self.backbone.layer2(x)
        x = self.backbone.layer3(x)
        x = self.backbone.layer4(x)

        x = self.backbone.avgpool(x)
        x = torch.flatten(x, 1)

        #feature fusion strategy
        x = self.feature_fusion(x=x, command=command)
        
        #smooth estimation
        x = self.head(x)
        x = self.logit(x)
        
        return x

class CarlaDatset():
    """Face path dataset."""
    def __init__(self, img_lidar_color , foot_print_objects_image, img_stereo , img_global_plan_points , 
    	img_global_plan_line , img_gps_backward,root_dir='',  train=True,transform=None):

        self.leaderboard_frame = [img_lidar_color]
        self.transform = transform

        # self.img_lidar  = img_lidar
        self.foot_print_objects_image=foot_print_objects_image
        self.img_lidar_color  = img_lidar_color
        self.img_stereo  = img_stereo
        self.img_global_plan_points  = img_global_plan_points
        self.img_global_plan_line  = img_global_plan_line#
        self.img_gps_backward  = img_gps_backward#

    def __len__(self):
        return len(self.leaderboard_frame)

    def __getitem__(self, idx):
        if torch.is_tensor(idx):
            idx = idx.tolist()

        img_lidar_color        =  transforms.ToTensor()(self.img_lidar_color  )
        img_stereo             =  transforms.ToTensor()(self.img_stereo  )
        img_global_plan_points =  transforms.ToTensor()(self.img_global_plan_points  )
        img_global_plan_line   =  transforms.ToTensor()(self.img_global_plan_line  )
        img_gps_backward       =  transforms.ToTensor()(self.img_gps_backward  )
        foot_print_objects_image       =  transforms.ToTensor()(self.foot_print_objects_image  )


        # save_image(img_stereo, '/home/luis/Desktop/img_stereo.png')
        # save_image(img_lidar_color, '/home/luis/Desktop/img_lidar_color.png')
        # save_image(im_color_lidar, '/home/luis/Desktop/im_color_lidar.png')
        # save_image(foot_print_objects_image, '/home/luis/Desktop/foot_print_objects_image.png')
        # save_image(img_global_plan_points, '/home/luis/Desktop/img_global_plan_points.png')


        mix_inputs = torch.cat([img_lidar_color, foot_print_objects_image, img_stereo, img_global_plan_points, img_global_plan_line, img_gps_backward])
        sample = {'image_inputs': mix_inputs}#

        return sample

if __name__ == '__main__':
	rospy.init_node("cnn_panner", anonymous=True)
	print ("[Create cnn planner running...")
	cnnplanner=CNNPlanner()
	rospy.spin()