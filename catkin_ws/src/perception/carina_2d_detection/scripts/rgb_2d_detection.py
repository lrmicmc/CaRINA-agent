#!/usr/bin/env python3
import numpy as np
import rospy

import ros_numpy
from std_msgs.msg import Bool

from visualization_msgs.msg import Marker
from visualization_msgs.msg import MarkerArray
from sensor_msgs.msg import PointCloud2, PointField

from geometry_msgs.msg import Pose
from geometry_msgs.msg import PoseArray

from msgs_perception.msg import ObstacleArray, Obstacle, StereoCloudImage

from inference import inference_detector, init_detector

import tf
import rospkg
import os
import time

from pathlib import Path
import random
import cv2
import numpy as np
import copy

import rospy
from msgs_perception.msg import BoundingBox, BoundingBoxArray
from std_msgs.msg import Empty, Bool
from cv_bridge import CvBridge, CvBridgeError

from sensor_msgs.msg import Image
import struct
from std_msgs.msg import Header
from sensor_msgs.msg import PointCloud2, PointField
from sensor_msgs import point_cloud2
from geometry_msgs.msg import PointStamped
from msgs_navigation.msg import SpeedConstraint

class RGB2Ddetection(object):

	def __init__ (self):
		self.tf_l = tf.TransformListener()
		self.result=None 
		self.model=None
		rospack = rospkg.RosPack()
		dir_pkg=rospack.get_path('carina_2d_detection')

		gpu_device = rospy.get_param("/gpu_device_stack_param")

		print('gpu_device stack ', gpu_device)

		# device='cuda:0'

		device=gpu_device

		self.checkpoint_file = os.path.join(dir_pkg,'checkpoints','epoch_217.pth')
		self.config_file = os.path.join(dir_pkg,'checkpoints','mask-rcnn_r50_fpn_amp-1x_coco_emergency.py') ####################  half precision

		self.odom_objs_tf_red_prev=[]
		self.odom_marker_tf_red_prev=[]

		print("loading model: ",self.config_file)

		# build the model from a config file and a checkpoint file
		self.model = init_detector(self.config_file, self.checkpoint_file, device=device)
		print("init detector ... ")

		self.img=None
		self.rgb=None
		self.pc_rgb=None
		self.classes  = {0:'car',
		        1:'pedestrian',
		        2:'traffic_light_red',
		        3:'traffic_light_yellow',
		        4:'traffic_light_green',
		        5:'stop',
		        6:'bicycle',
		        7:'emergency'}


		self.names = {'traffic_light_red':'traffic_light_red', 
		        'traffic_light_green': 'traffic_light_green',
		        'traffic_light_yellow':'traffic_light_yellow', 
		        'car': 'car',
		        'pedestrian': 'pedestrian', 
		        'bicycle':'bicycle', 
		        'stop':'stop',
		        'emergency':'emergency'}

		self.color = {'traffic_light_red':(0, 0, 255), 'traffic_light_green': (0,255,0),
		        'traffic_light_yellow':(0,255,255), 'car': (122,0,0), 'pedestrian':(255,0,191), 'stop':(255,122,100), 
		        'bicycle':(0,130, 100), 'emergency':(128,255,0)}

		self.cvbridge = CvBridge()
		img = None
		objects = None

		#subscriber
		self.elas_sub   = rospy.Subscriber("/carina/perception/stereo/point_cloud", PointCloud2, self.stereo_cloud_callback)
		self.image_topic = rospy.get_param('/obstacle_detection_node/image_topic', '/carina/sensor/stereo/left_image_rgb_elas')
		# self.cloud_plus_image = rospy.Subscriber("/carina/perception/stereo/cloud_plus_image", StereoCloudImage, self.stereo_cloud_image_callback)
		self.image_sub = rospy.Subscriber(self.image_topic, Image, self.rgb_image_cb, queue_size=1)
		self.shutdown_sub = rospy.Subscriber('/carina/vehicle/shutdown', Bool, self.shutdown_cb, queue_size=1)


		#publisher
		self.pub_obj = rospy.Publisher("/carina/perception/camera/signs_bb", BoundingBoxArray, queue_size=1)
		self.pub_img = rospy.Publisher('/carina/perception/camera/image_bb', Image, queue_size=1)
		self.odom_objs_tfsign_pub = rospy.Publisher("/carina/perception/stereo/traffic_sign_odom", ObstacleArray, queue_size = 1)
		self.odom_objs_tfsign_marker_pub = rospy.Publisher("/carina/perception/stereo/traffic_sign_odom_marker", MarkerArray, queue_size = 1)

		self.odom_objs_ob_pub = rospy.Publisher("/carina/perception/stereo/obstacles_odom", ObstacleArray, queue_size = 1)
		self.odom_objs_ob_marker_pub = rospy.Publisher("/carina/perception/stereo/obstacles_marker", MarkerArray, queue_size = 1)

		self.velo_objs_ob_pub = rospy.Publisher("/carina/perception/stereo/obstacles_velo", ObstacleArray, queue_size = 1)
		self.velo_objs_ob_marker_pub = rospy.Publisher("/carina/perception/stereo/obstacles_velo_marker", MarkerArray, queue_size = 1)

		self.odom_cloud_signs_pub = rospy.Publisher("/carina/perception/stereo/cloud_tf_signs_odom", PointCloud2, queue_size = 1)
		self.speed_constraint_pub = rospy.Publisher('/carina/control/speed_constraint', SpeedConstraint, queue_size=1)


	def rgb_image_cb(self,im):

		stamp=im.header.stamp
		# self.img = None
		try:
		    self.img = self.cvbridge.imgmsg_to_cv2(im, "bgr8")
		except CvBridgeError as e:
		    print (e)

	def stereo_cloud_image_callback(self,data):
		# print(data.point_cloud)
		# print(data.left_image)
		pass


	def stereo_cloud_callback(self,data):
		img =copy.deepcopy(self.img)
		self.rgb = img[...,::-1]

		self.pc = ros_numpy.numpify(data)
		pc =copy.deepcopy(self.pc)
		pc =ros_numpy.point_cloud2.split_rgb_field(pc)
		if pc is None:
			return

		stamp=data.header.stamp

		# test a single image
		# time_ini=time.time()
		self.result = inference_detector(self.model, self.rgb)
		# time_fin=time.time()
		# print(time_fin-time_ini)
		det=self.result.cpu().detach().numpy()#.tolist()

		if det is not None:# and len(det) > 0:
			self.process_detect(det.pred_instances.masks, det.pred_instances.bboxes, det.pred_instances.scores, det.pred_instances.labels, img, pc, stamp )
		else:
		    if self.pub_img.get_num_connections() > 0:
		        try:
		            self.pub_img.publish(self.cvbridge.cv2_to_imgmsg(img0, "bgr8"))
		        except CvBridgeError as e:
		            print (e)




	def process_detect(self, masks, bboxes, scores, labels, img_orig, point_cloud, stamp):

		pedestrian_presence=False
		bycicle_presence=False
		emergency_presence=False

		pc=point_cloud

		if pc is None:
			return	

		msg=BoundingBoxArray()
		bounding_array = []  

		odom_objs_tfsign = ObstacleArray()
		odom_objs_tfsign.obstacle = []
		odom_objs_tfsign_marker = MarkerArray()
		odom_objs_tfsign_marker.markers = []

		odom_objs_stereo = ObstacleArray()
		odom_objs_stereo.obstacle = []
		velo_objs_stereo = ObstacleArray()
		velo_objs_stereo.obstacle = []
		odom_objs_stereo_marker = MarkerArray()
		odom_objs_stereo_marker.markers = []
		velo_objs_stereo_marker = MarkerArray()
		velo_objs_stereo_marker.markers = []

		n_tfsigns = 0

		points_cloud=[]

		for mask, box, conf, cls  in zip(masks, bboxes, scores, labels):
			x0,x1,x2,x3 = box
			label = self.classes[int(cls)]

			if label == 'pedestrian':
				pedestrian_presence=True
			if label == 'bycicle':
				bycicle_presence=True
			if label == 'emergency':
				emergency_presence=True

			c_name = label
			prob_c = conf
			if x3 > (img_orig.shape[0]-20) and x1 > (img_orig.shape[0]-200):
				continue
			p1x= int(x0)
			p1y= int(x1)

			p2x= int(x0)
			p2y= int(x3)

			p3x= int(x2)
			p3y= int(x3)

			p4x= int(x2)
			p4y= int(x1)

			kernel_size = int(min(x3-x1,x2-x0)/4)

			kernel=np.ones((kernel_size, kernel_size), np.uint8)
			mask_erode=cv2.erode(mask.astype(np.uint8), kernel, iterations=1 )



			if self.pub_img.get_num_connections() != 0:
			    cv2.rectangle(img_orig, (p1x, p1y), (p3x,p3y), self.color[self.names[label]], 2)
			    cv2.putText(img_orig, label, (p1x - 2, p1y-2), cv2.FONT_HERSHEY_SIMPLEX, 0.5, self.color[self.names[label]], 1, cv2.LINE_AA)
			    # for mask_i in masks:
			    img_orig = self.overlay(img_orig, mask, color=self.color[self.names[label]][::-1], alpha=0.5)

			b = BoundingBox()

			b.p1.x= p1x
			b.p1.y= p1y

			b.p2.x= p2x
			b.p2.y= p2y

			b.p3.x= p3x
			b.p3.y= p3y

			b.p4.x= p4x
			b.p4.y= p4y
			b.classe.data = self.names[c_name]
			b.probability = float(prob_c)
			bounding_array.append(b)



			if ( (int(p3y)) - (int(p1y))) < 2 or ( (int(p3x)) - (int(p1x))) < 2:
				continue
			
			pc_roi = pc[mask_erode.astype(bool)]
			
			points=np.zeros((pc_roi.shape[0],3))
			points[:,0]=pc_roi['x']
			points[:,1]=pc_roi['y']
			points[:,2]=pc_roi['z']
			rgb=np.zeros((pc_roi.shape[0],3))
			rgb[:,0]=pc_roi['r']
			rgb[:,1]=pc_roi['g']
			rgb[:,2]=pc_roi['b']

			valid_points=[]
			for (xp, yp, zp), (r, g, b) in zip(points, rgb):
			        if (xp==0 and  yp==0 and  zp==0) or yp > 5 or yp < -5: #valid points only
			        	continue
			        x = xp
			        y = yp
			        z = zp
			        ptc = [x, y, z, 0]
			        pt= [x, y, z]

			        a = 255
			        rgb_point = struct.unpack('I', struct.pack('BBBB', int(b), int(g), int(r), a))[0]
			        ptc[3] = rgb_point
			        points_cloud.append(ptc)
			        valid_points.append(pt)

			if len(valid_points)==0:
				continue

			norm_points=np.linalg.norm(valid_points,axis=1)


			a_min_point=np.argmin(norm_points)

			obstacle_point=valid_points[a_min_point]


			pt=obstacle_point
	

			pc_point=PointStamped()
			pc_point.header.frame_id = "stereo"
			pc_point.header.stamp =rospy.Time(0)
			pc_point.point.x=pt[0]
			pc_point.point.y=pt[1]
			pc_point.point.z=pt[2]

			dist_to_obstacle=np.linalg.norm((pt[0],pt[2]))

			# try:
			# 	(trans_odom,rot_odom) = self.tf_l.lookupTransform('map', "stereo", rospy.Time(0))
			# except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
			# 	print('tf error transforming from stereo to map ')

			# try:
			# 	(trans_stereo2velodyne,rot_stereo2velodyne) = self.tf_l.lookupTransform('velodyne', "stereo", rospy.Time(0))
			# except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
			# 	print('tf error transforming from stereo to velodyne ')


			pt_odom = self.tf_l.transformPoint("map",pc_point)

			pt_velo = self.tf_l.transformPoint("velodyne",pc_point)

			color = self.color[self.names[label]]#classe[label]

			obj_odom_marker = Marker()
			obj_odom_marker.header.frame_id = "map"
			obj_odom_marker.header.stamp = stamp#rospy.Time.now()
			obj_odom_marker.type = obj_odom_marker.CUBE
			obj_odom_marker.action = obj_odom_marker.ADD
			obj_odom_marker.ns = "map_namespace";
			obj_odom_marker.pose.position = pt_odom.point
			obj_odom_marker.pose.orientation.x = 0.0
			obj_odom_marker.pose.orientation.y = 0.0
			obj_odom_marker.pose.orientation.z = 0.0
			obj_odom_marker.pose.orientation.w = 1.0
			obj_odom_marker.scale.x = 2.
			obj_odom_marker.scale.y = 2.
			obj_odom_marker.scale.z = 2.
			obj_odom_marker.color.b = color[0]/255
			obj_odom_marker.color.g = color[1]/255
			obj_odom_marker.color.r = color[2]/255
			obj_odom_marker.color.a = 1.
			obj_odom_marker.lifetime= rospy.Duration(0.5)
			obj_odom_marker.id = n_tfsigns

			obj_velo_marker = Marker()
			obj_velo_marker.header.frame_id = "velodyne"
			obj_velo_marker.header.stamp = stamp#rospy.Time.now()
			obj_velo_marker.type = obj_velo_marker.CUBE
			obj_velo_marker.action = obj_velo_marker.ADD
			obj_velo_marker.ns = "map_namespace";
			obj_velo_marker.pose.position = pt_velo.point
			obj_velo_marker.pose.orientation.x = 0.0
			obj_velo_marker.pose.orientation.y = 0.0
			obj_velo_marker.pose.orientation.z = 0.0
			obj_velo_marker.pose.orientation.w = 1.0
			obj_velo_marker.scale.x = 2.
			obj_velo_marker.scale.y = 2.
			obj_velo_marker.scale.z = 2.
			obj_velo_marker.color.b = color[0]/255
			obj_velo_marker.color.g = color[1]/255
			obj_velo_marker.color.r = color[2]/255
			obj_velo_marker.color.a = 1.
			obj_velo_marker.lifetime= rospy.Duration(0.5)
			obj_velo_marker.id = n_tfsigns

			obj_odom = Obstacle()
			obj_odom.header.frame_id = "map"
			obj_odom.header.stamp = stamp#rospy.Time.now()
			obj_odom.ns = "map_namespace";
			obj_odom.pose.position = pt_odom.point
			obj_odom.scale.x = 2.
			obj_odom.scale.y = 2.
			obj_odom.scale.z = 2.
			obj_odom.classes = [label]
			obj_odom.color.b = int(color[0]/255)
			obj_odom.color.g = int(color[1]/255)
			obj_odom.color.r = int(color[2]/255)
			obj_odom.color.a = 1.
			obj_odom.lifetime= rospy.Duration(0.5)
			obj_odom.track_status = 1
			obj_odom.type = -1
			obj_odom.id = n_tfsigns


			obj_velo = Obstacle()
			obj_velo.header.frame_id = "map"
			obj_velo.header.stamp = stamp#rospy.Time.now()
			obj_velo.ns = "map_namespace";
			obj_velo.pose.position = pt_velo.point
			obj_velo.scale.x = 2.
			obj_velo.scale.y = 2.
			obj_velo.scale.z = 2.
			obj_velo.classes = [label]
			obj_velo.color.b = int(color[0]/255)
			obj_velo.color.g = int(color[1]/255)
			obj_velo.color.r = int(color[2]/255)
			obj_velo.color.a = 1.
			obj_velo.lifetime= rospy.Duration(0.5)
			obj_velo.track_status = 1
			obj_velo.type = -1
			obj_velo.id = n_tfsigns


			n_tfsigns+=1

			
			if label=='traffic_light_red' or label=='traffic_light_green' or label=='traffic_light_yellow'or label=='stop':
				odom_objs_tfsign_marker.markers.append(obj_odom_marker)
				odom_objs_tfsign.obstacle.append(obj_odom)
			else:
				odom_objs_stereo_marker.markers.append(obj_odom_marker)
				velo_objs_stereo_marker.markers.append(obj_velo_marker)

				odom_objs_stereo.obstacle.append(obj_odom)
				velo_objs_stereo.obstacle.append(obj_velo)


		if (pedestrian_presence or emergency_presence or (bycicle_presence and (pt[0]>1.5))) and dist_to_obstacle < 25 :
			collision_constraint = SpeedConstraint()
			collision_constraint.header.stamp = rospy.Time().now()
			
			if dist_to_obstacle > 10:
				collision_constraint.speed = 0.8
			else:
				collision_constraint.speed = 0.2

			collision_constraint.reason = "\033[31m[From 2d_detection node] "+str(label)+"  presence: \033[0m" + " " +str(dist_to_obstacle) +" meters"
			self.speed_constraint_pub.publish(collision_constraint)


		if self.pub_obj.get_num_connections() != 0:
		    msg.objects = bounding_array
		    msg.size = len(bounding_array)
		    msg.header.stamp = stamp#rospy.Time.now()
		    msg.header.frame_id = 'stereo'
		    if len(bounding_array)>0:
		        self.pub_obj.publish(msg)

		if self.pub_img.get_num_connections() != 0:
		    try:
		        self.pub_img.publish(self.cvbridge.cv2_to_imgmsg(img_orig, "bgr8"))
		    except CvBridgeError as e:
		        print (e)

# ###################################  publish_obstacles_cloud  ###################################
		self.odom_objs_tfsign_pub.publish(odom_objs_tfsign )
		self.odom_objs_tfsign_marker_pub.publish(odom_objs_tfsign_marker )

		self.odom_objs_ob_pub.publish(odom_objs_stereo)
		self.velo_objs_ob_pub.publish(velo_objs_stereo)

		self.odom_objs_ob_marker_pub.publish(odom_objs_stereo_marker)
		self.velo_objs_ob_marker_pub.publish(velo_objs_stereo_marker)


		if self.odom_cloud_signs_pub.get_num_connections() != 0:
			fields = [PointField('x', 0, PointField.FLOAT32, 1),
			PointField('y', 4, PointField.FLOAT32, 1),
			PointField('z', 8, PointField.FLOAT32, 1),
			# PointField('rgb', 12, PointField.UINT32, 1)]
			PointField('rgba', 12, PointField.UINT32, 1)]

			header = Header()
			header.stamp = rospy.Time.now()
			header.frame_id = "stereo"
			pc2 = point_cloud2.create_cloud(header, fields, points_cloud, )
			self.odom_cloud_signs_pub.publish(pc2)


	def overlay(self, image, mask, color, alpha, resize=None):
	    """Combines image and its segmentation mask into a single image.
	    https://www.kaggle.com/code/purplejester/showing-samples-with-segmentation-mask-overlay

	    Params:
	        image: Training image. np.ndarray,
	        mask: Segmentation mask. np.ndarray,
	        color: Color for segmentation mask rendering.  tuple[int, int, int] = (255, 0, 0)
	        alpha: Segmentation mask's transparency. float = 0.5,
	        resize: If provided, both image and its mask are resized before blending them together.
	        tuple[int, int] = (1024, 1024))

	    Returns:
	        image_combined: The combined image. np.ndarray

	    """
	    color = color[::-1]
	    colored_mask = np.expand_dims(mask, 0).repeat(3, axis=0)
	    colored_mask = np.moveaxis(colored_mask, 0, -1)
	    masked = np.ma.MaskedArray(image, mask=colored_mask, fill_value=color)
	    image_overlay = masked.filled()

	    if resize is not None:
	        image = cv2.resize(image.transpose(1, 2, 0), resize)
	        image_overlay = cv2.resize(image_overlay.transpose(1, 2, 0), resize)

	    image_combined = cv2.addWeighted(image, 1 - alpha, image_overlay, alpha, 0)

	    return image_combined

	def shutdown_cb(self, msg):
		if msg.data:

			self.model=None
			self.checkpoint_file = None
			self.config_file = None
			self.result=None 
			self.img = None
			self.rgb = None
			self.classes = None  
			self.color = None 
			self.cvbridge = None 
			self.names  = None 

			#publisher
			del self.pub_obj
			del self.pub_img

			#subscriber
			del self.image_topic 
			del self.image_sub 
			del self.shutdown_sub
			# torch.cuda.empty_cache()
			# print('empty_cache')
			print ("Bye!")
			rospy.signal_shutdown("finished route")



if __name__ == '__main__':
	rospy.init_node("rgb_2d_detection", anonymous=True)
	print ("[2D detection pointcloud] running...")
	det=RGB2Ddetection()
	rospy.spin()



