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


class EmergencyVehicleDetection(object):

	def __init__ (self):
		self.result=None 
		self.model=None
		rospack = rospkg.RosPack()
		dir_pkg=rospack.get_path('carina_2d_detection')
		# gpu_device = rospy.get_param("/gpu_device_stack_param")
		# print('gpu_device stack ', gpu_device)
		# device=gpu_device
		device='cuda:0'

		self.checkpoint_file = os.path.join(dir_pkg,'checkpoints','epoch_217.pth')
		self.config_file = os.path.join(dir_pkg,'checkpoints','mask-rcnn_r50_fpn_amp-1x_coco_emergency.py') ####################  half precision

		print("loading model: ",self.config_file)
		# build the model from a config file and a checkpoint file
		self.model = init_detector(self.config_file, self.checkpoint_file, device=device)
		print("init detector ... ")


		self.emergency_back=False
		self.img=None
		self.rgb=None

		self.classes  = {0:'car',
		        1:'pedestrian',
		        2:'traffic_light_red',
		        3:'traffic_light_yellow',
		        4:'traffic_light_green',
		        5:'stop',
		        6:'bicycle',
		        7:'emergency'}

		self.color = {'traffic_light_red':(0, 0, 255), 'traffic_light_green': (0,255,0),
		        'traffic_light_yellow':(0,255,255), 'car': (122,0,0), 'pedestrian':(255,0,191), 'stop':(255,122,100), 
		        'bicycle':(0,130, 100), 'emergency':(128,255,0)}

		self.cvbridge = CvBridge()
		#subscriber
		self.image_sub = rospy.Subscriber('/carina/sensor/camera/back/image_rect_color', Image, self.rgb_image_cb, queue_size=1)
		self.shutdown_sub = rospy.Subscriber('/carina/vehicle/shutdown', Bool, self.shutdown_cb, queue_size=1)

		#publisher
		self.pub_img = rospy.Publisher('/carina/perception/camera/back/image_inst_back', Image, queue_size=1)
		self.pub_emergency_back = rospy.Publisher('/carina/perception/camera/back/emergency_vehicle_back', Bool, queue_size=1)


	def rgb_image_cb(self,im):
		self.emergency_back=False
		stamp=im.header.stamp
		try:
		    self.img = self.cvbridge.imgmsg_to_cv2(im, "bgr8")
		except CvBridgeError as e:
		    print (e)

		img =copy.deepcopy(self.img )
		self.rgb = img[...,::-1]
		# time_ini=time.time()
		self.result = inference_detector(self.model, self.rgb)
		# time_fin=time.time()
		# print(time_fin-time_ini)
		det=self.result.cpu().detach().numpy()#.tolist()

		if det is not None:# and len(det) > 0:
			self.process_detect(det.pred_instances.masks, det.pred_instances.bboxes, det.pred_instances.scores, 
				det.pred_instances.labels, self.img, stamp )
		else:
		    if self.pub_img.get_num_connections() > 0:
		        try:
		            self.pub_img.publish(self.cvbridge.cv2_to_imgmsg(img0, "bgr8"))
		        except CvBridgeError as e:
		            print (e)


	def process_detect(self, masks, bboxes, scores, labels, img_orig, stamp):
		warning_area_mask=np.zeros((img_orig.shape[0], img_orig.shape[1]),dtype=np.uint8)
		n_tfsigns = 0
		h=img_orig.shape[0]
		w=img_orig.shape[1]
		# cv2.rectangle(img_orig, ( int(w/2)-20, int(2*h/3)+3),  ( int(w/2)+70, h ), (0,0,255), 2)
		# cv2.rectangle(img_orig, ( int(w/2)-100, int(2*h/3)),  ( int(w/2)+200, h ), (255,0,0), 2)
		p1=[int(w/2)-20,  int(2*h/3)]
		p2=[int(w/2)+50,  int(2*h/3)]
		p3=[int(w/2)+160, h]
		p4=[int(w/2)-100, h]

		pts = np.array([p1,p2,p3,p4], np.int32)
		pts = pts.reshape((-1,1,2))

		warning_area_mask=cv2.fillPoly(warning_area_mask,[pts],(255),8)

		color_w_area=(0,255,0)

		for mask, box, conf, cls  in zip(masks, bboxes, scores, labels):
			x0,x1,x2,x3 = box
			label = self.classes[int(cls)]
			c_name = label
			prob_c = conf

			p1x= int(x0)
			p1y= int(x1)

			p2x= int(x0)
			p2y= int(x3)

			p3x= int(x2)
			p3y= int(x3)

			p4x= int(x2)
			p4y= int(x1)

			bitwiseAnd = cv2.bitwise_and(warning_area_mask, np.uint8(mask))

			sum_pixels=np.sum(bitwiseAnd[:,:])

			if sum_pixels>0 and label==	'emergency' and conf>0.8:
				color_w_area=(255,0,0)
				self.emergency_back=True

			if self.pub_img.get_num_connections() != 0:
			    # cv2.rectangle(img_orig, (0, 10), (10, 10), self.color[self.names[label]], 2)
			    cv2.putText(img_orig, label, (p1x - 2, p1y-2), cv2.FONT_HERSHEY_SIMPLEX, 0.5, self.color[label], 1, cv2.LINE_AA)
			    img_orig = self.overlay(img_orig, mask, color=self.color[label][::-1], alpha=0.5)

		if self.pub_img.get_num_connections() != 0:
		    img_orig = self.overlay(img_orig, warning_area_mask, color=color_w_area, alpha=0.35)
		    try:
		        self.pub_img.publish(self.cvbridge.cv2_to_imgmsg(img_orig, "bgr8"))
		    except CvBridgeError as e:
		        print (e)
		self.pub_emergency_back.publish(self.emergency_back)



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
			self.emergency_back=None
			self.img = None
			self.rgb = None
			self.classes = None  
			self.color = None 
			self.cvbridge = None 

			#publisher
			del self.pub_img
			del self.pub_emergency_back
			#subscriber
			del self.image_sub 
			del self.shutdown_sub
			# torch.cuda.empty_cache()
			# print('empty_cache')
			print ("Bye!")
			rospy.signal_shutdown("finished route")


if __name__ == '__main__':
	rospy.init_node("emergency_detection_node", anonymous=True)
	print ("[emergency_detection_node] running...")
	det=EmergencyVehicleDetection()
	rospy.spin()



