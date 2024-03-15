#!/usr/bin/env python3
# YOLOv5 🚀 by Ultralytics, GPL-3.0 license


import ros_numpy
from std_msgs.msg import Bool

from visualization_msgs.msg import Marker
from visualization_msgs.msg import MarkerArray
from sensor_msgs.msg import PointCloud2, PointField

from geometry_msgs.msg import Pose
from geometry_msgs.msg import PoseArray

from msgs_perception.msg import ObstacleArray, Obstacle, StereoCloudImage
from geometry_msgs.msg import PoseWithCovarianceStamped

import tf

from pathlib import Path
import random
import cv2
import numpy as np
import torch
from models.experimental import attempt_load
from utils.general import apply_classifier, check_img_size, check_suffix, non_max_suppression, scale_coords
from utils.torch_utils import select_device, time_sync
from utils.augmentations import letterbox
import rospy
from msgs_perception.msg import BoundingBox, BoundingBoxArray
from std_msgs.msg import Empty, Bool
from cv_bridge import CvBridge, CvBridgeError
from utils import *
from utils.torch_utils import *
from models import *
import rospkg
import torch
from sensor_msgs.msg import Image

import copy
import time

import struct
from std_msgs.msg import Header
from sensor_msgs.msg import PointCloud2, PointField
from sensor_msgs import point_cloud2
from geometry_msgs.msg import PointStamped
from msgs_navigation.msg import SpeedConstraint
@torch.no_grad()


class TrafficLightDetection(object):
    def __init__ (self):

        self.tf_l = tf.TransformListener()

        print ("[Camera Detection Node] running...")
        rospack = rospkg.RosPack()

        self.classes  = {0:'traffic_light_red', 
                1:'traffic_light_green',
                2:'traffic_light_yellow', 
                3:'30_1km',
                4:'60_1km', 
                5:'90_1km', 
                6:'stop_1', 
                7:'30_2km', 
                8:'60_2km', 
                9:'90_2km', 
                10:'stop_2', 
                11:'40_2km'}

        self.list_obj=None

        with open(rospack.get_path('yolov5')+'/scripts/'+'objects.txt','r') as f:
             self.list_obj=f.read()
        # self.list_obj_frames = []
        self.cvbridge = CvBridge()
        img = None
        objects = None

        self.color = {'traffic_light_red':(0, 0, 255), 'traffic_light_green': (0,255,0),
                'traffic_light_yellow':(0,255,255), '30km': (122,0,0),'60km': (255,0,0), '90km':(255,0,191), 'stop':(255,122,100), '40km':(0,130, 100)}

        self.names = {'traffic_light_red':'traffic_light_red', 
                'traffic_light_green': 'traffic_light_green',
                'traffic_light_yellow':'traffic_light_yellow', 
                '30_1km': '30km',
                '60_1km': '60km', 
                '90_1km':'90km', 
                'stop_1':'stop', 
                '30_2km':'30km', 
                '60_2km':'60km', 
                '90_2km':'90km', 
                'stop_2':'stop', 
                '40_2km':'30km'}
        # speed_names={"30":"30km", "60":"60km", "90":"90km", "40":"40km" }
        # short_names = {'traffic_light_red':'r', 'traffic_light_green': 'g',
        #         'traffic_light_yellow':'y', '30_1km': '30','60_1km': '60', 
        #         '90_1km':'90', 'stop_1':'stop', '30_2km':'30', '60_2km':'60', 
        #         '90_2km':'90', 'stop_2':'stop', '40_2km':'40'}

        weights=rospack.get_path('yolov5')+'/scripts/runs/train/exp21/weights/best.pt'
        # imgsz=1408  # inference size (pixels)
        self.conf_thres=0.45  # confidence threshold
        self.iou_thres=0.45  # NMS IOU threshold
        self.max_det=1000  # maximum detections per image
        # device='0'  # cuda device, i.e. 0 or 0,1,2,3 or cpu
        self.agnostic_nms=False  # class-agnostic NMS
        self.augment=False  # augmented inference
        self.visualize=False  # visualize features
        self.half=False  # use FP16 half-precision inference
        self.pred=None
        self.img=None

        self.pc=None

        self.current_pose=None

        self.device = select_device('0')

        # Load model
        self.model=None
        w = weights[0] if isinstance(weights, list) else weights
        classify, suffix, suffixes = False, Path(w).suffix.lower(), ['.pt', '.onnx', '.tflite', '.pb', '']
        check_suffix(w, suffixes)  # check weights have acceptable suffix
        self.pt=None
        self.onnx=None
        self.pt, self.onnx, tflite, pb, saved_model = (suffix == x for x in suffixes)  # backend booleans
        self.stride, names = 64, [f'class{i}' for i in range(1000)]  # assign defaults
        if self.pt:
            self.model = attempt_load(weights, map_location=self.device)  # load FP32 model
            self.stride = int( self.model.stride.max())  # model stride
            names =  self.model.module.names if hasattr( self.model, 'module') else  self.model.names  # get class names
            if self.half:
                 self.model.half()  # to FP16

        self.imgsz=[1408,1408]
        self.imgsz = check_img_size(self.imgsz, s=self.stride)  # check image size

        # bs = 1  # batch_size
        # Run inference
        if self.pt and self.device.type != 'cpu':
             self.model(torch.zeros(1, 3, *self.imgsz).to(self.device).type_as(next( self.model.parameters())))  # run once


        #subscriber
        self.image_topic = rospy.get_param('/obstacle_detection_node/image_topic', '/carina/sensor/stereo/left_image_rgb_elas')
        self.image_sub = rospy.Subscriber(self.image_topic, Image, self.imageCallback, queue_size=1)
        self.shutdown_sub = rospy.Subscriber('/carina/vehicle/shutdown', Bool, self.shutdown_cb, queue_size=1)

        self.elas_sub   = rospy.Subscriber("/carina/perception/stereo/point_cloud", PointCloud2, self.stereo_cloud_callback)
        self.current_pose_sub = rospy.Subscriber('/carina/localization/pose', PoseWithCovarianceStamped, self.pose_cb, queue_size=1)



        #publisher
        self.pub_obj = rospy.Publisher("/carina/perception/camera/signs_bb_yolo", BoundingBoxArray, queue_size=1)
        self.pub_img = rospy.Publisher('/carina/perception/camera/image_bb_yolo', Image, queue_size=1)

        self.odom_objs_tfsign_pub = rospy.Publisher("/carina/perception/stereo/traffic_sign_odom_yolo", ObstacleArray, queue_size = 1)
        self.odom_objs_tfsign_marker_pub = rospy.Publisher("/carina/perception/stereo/traffic_sign_odom_marker_yolo", MarkerArray, queue_size = 1)

        self.odom_objs_ob_pub = rospy.Publisher("/carina/perception/stereo/obstacles_odom_yolo", ObstacleArray, queue_size = 1)
        self.odom_objs_ob_marker_pub = rospy.Publisher("/carina/perception/stereo/obstacles_marker_yolo", MarkerArray, queue_size = 1)

        self.velo_objs_ob_pub = rospy.Publisher("/carina/perception/stereo/obstacles_velo_yolo", ObstacleArray, queue_size = 1)
        self.velo_objs_ob_marker_pub = rospy.Publisher("/carina/perception/stereo/obstacles_velo_marker_yolo", MarkerArray, queue_size = 1)

        self.odom_cloud_signs_pub = rospy.Publisher("/carina/perception/stereo/cloud_tf_signs_odom_yolo", PointCloud2, queue_size = 1)
        self.speed_constraint_pub = rospy.Publisher('/carina/control/speed_constraint', SpeedConstraint, queue_size=1)

    def pose_cb(self, msg):
        self.current_pose = msg

    def stereo_cloud_callback(self,data):
        # img =copy.deepcopy(self.img)
        # self.rgb = img[...,::-1]

        pc_d = ros_numpy.numpify(data)
        pc =copy.deepcopy(pc_d)
        self.pc =ros_numpy.point_cloud2.split_rgb_field(pc)


    def imageCallback(self,im):
        start_time = time.time()
        # Initialize
        if self.pc is None or self.current_pose is None:
            return
        stamp=im.header.stamp
        img0 = None
        try:
            img0 = self.cvbridge.imgmsg_to_cv2(im, "bgr8")
        except CvBridgeError as e:
            print (e)

        self.img = letterbox(img0, self.imgsz, stride=self.stride, auto=self.pt)[0]
        self.img = self.img.transpose((2, 0, 1))[::-1]  # HWC to CHW, BGR to RGB
        self.img = np.ascontiguousarray(self.img)

        dt, seen = [0.0, 0.0, 0.0], 0
        # for img in [img]:

        t1 = time_sync()
        if self.onnx:
            self.img = self.img.astype('float32')
        else:
            self.img = torch.from_numpy(self.img).to(self.device)
            self.img = self.img.half() if self.half else self.img.float()  # uint8 to fp16/32
        self.img = self.img / 255.0  # 0 - 255 to 0.0 - 1.0
        if len(self.img.shape) == 3:
            self.img = self.img[None]  # expand for batch dim
        t2 = time_sync()
        dt[0] += t2 - t1

        # Inference
        if self.pt:
            self.pred =  self.model(self.img, augment=self.augment, visualize=self.visualize)[0]
        t3 = time_sync()
        dt[1] += t3 - t2

        # NMS
        classes=None
        self.pred = non_max_suppression(self.pred, self.conf_thres, self.iou_thres, classes, self.agnostic_nms, max_det=self.max_det)[0]
        det=self.pred.cpu().detach().numpy()#.tolist()
        dt[2] += time_sync() - t3
        if det is not None and len(det) > 0:
            det[:, :4] = scale_coords(self.img.shape[2:], det[:, :4], img0.shape).round()
            self.process_detect(det,img0,stamp,self.pc)
        else:
            if self.pub_img.get_num_connections() > 0:
                try:
                    self.pub_img.publish(self.cvbridge.cv2_to_imgmsg(img0, "bgr8"))
                except CvBridgeError as e:
                    print (e)
        print("--- %s seconds ---" % (time.time() - start_time))

    def process_detect(self, objects,img_orig,stamp,point_cloud):
        global list_obj, pub_obj, pub_img, color, names#, prob_cut#, stamp

        pc=point_cloud



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


        for x0,x1,x2,x3, conf, cls  in objects:
            label = self.classes[int(cls)]

            if (label in  self.list_obj):
                c_name = label
                prob_c = conf
                if x3 > (img_orig.shape[0]/4):

                    mask=np.zeros((img_orig.shape[0],img_orig.shape[1]))


                    p1x= int(x0)
                    p1y= int(x1)
                
                    p2x= int(x0)
                    p2y= int(x3)
                
                    p3x= int(x2)
                    p3y= int(x3)
                
                    p4x= int(x2)
                    p4y= int(x1)

                    cv2.rectangle(mask, (p1x, p1y), (p3x,p3y), 255, -1)



                    # kernel_size = int(min(x3-x1,x2-x0)/4)

                    # kernel=np.ones((kernel_size, kernel_size), np.uint8)
                    # mask_erode=cv2.erode(mask.astype(np.uint8), kernel, iterations=1 )

                    if self.pub_img.get_num_connections() != 0:

                        cv2.rectangle(img_orig, (p1x, p1y), (p3x,p3y), self.color[self.names[label]], 2)
                        cv2.putText(img_orig, label, (p1x - 2, p1y-2), cv2.FONT_HERSHEY_SIMPLEX, 0.8, self.color[self.names[label]], 1, cv2.LINE_AA)
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
                    
                    pc_roi = pc[mask.astype(bool)]
                    
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
                    #   (trans_odom,rot_odom) = self.tf_l.lookupTransform('map', "stereo", rospy.Time(0))
                    # except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                    #   print('tf error transforming from stereo to map ')

                    # try:
                    #   (trans_stereo2velodyne,rot_stereo2velodyne) = self.tf_l.lookupTransform('velodyne', "stereo", rospy.Time(0))
                    # except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                    #   print('tf error transforming from stereo to velodyne ')


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


        # if (pedestrian_presence or emergency_presence or (bycicle_presence and (pt[0]>1.5))) and dist_to_obstacle < 25 :
        #     collision_constraint = SpeedConstraint()
        #     collision_constraint.header.stamp = rospy.Time().now()
            
        #     if dist_to_obstacle > 10:
        #         collision_constraint.speed = 0.8
        #     else:
        #         collision_constraint.speed = 0.2

        #     collision_constraint.reason = "\033[31m[From 2d_detection node] "+str(label)+"  presence: \033[0m" + " " +str(dist_to_obstacle) +" meters"
        #     self.speed_constraint_pub.publish(collision_constraint)


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


    def shutdown_cb(self,msg):

        if msg.data:
            self.classes=None
            self.list_obj=None
            self.cvbridge=None

            self.color=None
            self.names=None
            self.conf_thres=None
            self.iou_thres=None
            self.max_det=None
            self.agnostic_nms=None
            self.augment=None
            self.visualize=None
            self.half=None
            self.pred=None
            self.img=None
            self.stride=None
            self.imgsz=None
            self.imgsz=None
            self.model=None
            #publisher
            del self.pub_obj
            del self.pub_img
            #subscriber
            del self.image_topic
            del self.image_sub
            del self.shutdown_sub

            print ("Bye!")
            rospy.signal_shutdown("finished route")
      


if __name__=='__main__':
    rospy.init_node("traffic_light_node", anonymous=True)
    print ("[Camera Signs Detection Node] ready to go!")
    tfld=TrafficLightDetection()
    rospy.spin()