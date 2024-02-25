#!/usr/bin/env python3
# YOLOv5 🚀 by Ultralytics, GPL-3.0 license
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
# from msgs_perception.msg import BoundingBox, BoundingBoxArray
from std_msgs.msg import Empty, Bool
from cv_bridge import CvBridge, CvBridgeError
from utils import *
from utils.torch_utils import *
from models import *
import rospkg
import torch
from sensor_msgs.msg import Image

import glob
import json
import cv2
from mmdet.apis import inference_detector, init_detector
import time
import numpy as np
from skimage import measure                        # (pip install scikit-image)
from shapely.geometry import Polygon, MultiPolygon # (pip install Shapely)
from tqdm import tqdm

@torch.no_grad()


class TrafficLightDetection(object):
    def __init__ (self):
        print ("[Camera Detection Node] running...")
        # rospack = rospkg.RosPack()

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

        # with open(rospack.get_path('yolov5')+'/scripts/'+'objects.txt','r') as f:
        with open('objects.txt','r') as f:
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

        weights= 'runs/train/exp21/weights/best.pt'
        # imgsz=1408  # inference size (pixels)
        self.conf_thres=0.35  # confidence threshold
        self.iou_thres=0.45  # NMS IOU threshold
        self.max_det=1000  # maximum detections per image
        # device='0'  # cuda device, i.e. 0 or 0,1,2,3 or cpu
        self.agnostic_nms=False  # class-agnostic NMS
        self.augment=False  # augmented inference
        self.visualize=False  # visualize features
        self.half=False  # use FP16 half-precision inference
        self.pred=None
        self.img=None

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

        #publisher
        # self.pub_obj = rospy.Publisher("/carina/perception/camera/signs_bb", BoundingBoxArray, queue_size=1)
        # self.pub_img = rospy.Publisher('/carina/perception/camera/image_bb', Image, queue_size=1)

        # #subscriber
        # self.image_topic = rospy.get_param('/obstacle_detection_node/image_topic', '/carina/sensor/stereo/left_image_rgb_elas')
        # self.image_sub = rospy.Subscriber(self.image_topic, Image, self.imageCallback, queue_size=1)
        # self.shutdown_sub = rospy.Subscriber('/carina/vehicle/shutdown', Bool, self.shutdown_cb, queue_size=1)


    # def imageCallback(self,im):
        # Initialize
        # stamp=im.header.stamp





        img0 = None
        self.annotations=[]
        self.images=[]

        jpg_carla_files=glob.glob('dataset_carla_tracking/training/images/rgb/*.jpg')
        # print(jpg_carla_files)
        i=0
        for jpg in tqdm(jpg_carla_files):
            # i+=1

            filename=jpg.split('/')[-1].split('.')[0]
            # print(filename)

            image_id=int(filename)#ann['image_id']

            file=jpg#'/mnt/Data1/dataset_carla_tracking/training/images/rgb/'+str(ann['image_id'])+'.jpg'
            # print(file)
            try:
                img0=cv2.imread(file)
                img=img0

                # img0=img[:, :, [2, 1, 0]]
            except Exception as e:
                print(file)
                print(e)
                continue

            height_img, width_img = img.shape[:2]

            image={
                    "id":image_id,
                    "width":width_img,
                    "height":height_img,
                    "file_name":str(image_id)+'.jpg'}

            self.images.append(image)






            # try:
            #     img0 = self.cvbridge.imgmsg_to_cv2(im, "bgr8")
            # except CvBridgeError as e:
            #     print (e)

            self.img = letterbox(img, self.imgsz, stride=self.stride, auto=self.pt)[0]
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
                self.process_detect(det,img0,image_id)
            # else:
            #     if self.pub_img.get_num_connections() > 0:
            #         try:
            #             self.pub_img.publish(self.cvbridge.cv2_to_imgmsg(img0, "bgr8"))
            #         except CvBridgeError as e:
            #             print (e)
        self.save_json(self.images, self.annotations, filename)

    def process_detect(self, objects,img,image_id):
        # global list_obj, pub_obj, pub_img, color, names#, prob_cut#, stamp
        # msg=BoundingBoxArray()
        # bounding_array = []  
        # print('process')
        for x0,x1,x2,x3, conf, cls  in objects:
            # label = self.classes[int(cls)]
            label = int(cls)

            # if (label in  self.list_obj):
            c_name = label
            prob_c = conf
            # if x3 > (img_orig.shape[0]/4):
            p1x= int(x0)
            p1y= int(x1)
        
            p2x= int(x0)
            p2y= int(x3)
        
            p3x= int(x2)
            p3y= int(x3)
        
            p4x= int(x2)
            p4y= int(x1)

            # if label==6:#stop
            #     category_id=34

            # print(label)

            if label==0:#red
                category_id=31

            elif label==2:# yellow 
                category_id=32

            elif label==1:#  green traffic light
                category_id=33
            else:
                continue

            label_dic={0:31,
                       1:33,
                       2:32,
                       6:34,
                       11:34}


            ann_dict = {
              "id": len(self.annotations),
              "segmentation": [[p1x, p1y,  p2x, p2y, p3x, p3y,  p4x, p4y, p1x, p1y,]],
              "image_id": image_id,
              "category_id": label_dic[int(label)],
              "bbox": [p1x, p1y, p3x - p1x, p3y - p1y],
              "area": (p3x - p1x) * (p3y - p1y),
              "iscrowd": 0
            }
            self.annotations.append(ann_dict)


            cv2.rectangle(img, (p1x, p1y), (p3x,p3y), (255,255,255), 2)

   


            # is_crowd=0

            # annotation_id = len(self.annotations)#id_ann
            # new_ann_inf['id']= annotation_id#bicycle on carla sensor instances

            # new_ann_inf = create_sub_mask_annotation(mask, image_id, category_id, annotation_id, is_crowd)

            # if category_id!=10:
            # else:
            #   new_ann_inf=None
            # if new_ann_inf is None:
            #     print('none ann')
            #     continue
            # self.annotations.append(new_ann_inf)
            # print(self.annotations)

        # cv2.imwrite(str(image_id)+'img.png',img)
            # cv2.waitKey(100)
                    # if self.pub_img.get_num_connections() != 0:

                    #     cv2.rectangle(img_orig, (p1x, p1y), (p3x,p3y), self.color[self.names[label]], 2)
                    #     cv2.putText(img_orig, label, (p1x - 2, p1y-2), cv2.FONT_HERSHEY_SIMPLEX, 0.8, self.color[self.names[label]], 1, cv2.LINE_AA)

                    # b = BoundingBox()

                    # b.p1.x= p1x
                    # b.p1.y= p1y

                    # b.p2.x= p2x
                    # b.p2.y= p2y

                    # b.p3.x= p3x
                    # b.p3.y= p3y

                    # b.p4.x= p4x
                    # b.p4.y= p4y
                    # b.classe.data = self.names[c_name]
                    # b.probability = float(prob_c)
                    # bounding_array.append(b)

        # if self.pub_obj.get_num_connections() != 0:
        #     msg.objects = bounding_array
        #     msg.size = len(bounding_array)
        #     # msg.header.stamp = stamp#rospy.Time.now()
        #     msg.header.frame_id = 'stereo'
        #     if len(bounding_array)>0:
        #         self.pub_obj.publish(msg)
        # if self.pub_img.get_num_connections() != 0:
        #     try:
        #         self.pub_img.publish(self.cvbridge.cv2_to_imgmsg(img_orig, "bgr8"))
        #     except CvBridgeError as e:
        #         print (e)

    


    def create_sub_mask_annotation(self, sub_mask, image_id, category_id, annotation_id, is_crowd):
        # Find contours (boundary lines) around each sub-mask
        # Note: there could be multiple contours if the object
        # is partially occluded. (E.g. an elephant behind a tree)
        # print(sub_mask.shape)

        contours = measure.find_contours(sub_mask, 0.5, positive_orientation='low')
        # print(contours)

        segmentations = []
        polygons = []
        for contour in contours:
            if contour is None:
                continue
            # Flip from (row, col) representation to (x, y)
            # and subtract the padding pixel
            for i in range(len(contour)):
                row, col = contour[i]
                contour[i] = (col - 1, row - 1)

            # Make a polygon and simplify it
            poly = Polygon(contour)
            poly = poly.simplify(1.0, preserve_topology=False)

            if poly.geom_type == 'MultiPolygon':
                continue
            if poly.is_empty:
                # print('poligon empty')
                continue
            polygons.append(poly)
            segmentation = np.array(poly.exterior.coords).ravel().tolist()
            segmentations.append(segmentation)

        # Combine the polygons to calculate the bounding box and area
        multi_poly = MultiPolygon(polygons)
        # print(multi_poly)
        if multi_poly.is_empty:
            return None

        x, y, max_x, max_y = multi_poly.bounds
        width = max_x - x
        height = max_y - y
        bbox = (x, y, width, height)
        area = multi_poly.area
        if area < 10 or width<2 or height<2:
            return None
        annotation = {
            'segmentation': segmentations,
            'iscrowd': is_crowd,
            'image_id': image_id,
            'category_id': category_id,
            'id': annotation_id,
            'bbox': bbox,
            'area': area
        }

        return annotation



    def save_json(self, images, annotations, file_out):
        categories=[]
        categorie_Vehicles={"id":10,"name":"Vehicles","supercategory":"","metadata":{},"keypoint_colors":[]}
        categories.append(categorie_Vehicles)

        categorie_Pedestrian={"id":4,"name":"Pedestrian","supercategory":"","metadata":{},"keypoint_colors":[]}
        categories.append(categorie_Pedestrian)

        categorie_TrafficSign={"id":12,"name":"TrafficSign","supercategory":"","metadata":{},"keypoint_colors":[]}
        categories.append(categorie_TrafficSign)

        categorie_TrafficLight={"id":18,"name":"TrafficLight","supercategory":"","metadata":{},"keypoint_colors":[]}
        categories.append(categorie_TrafficLight)

        categorie_RedTrafficLight={"id":31,"name":"RedTrafficLight","supercategory":"","metadata":{},"keypoint_colors":[]}
        categories.append(categorie_RedTrafficLight)

        categorie_YellowTrafficLight={"id":32,"name":"YellowTrafficLight","supercategory":"","metadata":{},"keypoint_colors":[]}
        categories.append(categorie_YellowTrafficLight)

        categorie_GreenTrafficLight={"id":33,"name":"GreenTrafficLight","supercategory":"","metadata":{},"keypoint_colors":[]}
        categories.append(categorie_GreenTrafficLight)

        categorie_stop={"id":34,"name":"stop","supercategory":"","metadata":{},"keypoint_colors":[]}
        categories.append(categorie_stop)

        categorie_bicycle={"id":35,"name":"bicycle","supercategory":"","metadata":{},"keypoint_colors":[]}
        categories.append(categorie_bicycle)


        # {"id":1,"name":"whale","supercategory":"","color":"#5af882","metadata":{},"keypoint_colors":[]}
        sting_dataset={'images':images,'annotations': annotations,'categories': categories}
        with open(str(file_out)+'inference_yolo.json', 'w') as outfile:
            json.dump(sting_dataset, outfile)


if __name__=='__main__':
    # rospy.init_node("traffic_light_node", anonymous=True)
    # print ("[Camera Signs Detection Node] ready to go!")
    tfld=TrafficLightDetection()
    # rospy.spin()