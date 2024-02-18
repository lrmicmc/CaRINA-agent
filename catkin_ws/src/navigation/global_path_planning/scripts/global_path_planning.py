#!/usr/bin/env python

#ros
import rospy

#python
import numpy as np
from scipy import interpolate
import matplotlib.pyplot as plt
from threading import Thread

import warnings
warnings.filterwarnings("ignore")

#ros messages
from nav_msgs.msg import Path 
from std_msgs.msg import Bool
from geometry_msgs.msg import PoseStamped
from msgs_mapping.msg import HDMap as HDMapMsg

#utilities
# from path_optimizer import *

from topological_graph import TopologicalMap
# from path_optimizer import generate_trajectory

from joblib import Parallel, delayed
import time
import os

class GlobalPathPlanning(object):

	def __init__(self):

		self.waypoints = np.array([])
		self.route = np.array([])
		self.path = np.array([])
		self.last_stamp = rospy.Time().now()
		self.precision = 0.1
		self.index = 0
		# self.msg_path = None
		self.ros_msg = None
		self.path_msg = None
		# self.msg_ros = None
		self.msg_hd_map = None
		self.path_received=False
		self.thread_paths = None
		self.topological_map = TopologicalMap(verbose=False)

		self.hd_map_sub = rospy.Subscriber('/carina/map/hdmap', HDMapMsg, self.hd_map_cb, queue_size=1)
		self.waypoints_sub = rospy.Subscriber('/carina/navigation/waypoints', Path, self.waypoints_cb, queue_size=1)
		self.shutdown_sub = rospy.Subscriber('/carina/vehicle/shutdown', Bool, self.shutdown_cb, queue_size=1)
		self.path_pub = rospy.Publisher('/carina/navigation/path', Path, queue_size=1, latch=True)
		self.path_ros_pub = rospy.Publisher('/carina/navigation/path_global_ros', Path, queue_size=1, latch=True)


		# rospy.Timer(rospy.Duration(1.), self.sendPathRos)

	def __convert_coordinates(self, waypoints:np.ndarray, dir:int):
		waypoints=np.column_stack((waypoints[:,1],waypoints[:,0]))
		waypoints[:,dir] = -waypoints[:,dir]
		return waypoints

	def hd_map_cb(self, msg):
		print("Received HD Map")
		if msg.XML_HDMap == "":
			return
		self.msg_hd_map = msg
		self.topological_map.set_map_content(str(msg.XML_HDMap))
		# print("F HD Map")
  
	def waypoints_cb(self, msg):
		if msg.poses==[]:
			return
		# print('waypoints cb',msg)
		# print(msg.header.stamp, self.last_stamp)
     
		# if msg.header.stamp > self.last_stamp:
		if not self.path_received:
			self.last_stamp = msg.header.stamp
			self.path_received=True
		else:
			return
		# print("Reveived Waypoints")

		# print ("[Global Path Planning] Received a new route! (stamp.secs:" +str(self.last_stamp) + ")")

		self.waypoints= [[p.pose.position.x, p.pose.position.y, p.pose.position.z] for p in msg.poses]
		self.waypoints = np.asarray(self.waypoints)

		#if self.leaderboard==1:
		self.waypoints = self.__convert_coordinates(self.waypoints, 0)

  
		self.thread_paths = Thread(target=self.process_waypoints)
		self.thread_paths.start()
		
	def process_waypoints(self):
     
		start_time = time.time()
		while(not self.topological_map.is_active()):
			# print("[Process Waypoints] waiting topological map receive the hd-map....")
			time.sleep(0.5)
			if (time.time() - start_time) > 130.0: #timeout 30 seconds
				raise RuntimeError("Timeout: could not process the waypoints since the hd-map is not active!")
       
		# print("[Process Waypoints] initial waypoints...")
		initial_size = max(int(len(self.waypoints)*0.1), 4)#5% of waypoints
		# print(initial_size,self.waypoints.shape)

		_r = self.topological_map.get_route_from_waypoints(
							waypoints=self.waypoints[:initial_size], 
							look_ahead=1, 
							build_graph=True
					)
		# print(_r)
		_p = self.topological_map.get_path_from_nodes_list(
            				nodes=_r
                	)
  
		start_time = time.time()  
		_msg_path, _msg_odom = self.__build_path_messages(_p)
		self.path_pub.publish(_msg_path)
		self.path_ros_pub.publish(_msg_odom)
  
		# print("[Process Waypoints] processing waypoints...")
		start_time = time.time()  
		self.route = self.topological_map.get_route_from_waypoints(
							waypoints=self.waypoints, 
							look_ahead=1, 
							build_graph=True
					)
		start_time = time.time()  
		# print("[Process Waypoints] getting path from route...")
		self.path = self.topological_map.get_path_from_nodes_list(
            				nodes=self.route
                	)
		start_time = time.time()
  
		# print("[Process Waypoints] publishing messages...")
		self.path_msg, self.ros_msg = self.__build_path_messages(self.path)

		self.path_pub.publish(self.path_msg)
		self.path_ros_pub.publish(self.ros_msg)
  
		# print("Done!")
  
  
	def __build_path_messages(self, path):
		# print(path.shape)
		# if self.leaderboard==1:
		path = self.__convert_coordinates(path, 1)
		msg_path = Path()
		stamp = rospy.Time().now()
		msg_path.header.stamp = stamp
		msg_path.header.frame_id = 'map'
		msg_path.poses = []
  
		for w in path:
			t = PoseStamped()
			t.header.frame_id = 'map'
			t.header.stamp = stamp
			t.pose.position.x = w[0]
			t.pose.position.y = w[1]
			t.pose.position.z = 0.0
			msg_path.poses.append(t)
	
		ros_path = Path()
		ros_path.header.stamp = stamp
		ros_path.header.frame_id = 'odom'
		ros_path.poses = []
		for p in path:
			ps = PoseStamped()
			ps.header.frame_id = 'odom'
			ps.header.stamp = stamp
			ps.pose.position.x = p[0] - path[0, 0]
			ps.pose.position.y = p[1] - path[0, 1]
			ps.pose.position.z = 0.0
			ros_path.poses.append(ps)
   
		return msg_path, ros_path


	def shutdown_cb(self, msg):
		if msg.data:

			self.waypoints = None
			self.route = None
			self.path = None
			self.last_stamp = None
			self.precision = None
			self.index = None
			self.ros_msg = None
			self.path_msg = None
			self.msg_hd_map = None
			self.path_received=None
			self.thread_paths = None
			self.topological_map = None
			del self.waypoints_sub
			del self.shutdown_sub 
			del self.path_pub 
			del self.path_ros_pub 
			del self.hd_map_sub 
			print ("Bye!")
			rospy.signal_shutdown("finished route")
		self.thread_paths = None

	# def sendPathRos(self, event):
     
	# 	# if self.msg_path is not None:
	# 	if self.ros_msg is not None:
	# 		self.path_ros_pub.publish(self.ros_msg)

	# 	# if self.msg_trajectory is not None:
	# 	if self.path_msg is not None:
	# 		self.path_pub.publish(self.path_msg)

