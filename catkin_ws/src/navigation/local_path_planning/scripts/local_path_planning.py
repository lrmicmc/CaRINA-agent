#!/usr/bin/env python

#ros
import rospy
import tf

#python
import numpy as np
from scipy import interpolate
import matplotlib.pyplot as plt
import warnings
# warnings.filterwarnings("ignore")

#ros messages
from nav_msgs.msg import Path as RosPath
from msgs_navigation.msg import Path, TrajectoryPoint
from std_msgs.msg import Bool
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from msgs_action.msg import VehicleState

#utilities
from path_optimizer import *
from PathPlanning import generate_maneuver, GetClothoidPath
import os

class LocalPathPlanning(object):

	def __init__(self):

		# self.last_path_time=rospy.Time.now()#rospy.to_sec()

		self.current_pose = None
		self.kappa = 0.0
		self.L = 2.85

		self.waypoints = np.array([])

		# self.last_stamp = rospy.Time().now()

		# self.precision = 0.3
		self.window = 300

		self.index = 0
		self.i_path = 0
		self.path_segment=np.array([])

		self.msg_path = None
		# self.msg_trajectory = None

		# self.hHorizon = [30.0, 50.0]
		# self.vHorizon = [-6.0, 0.0, 6.0]

		# self.init_point_odom = [-1, -1]

		# self.last_samples = None
		self.old_pose=None

		# self.len_waypoints=0

		self.delta_time_sendPath = 4.
		self.delta_dist_sendPath = 0.2

		self.dist=100
		# self.first_point_achived=False
		self.first_point_achived=False

		# self.create_dataset=False#True

		self.firts_map=True

		# self.full_path_done=True

		# self.pub_ini_5=0
		self.speed = 0.

		self.last_time_run = 0.

		self.time_stoped = 100


		self.waypoints_sub = rospy.Subscriber('/carina/navigation/path', RosPath, self.path_cb, queue_size=1)
		self.shutdown_sub = rospy.Subscriber('/carina/vehicle/shutdown', Bool, self.shutdown_cb, queue_size=1)
		self.current_pose_sub = rospy.Subscriber('/carina/localization/pose', PoseWithCovarianceStamped, self.pose_cb, queue_size=1)
		self.vehicle_state_sub = rospy.Subscriber('/carina/vehicle/state', VehicleState, self.vehicle_state_cb, queue_size=1)


		self.first_point_achived_sub = rospy.Subscriber('/carina/map/first_point_achived', Bool, self.first_point_achived_cb,queue_size=1)		# self.radar_pub_front = rospy.Publisher('/carina/sensor/radar/front/obstacles_array', ObstacleArray, queue_size=1)

		self.shutdown_using_map = rospy.Publisher('/carina/map/shutdown_using_map', Bool, queue_size=1)		# self.radar_pub_front = rospy.Publisher('/carina/sensor/radar/front/obstacles_array', ObstacleArray, queue_size=1)
		self.path_pub = rospy.Publisher('/carina/navigation/path_segment', Path, queue_size=1, latch=True)
		self.path_ros_pub = rospy.Publisher('/carina/navigation/path_local_ros', RosPath, queue_size=1, latch=True)


		# self.last_time = rospy.Time().now()
		# rospy.Timer(rospy.Duration(0.1), self.sendPathRos)

	def first_point_achived_cb(self, msg):
		# print('first point achieved')
		self.first_point_achived=msg.data

	def pose_cb(self, msg):
		# print('pose received')
		self.current_pose = msg
  
		if self.old_pose is None:
			self.old_pose = self.current_pose 
   
   
		curr_pose = [self.current_pose.pose.pose.position.x,
               		 self.current_pose.pose.pose.position.y,
                  	 self.current_pose.pose.pose.position.z]
		curr_pose = np.array(curr_pose)
  
		past_pose = [self.old_pose.pose.pose.position.x,
               		 self.old_pose.pose.pose.position.y,
                  	 self.old_pose.pose.pose.position.z]
		past_pose = np.array(past_pose)

		self.dist = np.sqrt(np.power(curr_pose - past_pose, 2).sum())
		# print('dist ',dist)
		# elapsed_time = (self.current_pose.header.stamp - self.old_pose.header.stamp).to_sec() 
		# print('elapsed_time ' ,elapsed_time)
		print(self.dist, self.delta_dist_sendPath, self.time_stoped, self.delta_time_sendPath)

		if (self.dist > self.delta_dist_sendPath) or (self.time_stoped > self.delta_time_sendPath):
			
			if len(self.waypoints)>0:
				# print('sendPathRos')
				self.sendPathRos()
				self.old_pose = self.current_pose
				self.last_time_run = rospy.Time().now().to_sec()
   
	def vehicle_state_cb(self, msg):
		# print('vehicle state received')
		self.kappa = np.tan(msg.drive.steering_angle)/self.L

		self.speed = msg.drive.speed

		if self.speed > 0.5:
			self.last_time_run = rospy.Time().now().to_sec()#msg.header.stamp.to_sec()

		self.time_stoped = (rospy.Time().now().to_sec() - self.last_time_run)



	def path_cb(self, msg):
		# print('received path ',msg)
		if self.firts_map:
			self.index = 0
			# self.init_point_odom = [None, None]
			self.waypoints = np.array([])
			self.i_path = 0
			self.path_segment=np.array([])

			self.firts_map=False

		way = [[p.pose.position.x, p.pose.position.y, p.pose.position.z] for p in msg.poses]
		way = np.asarray(way)

		x_new = way[:, 0]
		y_new = way[:, 1]


		# stamp = rospy.Time().now()
		
		while(self.index < len(x_new)):
			
			# max_index = self.index

			if self.index + self.window > len(x_new):
				max_index = len(x_new)
			else:
				max_index = self.index + self.window

			ways = generate_trajectory(np.array([x_new[self.index:max_index], y_new[self.index:max_index]]).T)
			self.index=max_index

			if len(self.waypoints) == 0:
				self.waypoints =  ways
			else:
				# ways[:, 4] = ways + self.waypoints
				self.waypoints  = np.concatenate((self.waypoints , ways), axis=0)
			
			# time.sleep(1)

		# print('self.waypoints ', self.waypoints)
		# self.full_path_done=True
		# print ("Done!")

	

	# def sendPathRos(self, event):
	def sendPathRos(self):
		if(self.current_pose is not None):
	   
			path = self.get_segment_path(200)
			# print('path ',path)
			if path is None:
				return
			if len(path) == 0:
				return

			# to = rospy.Time().now()
			path_msg = Path()
			
			path_msg.header.frame_id = 'map'
			path_msg.path = []
			count = 0
			for w in path:
				t = TrajectoryPoint()
				t.point = w 
				t.point_number = count
				t.end_track = False
				count = count + 1
				path_msg.path.append(t)
 
			path_msg.path[-1].end_track = True

			path_msg.header.stamp = rospy.Time().now()

			self.msg_path = RosPath()
			self.msg_path.header.frame_id='map'
			self.msg_path.poses = []
			
			# if (self.init_point_odom[0] is None) and (self.init_point_odom[1] is None):
			# 	self.init_point_odom[0] = path[0, 0]
			# 	self.init_point_odom[1] = path[0, 1]

			# dist_path_pose=1000
			for p in path:
				ps = PoseStamped()
				ps.header.stamp = rospy.Time().now()
				ps.pose.position.x = p[0]# - self.init_point_odom[0]
				ps.pose.position.y = p[1]# - self.init_point_odom[1]
				ps.pose.position.z = self.current_pose.pose.pose.position.z#[1]# - self.init_point_odom[1]

				self.msg_path.poses.append(ps)

				# dist_path_pose_temp=np.linalg.norm(np.array([ps.pose.position.x, ps.pose.position.y]) -\
    #     										   np.array([self.current_pose.pose.pose.position.x, self.current_pose.pose.pose.position.y]))

				# dist_path_pose=min(dist_path_pose_temp,dist_path_pose)

			self.msg_path.header.stamp =  rospy.Time().now()

			# print('pub path')
   
			self.path_pub.publish(path_msg)
			self.path_ros_pub.publish(self.msg_path)

			signal_shutdown = Bool()
			signal_shutdown.data = True
			self.shutdown_using_map.publish(signal_shutdown)


			# to1=(rospy.Time().now() - to).to_sec()

			# print 'TT:', (rospy.Time().now() - ti).to_sec(), ti1, th1, to1

	def get_segment_path(self, size=50):
		# print('len(self.waypoints',len(self.waypoints))
		if (len(self.waypoints) > 0) and (self.current_pose is not None ):
			
			if self.i_path >= len(self.waypoints):
				print('if1')
				return np.array([])

			if len(self.path_segment) == 0:
				print('if2')
				i_min = self.i_path
				i_max = min(self.i_path + size, len(self.waypoints))
				self.i_path = i_max
				
				self.path_segment = self.waypoints[i_min:i_max, :]
				# return np.array([])
				return self.path_segment
			else:
				print('else')
				pose = np.array([self.current_pose.pose.pose.position.x, self.current_pose.pose.pose.position.y])
			
				dist2path = np.sqrt(np.power(pose - self.path_segment[:, 0:2], 2).sum(axis=1))
				i_min = dist2path.argmin()
				# print('i_min ',i_min)
				crossed_points = len(self.path_segment) - i_min
				append_points = size  - crossed_points
				
				if append_points > 0:
					print('if append_points')
					spath = self.waypoints[self.i_path:self.i_path+append_points, :]
					self.i_path = self.i_path + append_points
					self.path_segment = np.concatenate((self.path_segment[i_min:,:], spath), axis=0)
					# print('self.path_segment ', self.path_segment )

					if i_min==size-1:
						return np.array([])
					else: 
						return self.path_segment



	# def compute_path(self):
	# 	if (len(self.waypoints) > 0) and (self.current_pose is not None):
	# 		pose = np.array([self.current_pose.pose.pose.position.x, self.current_pose.pose.pose.position.y])
			
	# 		# Orientation is represented in quaternion
	# 		q = self.current_pose.pose.pose.orientation

	# 		# Get yaw
	# 		ori = (q.x,
	# 				q.y, 
	# 				q.z, 
	# 				q.w)
	# 		_, _, yaw = tf.transformations.euler_from_quaternion(ori)

			
	# 		#Find closest point on the lane
	# 		dist2path = np.sqrt(np.power(pose - self.waypoints[:, 0:2], 2).sum(axis=1))
	# 		i_min = dist2path.argmin()
	# 		pose_lane = self.waypoints[i_min, :]

	# 		#Vector point to sample points at s_final
	# 		s_lane = pose_lane[4]
	# 		# print (pose_lane)
	# 		hHori_final = self.hHorizon[-1] + s_lane
	# 		i_final_point = np.abs(self.waypoints[i_min:,4] - hHori_final).argmin() + i_min

	# 		vec_final = np.array([np.cos(self.waypoints[i_final_point, 3]-np.pi/2),
	# 					np.sin(self.waypoints[i_final_point, 3]-np.pi/2)])
	# 		theta_final = self.waypoints[i_final_point, 3]
	# 		x_hor_final = self.waypoints[i_final_point, 0]
	# 		y_hor_final = self.waypoints[i_final_point, 1]


	# 		samples = []

	# 		try:
	# 			#Sample trajectories
	# 			for hHor in self.hHorizon:

	# 				s_hor = s_lane + hHor
	# 				i_point = np.abs(self.waypoints[i_min:,4] - s_hor).argmin() + i_min

	# 				#Vector point to sample points at s_horizon
	# 				vec =  np.array([np.cos(self.waypoints[i_point, 3]-np.pi/2),
	# 							np.sin(self.waypoints[i_point, 3]- np.pi/2)])
	# 				theta_sample = self.waypoints[i_point, 3]

	# 				for vHor in self.vHorizon:

	# 					x_hor = self.waypoints[i_point, 0]
	# 					y_hor = self.waypoints[i_point, 1]

	# 					x_sample = vHor*vec[0] + x_hor
	# 					y_sample = vHor*vec[1] + y_hor

	# 					# print i_point, x_sample, y_sample

	# 					#print i_point, x_hor, y_hor, x_sample, y_sample, vHor, vec, self.waypoints.shape, i_min, s_hor, s_lane

	# 					#Consider the curvature at the sample point
	# 					r = 1/(abs(self.waypoints[i_point, 2]) + 1e-8)
	# 					if vHor < 0:
	# 						kappa_sample = self.waypoints[i_point, 2] - 1/r
	# 					else:
	# 						kappa_sample = self.waypoints[i_point, 2] + 1/r

	# 					# plt.plot(pose[0], pose[1], 'r*')
	# 					# plt.plot(x_sample, y_sample, 'g*')
	# 					# plt.pause(0.001)
	# 					#print yaw, self.kappa, theta_sample, kappa_sample

	# 					# print (pose[0], pose[1], pose_lane[3], pose_lane[2], x_sample, y_sample, theta_sample, kappa_sample)
	# 					init, sharp = generate_maneuver(pose[0], pose[1], pose_lane[3], pose_lane[2],
	# 												 x_sample, y_sample, theta_sample, kappa_sample)


	# 					if hHor != self.hHorizon[-1]:
	# 						x_final = vHor*vec_final[0] + x_hor_final
	# 						y_final = vHor*vec_final[1] + y_hor_final

	# 						#Consider the curvature at the sample point
	# 						r = 1/(abs(self.waypoints[i_final_point, 2]) + 1e-8)
	# 						if vHor < 0:
	# 							kappa_final = self.waypoints[i_final_point, 2] - 1/r
	# 						else:
	# 							kappa_final = self.waypoints[i_final_point, 2] + 1/r

	# 						init1, sharp1 = generate_maneuver(x_sample, y_sample, theta_sample, kappa_sample,
	# 														x_final, y_final, theta_final, kappa_final)

	# 						init = np.concatenate((init, init1))
	# 						sharp = np.concatenate((sharp, sharp1))

	# 					result = {}
	# 					result['init'] = init
	# 					result['sharp'] = sharp
	# 					samples.append(result)
	# 		# except Exception, e:
	# 		except Exception as e: 
	# 			print ("ERROR!!")
	# 			samples = []
	# 		# 	print e
	# 		# 	samples = self.last_samples

	# 		return samples

	# def norm_ang(self, theta):
	# 		if theta > np.pi:
	# 			theta_n = theta - 2*np.pi
	# 		elif theta < -np.pi:
	# 			theta_n = 2*np.pi + theta
	# 		else:
	# 			theta_n = theta
	# 		return theta_n












	def shutdown_cb(self, msg):
		if msg.data:
			# self.last_path_time= None
			self.current_pose = None
			self.kappa = None
			self.L = None
			self.waypoints = None
			# self.last_stamp = None
			# self.precision = None
			self.window = None
			self.index = None
			self.i_path = None
			self.path_segment=None
			self.msg_path = None
			# self.msg_trajectory = None
			# self.hHorizon = None
			# self.vHorizon = None
			# self.init_point_odom = None
			# self.last_samples = None
			self.old_pose=None
			# self.len_waypoints= None
			self.delta_time_sendPath = None
			self.delta_dist_sendPath = None
			self.dist= None
			self.first_point_achived= None
			# self.create_dataset= None
			self.firts_map= None
			# self.full_path_done= None
			# self.pub_ini_5= None

			del self.waypoints_sub
			del self.shutdown_sub 
			del self.current_pose_sub 
			del self.vehicle_state_sub 
			del self.first_point_achived_sub 	
			del self.shutdown_using_map 
			del self.path_pub 
			del self.path_ros_pub 

			print ("Bye!")
			rospy.signal_shutdown("finished route")















