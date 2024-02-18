#!/usr/bin/env python3


import rospkg
import rospy
from geometry_msgs.msg import PoseWithCovarianceStamped, PoseStamped#, TransformStamped, Transform, Vector3, Quaternion#, Twist
from nav_msgs.msg import Path as RosPath
from msgs_navigation.msg import Path as NavPath
from msgs_navigation.msg import TrajectoryPoint, GlobalPlan
from std_msgs.msg import Bool, Header# Float64, 
from msgs_action.msg import VehicleState#, SteeringAngle, Throttle, Brake
from shapely.geometry import LineString#, get_point

import numpy as np
import os

from path_optimizer import *
from scipy import interpolate

class Replanner(object):
	def __init__ (self):


		self.msg_path_p = RosPath()
		self.msg_path_p.header.frame_id='map'
		self.msg_path_p.poses = []

		self.path_msg = NavPath()
		self.path_msg.header.frame_id = 'map'
		self.path_msg.path = []

		# self.path_shapely=None

		self.old_pose=None
		self.last_time_run=0.

		self.speed=0

		self.cnn_path=None

		self.waypoints = np.array([])
		self.precision = 0.2
		self.window = 50
		self.index = 0


		self.time_last_path=0
		self.time_last_replanning=0

		self.path_ros_pub = rospy.Publisher('/carina/navigation/cnn_path_ros', RosPath, queue_size=1)#same than local_path_planing_node
		self.path_pub = rospy.Publisher("/carina/navigation/cnn_path", NavPath,queue_size=1)#same than local_path_planing_node
		

		self.pose_sub = rospy.Subscriber('/carina/localization/pose', PoseWithCovarianceStamped, self.pose_cb, queue_size=1)
		self.path_sub = rospy.Subscriber("/carina/navigation/cnn_path", NavPath, self.cnn_path_cb, queue_size=1)#same than local_path_planing_node
		self.state_sub = rospy.Subscriber('/carina/vehicle/state', VehicleState, self.vehicle_state_cb, queue_size=1)


		self.shutdown_sub = rospy.Subscriber('/carina/vehicle/shutdown', Bool, self.shutdown_cb, queue_size=1)


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


	def vehicle_state_cb(self,msg):
		self.speed = msg.drive.speed

	def cnn_path_cb(self, msg):
		self.cnn_path=msg
		self.publishing_replanning=False
		self.time_last_path=msg.header.stamp.to_sec()
		
	def pose_cb(self,msg):

		current_pose=msg
		stamp=msg.header.stamp

		if self.old_pose==None: #or global_plan==None:
			self.old_pose=current_pose
			return

		if self.cnn_path==None:
			return

		
		if self.speed > 0.5:
			self.last_time_run = msg.header.stamp.to_sec()


		time_stoped = msg.header.stamp.to_sec() - self.last_time_run 
		print(self.publishing_replanning,time_stoped)

		time_from_last_path = msg.header.stamp.to_sec() - self.time_last_path 
		time_from_last_replanning = msg.header.stamp.to_sec() - self.time_last_replanning


		if time_stoped > 100 and not(self.publishing_replanning) and time_from_last_path < 0.5 and time_from_last_replanning>10:
			self.publishing_replanning=True
			

			self.path_msg.path = []


			path=[]
			for p in self.cnn_path.path:
				path.append(p.point[:2])

			path_shapely = LineString(path)

			offset_path = path_shapely.parallel_offset(2., 'left', join_style=1)

			x, y = offset_path.xy

			print(x,y)

			msg_path = RosPath()
			msg_path.header.frame_id='map'
			msg_path.poses = []

			self.msg_path_p.poses = []

			for x,y in zip(x[1:-1],y[1:-1]):
				ps = PoseStamped()
				ps.header.stamp = rospy.Time().now()#stamp#
				ps.pose.position.x = x #- self.init_point_odom[0]
				ps.pose.position.y = y #- self.init_point_odom[1]

				msg_path.poses.append(ps)
			msg_path.header.stamp =  rospy.Time().now()#stamp#


			try:
				path_imp=self.improve_waypoints(msg_path)
			except Exception as e:
				self.first_frame=True
				print(e)
				return

			count = 0
			for w in path_imp:
				t = TrajectoryPoint()
				t.point = w
				t.point_number = count
				t.end_track = False
				count = count + 1
				self.path_msg.path.append(t)

				ps = PoseStamped()
				ps.header.stamp = stamp
				ps.pose.position.x = w[0] 
				ps.pose.position.y = w[1]
				ps.pose.position.z = current_pose.pose.pose.position.z
				self.msg_path_p.poses.append(ps)

			self.time_last_replanning=msg.header.stamp.to_sec()
			self.path_msg.path[-1].end_track = False

			self.msg_path_p.header.stamp =  stamp#rospy.Time().now()
			self.path_msg.header.stamp = rospy.Time().now()#stamp#
			self.path_pub.publish(self.path_msg)
			self.path_ros_pub.publish(self.msg_path_p)



	def shutdown_cb(self, msg):
		if msg.data:

			self.stamp=None
			self.steering_angle=None
			self.speed=None
			self.current_pose=None
			self.old_pose=None
			self.old_poses_list=None
			self.cvbridge = None
			self.rgb_image =None
			self.rgb_cam_info=None
			self.stereo_bev_image =None
			self.lidar_bev_image =None
			self.global_plan=None
			self.last_path_time=None
			self.max_steering = None
			self.next_index_goal_plan=None
			self.len_global_plan=None
			self.first_frame=None
			self.waypoints = None
			self.precision =None
			self.window = None
			self.index = None
			self.msg_path_p = None
			self.path_msg = None
			self.markerArray = None
			self.intersections_markerArray = None
			self.model = None
			self.data=None 
			self.output=None
			self.device = None

			self.tf2_buffer_blink2odom = None
			self.tf2_listener_blink2odom = None

			self.pose_sub = None
			self.state_sub = None
			self.image_bev_sub =None
			self.image_bev_stereo_sub = None
			self.global_plan_raw_sub = None
			self.shutdown_sub = None
			self.shutdown_using_map_sub = None

			self.first_point_achived = None	
			self.marker_route_pub = None
			self.path_ros_pub = None
			self.path_pub = None
			self.pub_intersections_obj = None
			self.pub_global_plan_img = None
			self.global_plan_line_img = None

			torch.cuda.empty_cache()
			print ("Bye!")
			rospy.signal_shutdown("finished route")




if __name__ == '__main__':
	rospy.init_node("replaner_node", anonymous=True)
	print ("[Create replanner running...")
	replanner=Replanner()
	rospy.spin()