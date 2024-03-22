#!/usr/bin/env python3
import numpy as np
import rospy
import sys
from geometry_msgs.msg import PoseWithCovarianceStamped#, Twist, PoseStamped 
from msgs_action.msg import VehicleState
from msgs_perception.msg import Obstacle, ObstacleArray
from std_msgs.msg import Float64, Bool
from msgs_navigation.msg import Path
from nav_msgs.msg import Odometry

# from nav_msgs.msg import Path
from msgs_traffic.msg import TrafficSign, TrafficSignArray
from msgs_navigation.msg import EmergencyStop
from std_msgs.msg import String
import time

class FSM(object):

	def __init__ (self):
		self.speed = 0.0
		self.current_pose = None
		self.current_odom = None
		self.last_time_odom=0
		self.obstacle_ahead = {'dist': 1000, 'speed': -1.0, 'time':None}
		self.traffic_light_ahead = {'dist': 1000, 'color': -1.0, 'time':None}
		speed_rate=1.0

		self.speed_max_param=rospy.get_param("/navigation/control/max_speed")#parameters["cameras"]["stereo"]["baseline"]#0.24

		# self.speed_limit_ahead = {'dist': 1000, 'current_speed_limit': 11.11*speed_rate, 'next_speed_limit': 11.11*speed_rate}
		# self.speed_limit_ahead = {'dist': 1000, 'current_speed_limit': 9.1*speed_rate, 'next_speed_limit': 9.1*speed_rate}
		self.speed_limit_ahead = {'dist': 1000, 'current_speed_limit': self.speed_max_param*speed_rate, 'next_speed_limit': self.speed_max_param*speed_rate}
		# self.speed_limit_ahead = {'dist': 1000, 'current_speed_limit': 0.155*speed_rate, 'next_speed_limit': 0.155*speed_rate}

		self.stop_sign_ahead = {'dist': 1000, 'stamp': None, 'time': None, 'pose': None}
		self.speed_ref = 0.
		self.flag_pub_vel = True
		self.path_received = False
		self.emergency_stop_flag=False
		self.emergency_stop_pub=None
		self.collision = False
		self.frames_ignore_stop=0

		self.names_state = {0: 'STATE_TRAFFIC_LIGHT', 
		               1: 'STATE_FOLLOWING_LEADER', 
		               2: 'STATE_FOLLOWING_LANE',
		               3: 'STATE_DECELERATE',
		               4: 'STATE_IDLE_STOP'}
		self.STATE_TRAFFIC_LIGHT = 0
		self.STATE_FOLLOWING_LEADER = 1
		self.STATE_FOLLOWING_LANE = 2
		self.STATE_DECELERATE = 3
		self.STATE_IDLE_STOP = 4
		self.state = self.STATE_FOLLOWING_LANE

		self.time_decelerate=0.
		self.time_stopped=0.
		self.obstacle_ahead['time'] = rospy.Time().now()

		self.pose_sub = rospy.Subscriber('/carina/localization/pose', PoseWithCovarianceStamped, self.pose_cb, queue_size=1)
		self.vis_odom_sub = rospy.Subscriber('/localization/odometry/visual', Odometry, self.visual_odom_cb, queue_size=1)
		self.odom_sub = rospy.Subscriber('/carina/localization/odom', Odometry, self.odom_cb, queue_size=1)

		self.state_sub = rospy.Subscriber('/carina/vehicle/state', VehicleState, self.vehicle_state_cb, queue_size=1)
		self.obst_sub = rospy.Subscriber('/carina/perception/lidar/nearest_obstacle_ahead', ObstacleArray, self.obstacle_cb, queue_size=1)
		self.obst_tl_sub = rospy.Subscriber('/carina/perception/lidar/obstacles_array_tfl', ObstacleArray, self.obstacle_tl_red_cb, queue_size=1)
		self.path_sub = rospy.Subscriber("/carina/navigation/path_segment", Path, self.path_callback, queue_size=1)
		self.cnn_path_sub = rospy.Subscriber("/carina/navigation/cnn_path", Path, self.cnn_path_callback, queue_size=1)
		self.shutdown_sub = rospy.Subscriber('/carina/vehicle/shutdown', Bool, self.shutdown_cb, queue_size=1)
		self.traffic_sign_sub = rospy.Subscriber('/carina/perception/traffic_rules/traffic_signs', TrafficSignArray, self.traffic_signs_cb, queue_size=1)
		self.risk_of_collision = rospy.Subscriber('/carina/sensor/risk_of_collision', Bool, self.collision_cb, queue_size=1)
		# if not('vel_pub' in locals()) or not('vel_pub' in globals()) or vel_pub is None:
		self.vel_pub = rospy.Publisher('/carina/control/vel_ref', Float64, queue_size=1)
		self.emergency_stop_pub = rospy.Publisher('/carina/navigation/emergency_stop', EmergencyStop, queue_size=1)
		# if not('vel_pub' in locals()) or not('vel_pub' in globals()) or self.vel_pub is None:
		# self.vel_pub = rospy.Publisher('/carina/control/vel_ref', Float64, queue_size=1)
		self.fsm_state = rospy.Publisher('/carina/navigation/fsm_state', String, queue_size=1)



	def vehicle_state_cb(self, msg):
		# print('vehicle state cb')
		start_time=time.time()

		self.speed = msg.drive.speed


		####################################### STATE MACHINE #################################
		dist_threshold = 4
		if (self.stop_sign_ahead['time'] is not None) and \
		   (msg.header.stamp - self.stop_sign_ahead['time']).to_sec() > .5:
			self.stop_sign_ahead['dist'] = 1000

		dist_to_stop_sign = self.stop_sign_ahead['dist'] if self.stop_sign_ahead['dist'] > 2. else 1000

		if (self.obstacle_ahead['time'] is not None) and \
		   (msg.header.stamp - self.obstacle_ahead['time']).to_sec() > .5:
			self.obstacle_ahead['dist'] = 1000

		dist_to_obstacle = self.obstacle_ahead['dist'] if self.obstacle_ahead['dist'] > 1. else 1000
		
		if (self.traffic_light_ahead['time'] is not None) and \
		   (msg.header.stamp - self.traffic_light_ahead['time']).to_sec() > .5:

			self.traffic_light_ahead['dist'] = 1000

		dist_to_traffic_light = self.traffic_light_ahead['dist'] if self.traffic_light_ahead['dist'] > (dist_threshold-1.) else 1000

		self.stop_sign_ahead['dist'] = dist_to_stop_sign
		self.traffic_light_ahead['dist'] = dist_to_traffic_light
		self.obstacle_ahead['dist'] = dist_to_obstacle


		delta_t = 1#0.1
		dist_to_obstacle_threshold = 40.

		time_to_last_odom = (msg.header.stamp.to_sec() -self.last_time_odom)
		# print(time_to_last_odom)


		if self.path_received and msg.header.stamp.to_sec() > 1.  and time_to_last_odom < 0.3: # wait 1. sec to start the mission and verify that the odom is published
			#No vehicle in front and no traffic light in front
			if self.state == self.STATE_FOLLOWING_LANE:
				if self.frames_ignore_stop>0:
					self.frames_ignore_stop=self.frames_ignore_stop-1

				self.speed_ref = self.speed_limit_ahead['current_speed_limit']
				distances = [dist_to_stop_sign, dist_to_traffic_light, dist_to_obstacle]
				min_dist_object = np.argmin(distances)
				if distances[min_dist_object] != 1000:
					if min_dist_object == 0 and self.frames_ignore_stop==0:
							self.time_decelerate= msg.header.stamp#rospy.Time().now()
							self.state = self.STATE_DECELERATE
					# elif min_dist_object == 1:
					# 	state = self.STATE_TRAFFIC_LIGHT
					elif dist_to_obstacle < dist_to_obstacle_threshold or dist_to_traffic_light < dist_to_obstacle_threshold:
							self.state = self.STATE_FOLLOWING_LEADER
							if dist_to_traffic_light < dist_to_obstacle:
								self.state = self.STATE_TRAFFIC_LIGHT

			#No traffic light in front, but there is a front vehicle
			elif self.state == self.STATE_FOLLOWING_LEADER or self.state == self.STATE_TRAFFIC_LIGHT:

				if self.frames_ignore_stop>0:
					self.frames_ignore_stop=self.frames_ignore_stop-1

				self.speed_ref = self.speed_limit_ahead['current_speed_limit']
				distances = [dist_to_stop_sign, dist_to_traffic_light, dist_to_obstacle]
				min_dist_object = np.argmin(distances)
				if distances[min_dist_object] != 1000:
					if min_dist_object == 0 and self.frames_ignore_stop==0:
							self.time_decelerate= msg.header.stamp#rospy.Time().now()
							self.state = self.STATE_DECELERATE

				max_break = 3.0
				time_to_collision_ob = dist_to_obstacle/(self.speed + 1e-6)
				time_to_collision_tfl = dist_to_traffic_light/(self.speed + 1e-6)
				time_to_collision=min(time_to_collision_ob,time_to_collision_tfl)
				time_threshold = 5
				time_threshold_emergency = 2.51#3
				if time_to_collision < time_threshold_emergency:
					self.speed_ref = 0.2#0
				elif time_to_collision < time_threshold:
					self.speed_ref = max(0.5, self.speed - max_break*delta_t)
				else: 
					self.speed_ref = self.speed_limit_ahead['current_speed_limit']
				# if dist_to_obstacle < 7:

				# if 7<=dist_to_traffic_light<15:
				# 	print('disto traffic light',dist_to_traffic_light)
				# 	speed_ref = 1.5
				if dist_to_traffic_light < 3:
					# print('disto traffic light',dist_to_traffic_light)
					self.speed_ref = 0


				self.state = self.STATE_FOLLOWING_LEADER

				if dist_to_obstacle < dist_to_obstacle_threshold or dist_to_traffic_light < dist_to_obstacle_threshold:
						self.state = self.STATE_FOLLOWING_LEADER
						if dist_to_traffic_light < dist_to_obstacle:
							self.state = self.STATE_TRAFFIC_LIGHT

				if dist_to_obstacle > dist_to_obstacle_threshold and dist_to_traffic_light > dist_to_obstacle_threshold:
					self.state = self.STATE_FOLLOWING_LANE
				# print(' \033[0m | Time_to_collision_ob: {:.4f}'.format(time_to_collision_ob))

			#Decrease speed unil completly stopped
			elif self.state == self.STATE_DECELERATE:
				time_to_be_decelerated=5
				max_break = 3.5
				# print(dist_to_stop_sign)
				if dist_to_stop_sign < 5:
					self.speed_ref = 0.
					self.time_stopped= msg.header.stamp#rospy.Time().now()
					self.state = self.STATE_IDLE_STOP
				else:
					self.speed_ref = max(2, self.speed - max_break*delta_t)

				if dist_to_stop_sign > 15 or (msg.header.stamp - self.time_decelerate).to_sec() > time_to_be_decelerated:
						self.state = self.STATE_FOLLOWING_LANE
					
			#Stay stopped for a predefined time
			elif self.state == self.STATE_IDLE_STOP:
				time_to_be_stopped = 2.
				# if (rospy.Time().now() - time_stopped).to_sec() > time_to_be_stopped:
				if (msg.header.stamp - self.time_stopped).to_sec() > time_to_be_stopped:
					self.state = self.STATE_FOLLOWING_LANE
					self.frames_ignore_stop=100
				else:
					self.speed_ref = 0.

			if self.collision:
				self.speed_ref = 0

			if self.speed_ref==0:
					em = EmergencyStop()
					em.header.stamp = msg.header.stamp#rospy.Time().now()
					em.header.frame_id ='map'
					em.stop_now = True
					# em.obstacle = obst
					self.emergency_stop_pub.publish(em)
					self.emergency_stop_flag = True
			else:
					em = EmergencyStop()
					em.header.stamp = msg.header.stamp#rospy.Time().now()
					em.header.frame_id ='map'
					em.stop_now = False
					self.emergency_stop_pub.publish(em)
					self.emergency_stop_flag = False

			# print('curr_state: \033[94m{}\033[0m | speed: {:.2f}'.format(self.names_state[self.state], self.speed_ref))
			# print('\t| dist2obst:{:.2f} | dist_light:{:.2f} | dist_stop:{:.2f}'.format(self.obstacle_ahead['dist'] ,self.traffic_light_ahead['dist'], self.stop_sign_ahead['dist']))
			# print '\033[94m' + str(speed_ref) +'\033[0m'
			
			# self.fsm_state.publish(self.names_state[self.state])
			# self.pub_speed_ref()


		else:
			self.speed_ref = 0.
			em = EmergencyStop()
			em.header.stamp = msg.header.stamp#rospy.Time().now()
			em.header.frame_id ='map'
			em.stop_now = True
			self.emergency_stop_pub.publish(em)
			self.emergency_stop_flag = True

			
		self.fsm_state.publish(self.names_state[self.state])
		self.pub_speed_ref()
		# print("--- %s seconds ---" % (time.time() - start_time))



	def visual_odom_cb(self, msg):
		# print('visual odom state cb')
		# self.current_odom = msg
		# print(msg.pose.covariance[0])
		if msg.pose.covariance[0]<100:
			self.last_time_odom=msg.header.stamp.to_sec()

	def odom_cb(self, msg):
		# print('odom state cb')
		# self.current_odom = msg
		self.last_time_odom=msg.header.stamp.to_sec()

	def pose_cb(self, msg):
		# print('pose state cb')
		self.current_pose = msg

	def collision_cb(self, msg):
		# print('collision state cb')
		self.collision = msg.data

	def obstacle_cb(self, msg):
		# print('pbstacle state cb')
		if self.current_pose is None:
			return

		if len(msg.obstacle)>0:
			obst = msg.obstacle[0]
			pose = np.array([self.current_pose.pose.pose.position.x, self.current_pose.pose.pose.position.y])
			pose_ob = np.array([obst.pose.position.x, obst.pose.position.y])
			dist = np.sqrt(np.power(pose - pose_ob, 2).sum())
			vel = np.sqrt(np.power([obst.twist.linear.x, obst.twist.linear.y, obst.twist.linear.z], 2).sum())
			self.obstacle_ahead['dist'] = dist
			self.obstacle_ahead['speed'] = vel
			self.obstacle_ahead['time'] = obst.header.stamp#rospy.Time().now()
		else:
			self.obstacle_ahead['dist'] = 1000
			self.obstacle_ahead['speed'] = 100
			self.obstacle_ahead['time'] = rospy.Time().now()
		# print ('\033[91m [Obstacle] \033[0m', self.obstacle_ahead)

	def obstacle_tl_red_cb(self, msg):
		# print('obstacle state cb')
		if self.current_pose is None:
			return

		if len(msg.obstacle)==0:
			return

		obst = msg.obstacle[0]

		pose = np.array([self.current_pose.pose.pose.position.x, self.current_pose.pose.pose.position.y])
		pose_ob = np.array([obst.pose.position.x, obst.pose.position.y])
		dist = np.sqrt(np.power(pose - pose_ob, 2).sum())

		vel = np.sqrt(np.power([obst.twist.linear.x, obst.twist.linear.y, obst.twist.linear.z], 2).sum())

		self.traffic_light_ahead['dist'] = dist
		self.traffic_light_ahead['speed'] = vel
		self.traffic_light_ahead['time'] = obst.header.stamp#rospy.Time().now()

		# print ('\033[91m [Red tfl Obstacle] \033[0m', self.traffic_light_ahead)

	def path_callback(self, msg):
		self.path_received = True
		# print("path_received decision make ", self.path_received)

	def cnn_path_callback(self, msg):
		self.path_received = True
		# print("cnn path_received decision make ", self.path_received)

	# def pub_speed_ref(self, event):
	# 	print('speed state cb')
	# 	if self.flag_pub_vel:
	# 		v = Float64()
	# 		v.data = self.speed_ref
	# 		self.vel_pub.publish(v)
	def pub_speed_ref(self):
		# print('speed state cb')
		if self.flag_pub_vel:
			v = Float64()
			v.data = self.speed_ref
			self.vel_pub.publish(v)

	def traffic_signs_cb(self, msg):
		# print('traffic signs state cb')

		if self.current_pose is None:
			return

		pose = np.array([self.current_pose.pose.pose.position.x, self.current_pose.pose.pose.position.y])	
		for sign in msg.signs:
			pose_sign = np.array([sign.pose.pose.position.x, sign.pose.pose.position.y])	
			dist = np.sqrt(np.power(pose_sign - pose, 2).sum())
			# print(sign.type)
			if sign.type == sign.list.STOP:

				self.stop_sign_ahead ['time'] = msg.header.stamp
				self.stop_sign_ahead ['stamp'] = msg.header.stamp#rospy.Time().now()
				self.stop_sign_ahead ['pose'] = sign.pose.pose
				self.stop_sign_ahead ['dist'] = dist
				# print ('\033[92m [Stop Sign] \033[0m dist:{}'.format(dist))

	def shutdown_cb(self, msg):
		if msg.data:

			self.speed = None
			self.current_pose = None
			self.current_odom = None
			self.last_time_odom=None
			self.obstacle_ahead = None
			self.traffic_light_ahead = None
			self.speed_max_param=None
			self.speed_limit_ahead = None

			self.stop_sign_ahead = None
			self.speed_ref = None
			self.flag_pub_vel = None
			self.path_received = None
			self.emergency_stop_flag=None
			self.emergency_stop_pub=None
			self.collision = None
			self.frames_ignore_stop=None

			self.names_state = None
			self.STATE_TRAFFIC_LIGHT = None
			self.STATE_FOLLOWING_LEADER = None
			self.STATE_FOLLOWING_LANE = None
			self.STATE_DECELERATE = None
			self.STATE_IDLE_STOP = None
			self.state = None

			self.time_decelerate=None
			self.time_stopped=None

			del self.pose_sub 
			del self.odom_sub 
			del self.vis_odom_sub 
			del self.state_sub 
			del self.obst_sub 
			del self.obst_tl_sub 
			del self.path_sub 
			del self.cnn_path_sub 
			del self.shutdown_sub 
			del self.traffic_sign_sub 
			del self.risk_of_collision 
			del self.vel_pub 
			del self.emergency_stop_pub 
			del self.fsm_state 

			print ("Bye!")
			rospy.signal_shutdown("finished route")

# if __name__ == '__main__':
# 	rospy.Timer(rospy.Duration(0.05), self.pub_speed_ref)
# 	rospy.spin()
def main(args):
	print ("[Decision Making Node] running...")
	rospy.init_node("fsm_decision_making_node", anonymous=True)
	fsm = FSM()
	rospy.spin()


if __name__ == '__main__':
	main(sys.argv)