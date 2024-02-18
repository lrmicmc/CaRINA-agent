#!/usr/bin/env python3

#python 
from numpy import *
import math
import matplotlib.pyplot as plt
import time

#ros
import rospy
import tf


#ros msgs
from msgs_action.msg import VehicleState
from ackermann_msgs.msg import AckermannDriveStamped
from geometry_msgs.msg import PoseWithCovarianceStamped
from msgs_navigation.msg import TrajectoryPoint 
from msgs_navigation.msg import Path as NavPath
from std_msgs.msg import Bool, Float64
from msgs_navigation.msg import TrajectoryError

from msgs_navigation.msg import SpeedConstraint

#utilities
from mpc_traj_generator import *

waypoints = np.array([])
path = []

pathReceived = False

current_speed = 0
current_steer = 0

current_pose = PoseWithCovarianceStamped()

mpc_control = None

last_time_gps = 0.0

desired_angle_rate=0.0
desired_speed_rate=0.0
etheta = 0.0
el = 0.0
kappa_error = 0.0
desired_angle = 0.0
desired_speed = 0.0

trun_option_msg=False

def bootstrap():
	 global pathReceived,  waypoints, mpc_control

	 waypoints=np.load('way.npy')
	 mpc_control = MPC_Trajectory_Generator(waypoints)

	 
	 pathReceived = True


def pose_stamped_callback(msg):
	global current_speed, current_steer, current_pose
	global pathReceived 
	global waypoints
	global ackermann_pub
	global mpc_control
	global path
	global last_time_gps
	global desired_angle_rate, desired_speed_rate, etheta, el, kappa_error
	global desired_angle, desired_speed

	current_pose = msg
	# dt = 1.0/2.0

	# dt = (rospy.Time().now() - last_time_gps).to_nsec()/1e9
	
	# last_time_gps = rospy.Time().now()
	stamp=msg.header.stamp
	last_time_gps = msg.header.stamp#rospy.Time().now()


	if not pathReceived:
		 return

	if mpc_control is None: 
		 return

	# Orientation is represented in quaternion
	q = current_pose.pose.pose.orientation

	# Get yaw
	ori = (q.x,
			q.y, 
			q.z, 
			q.w)
	_, _, theta = tf.transformations.euler_from_quaternion(ori)


	x = current_pose.pose.pose.position.x
	y = current_pose.pose.pose.position.y


	desired_angle, desired_speed, etheta, el, kappa_error, k_ref = mpc_control.generate_trajectory(x, y, theta, current_steer, current_speed)

	# desired_angle = desired_angle_rate*dt + desired_angle
	# desired_speed = desired_speed_rate*dt + desired_speed

	# print ("Steer: ", desired_angle,", Speed:", desired_speed, ", Etheta:", etheta, ", El:", el, ', k_err:', kappa_error, ', k_ref: ', k_ref)

	ackermann_msg = AckermannDriveStamped()
	ackermann_msg.header.stamp = stamp#rospy.Time().now()
	ackermann_msg.drive.speed = desired_speed
	ackermann_msg.drive.steering_angle = desired_angle
	ackermann_pub.publish(ackermann_msg)

	t_error = TrajectoryError()
	t_error.header.stamp = stamp#rospy.Time().now()
	t_error.enable[t_error.LATERAL] = True
	t_error.enable[t_error.ANGULAR] = True
	t_error.enable[t_error.KAPPA] = True
	t_error.lateral_error = el
	t_error.angular_error=etheta
	t_error.kappa_error=kappa_error
	traj_erro_pub.publish(t_error)
	

def state_callback(msg):
	global current_speed, current_steer

	current_speed = msg.drive.speed
	current_steer = msg.drive.steering_angle


def path_callback(msg):
	# global path_cnn, trun_option_msg
	global trun_option_msg

	# print('trun_option_msg',trun_option_msg)

	
	# if trun_option_msg.data:
	# 	print('using cnn path')
	# 	msg=path_cnn

	global last_stamp
	#Path callback
	#get path to be followed
	global pathReceived, path, waypoints, event_view, mpc_control

	if msg.header.stamp>last_stamp:
		# print ("[Lateral Node] path received!!!")
		last_stamp = msg.header.stamp
		path = msg.path

		if len(waypoints) == 0:
			waypoints =  np.array([p.point for p in path])
			mpc_control = MPC_Trajectory_Generator(waypoints)
		else:
			# points =  np.array([p.point for p in path])
			# waypoints = np.concatenate((waypoints, points), axis=0)
			waypoints = np.array([p.point for p in path])
			mpc_control.update_ref(waypoints)
	
		pathReceived = True

def cnn_path_callback(msg):
	global path_cnn#, trun_option_msg
	# print('trun_option_msg',trun_option_msg)
	# path_cnn=msg

	global last_stamp
	#Path callback
	#get path to be followed
	global pathReceived, path, waypoints, event_view, mpc_control

	if msg.header.stamp>last_stamp:
		# print ("[Lateral Node] cnn path received!!!")
		last_stamp = msg.header.stamp
		path = msg.path

		if len(waypoints) == 0:
			waypoints =  np.array([p.point for p in path])
			mpc_control = MPC_Trajectory_Generator(waypoints)
		else:
			# points =  np.array([p.point for p in path])
			# waypoints = np.concatenate((waypoints, points), axis=0)
			waypoints = np.array([p.point for p in path])
			mpc_control.update_ref(waypoints)
	
		pathReceived = True


def turn_option(msg):
	global trun_option_msg
	trun_option_msg=msg


 
def velref_cb(msg):
	global mpc_control
 
	if mpc_control is not None:
		mpc_control.update_vel_ref(msg.data)




def shutdown_cb(msg):
	global waypoints, pathReceived
	global current_steer, current_speed

	if msg.data:
		waypoints = np.array([])
		pathReceived = False
		current_steer=0.0
		current_speed=0.0

		print ("Bye!")
		rospy.signal_shutdown("finished route")


def updateCommand(event):
	global mpc_control, current_pose
	global desired_angle_rate, desired_speed_rate, etheta, el, kappa_error
	global current_steer, current_speed, desired_angle, desired_speed

	if mpc_control is not None:
		
		if not pathReceived:
		  return


		# Orientation is represented in quaternion
		q = current_pose.pose.pose.orientation

		# Get yaw
		ori = (q.x,
				q.y, 
				q.z, 
				q.w)
		_, _, theta = tf.transformations.euler_from_quaternion(ori)

	
		x = current_pose.pose.pose.position.x
		y = current_pose.pose.pose.position.y
	

		desired_angle_rate, desired_speed_rate, etheta, el, kappa_error = mpc_control.generate_trajectory(x, y, theta, desired_angle, desired_speed)


if __name__ == "__main__":
		print("running...")
		rospy.init_node('lateral_control_node', anonymous=True)
		last_stamp = rospy.Time(0)
		
		ackermann_pub = rospy.Publisher("/carina/control/ackermann_cmd", AckermannDriveStamped, queue_size=1)
		traj_erro_pub = rospy.Publisher("/carina/control/trajectory_error", TrajectoryError, queue_size=1)

		rospy.Subscriber("/carina/localization/pose", PoseWithCovarianceStamped, pose_stamped_callback, queue_size=1)
		rospy.Subscriber("/carina/vehicle/state", VehicleState, state_callback, queue_size=1)    
		rospy.Subscriber("/carina/navigation/path_segment", NavPath, path_callback, queue_size=1)
		rospy.Subscriber("/carina/navigation/cnn_path", NavPath, cnn_path_callback, queue_size=1)
		rospy.Subscriber("/carina/vehicle/shutdown", Bool, shutdown_cb, queue_size=1)
		rospy.Subscriber("/carina/control/vel_ref", Float64, velref_cb, queue_size=1)

		rospy.Subscriber('/carina/control/near_to_intersection', Bool, turn_option, queue_size=1)


		last_time_gps = rospy.Time(0)#rospy.Time().now()

		# rospy.Timer(rospy.Duration(0.05), updateCommand)

		rospy.spin()
 
	 

