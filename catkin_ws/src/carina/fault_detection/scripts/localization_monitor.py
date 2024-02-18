#!/usr/bin/env python3
import numpy as np
import rospy
import sys
from geometry_msgs.msg import PoseWithCovarianceStamped#, Twist, PoseStamped 
from msgs_action.msg import VehicleState
from std_msgs.msg import Float64, Bool, String
from nav_msgs.msg import Odometry
from msgs_navigation.msg import EmergencyStop
import os
import subprocess
class GPSMonitor(object):

	def __init__ (self):
		self.current_pose = None
		self.current_odom = None
		self.last_time_odom=0.0#rospy.Time().now().to_sec()
		print('last_time_odom init ', self.last_time_odom)
		self.localization_fault=True
		self.first_frame=False
		self.pose_sub = rospy.Subscriber('/carina/localization/pose', PoseWithCovarianceStamped, self.pose_cb, queue_size=1)
		self.odom_v_sub = rospy.Subscriber('/localization/odometry/visual', Odometry, self.visual_odom_cb, queue_size=1)
		self.odom_sub = rospy.Subscriber('/carina/localization/odom', Odometry, self.odom_cb, queue_size=1)
		self.state_sub = rospy.Subscriber('/carina/vehicle/state', VehicleState, self.vehicle_state_cb, queue_size=1)
		self.shutdown_sub = rospy.Subscriber('/carina/vehicle/shutdown', Bool, self.shutdown_cb, queue_size=1)
		
		self.emergency_stop_pub = rospy.Publisher('/carina/navigation/emergency_stop', EmergencyStop, queue_size=1)
		self.pub_error = rospy.Publisher('/carina/fault/localization_error', String, queue_size=1)

	def vehicle_state_cb(self, msg):
		if self.last_time_odom == 0.0:
			print('last_time_odom vehicle func', self.last_time_odom)
			print('Waiting for first gps frame')
			#return
		# print('vehicle state cb')
		# self.speed = msg.drive.speed

		time_to_last_odom = (msg.header.stamp.to_sec() - self.last_time_odom)
		print('time_from_last_odom', time_to_last_odom)

		if not self.localization_fault and time_to_last_odom > 3.: # verify that the odom is published and restart node after 3. sec 
					#running nodes
			self.last_time_odom=msg.header.stamp.to_sec()
			self.localization_fault = True

			command_path='/'
			for c in os.path.realpath(__file__).split('/')[0:-1]:
				command_path = os.path.join(command_path, c)
			command_path=os.path.join(command_path, 'restart_localization.sh')
			rospy.loginfo("restarting localization stack...")
			if not os.path.exists(command_path):
				raise IOError("File '{}' defined by restar command invalid".format(command_path))
			print(command_path)

			self.stack_process = subprocess.Popen(command_path, shell=True, preexec_fn=os.setpgrp)

			# restarting=os.system("bash " + command_path)
			print('restarting ')

			fault_string = "Covariance error %s" % rospy.get_time()
			self.pub_error.publish(fault_string)


	def visual_odom_cb(self, msg):
		# print('visual odom state cb')
		# self.current_odom = msg
		print('last_time_odom ',msg.header.stamp.to_sec())
		print('covariance [0] ',msg.pose.covariance[0])
		if msg.pose.covariance[0]<9000:
			self.last_time_odom=msg.header.stamp.to_sec()
			self.localization_fault=False

	def odom_cb(self, msg):
		# print('odom state cb')
		# self.current_odom = msg
		self.last_time_odom=msg.header.stamp.to_sec()
		self.localization_fault=False

	def pose_cb(self, msg):
		# print('pose state cb')
		self.current_pose = msg

	def shutdown_cb(self, msg):
		if msg.data:

			self.localization_fault=True
			del self.pose_sub
			del self.state_sub
			del self.shutdown_sub
			del self.emergency_stop_pub
			del self.odom_v_sub
			del self.odom_sub
			print ("Bye!")
			rospy.signal_shutdown("finished route")


# if __name__ == '__main__':
# 	rospy.Timer(rospy.Duration(0.05), self.pub_speed_ref)
# 	rospy.spin()
def main(args):
	print ("[GPR monitor Node] running...")
	rospy.init_node("gps_monitor_node", anonymous=True)
	GPS = GPSMonitor()
	rospy.spin()


if __name__ == '__main__':
	main(sys.argv)
