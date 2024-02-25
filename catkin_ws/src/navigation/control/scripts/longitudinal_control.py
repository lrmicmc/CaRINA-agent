#!/usr/bin/env python3
import numpy as np
import math
import roslib
import rospy
import sys

from ackermann_msgs.msg import AckermannDriveStamped
from msgs_action.msg import VehicleState, Throttle, Brake, SteeringAngle
from std_msgs.msg import Bool
from msgs_navigation.msg import EmergencyStop, SpeedConstraint, TrajectoryError


class PID_long_control:

	def __init__(self, v_prev):

		self.v_prev = v_prev
		self.integ_error = 0
		self.accel = 0
		self.prev_error = 0

	def generate_accel(self, v, vref):

		deriv_error = v - self.v_prev
		error = vref - v
		if np.sign(error) != np.sign(self.prev_error):
			self.integ_error = 0
		self.integ_error = self.integ_error + error
		self.prev_error = error
		Kp = 0.4
		Ki = 0.04
		Kd = 0.001
		# if error <= -1.5:
		#     print  "\033[93m[Longitudinal Control]\033[0m BRAKE" 
		#     return -1, error
		control = (Kp*error + Ki*self.integ_error + Kd*deriv_error)
		self.v_prev = v
		return control, error


class LongitudinalControl(object):

	def __init__ (self):

		self.current_speed = 0
		self.reference_speed = 0
		self.emergency_stop = False
		self.previous_emergency_stop = False
		self.speed_constraint = -1
		self.last_speed_zero = None
		self.timeout_emergency_stop = 160.0
		#self.timeout_emergency_stop = 90
		self.ignore_emergency_stop = False
		self.longitudinal_control = PID_long_control(0)

		self.frames_save_speed_constraint=100

		self.akermamn_sub=rospy.Subscriber("/carina/control/ackermann_cmd", AckermannDriveStamped, self.ackermann_cmd_callback)
		self.state_sub=rospy.Subscriber("/carina/vehicle/state", VehicleState, self.vehicle_state_callback)
		self.shutdown_sub=rospy.Subscriber("/carina/vehicle/shutdown", Bool, self.shutdown_cb, queue_size=1)
		self.speed_sub=rospy.Subscriber("/carina/control/speed_constraint", SpeedConstraint, self.speed_constraint_cb, queue_size=1)
		self.stop_sub=emergency_stop_sub = rospy.Subscriber('/carina/navigation/emergency_stop', EmergencyStop, self.emergency_stop_cb)

		self.throttle_pub = rospy.Publisher( '/carina/control/throttle_cmd', Throttle, queue_size=1)
		self.brake_pub = rospy.Publisher( '/carina/control/brake_cmd', Brake, queue_size=1)
		self.steering_pub = rospy.Publisher( '/carina/control/steer_cmd', SteeringAngle, queue_size=1)
		self.hand_brake_pub = rospy.Publisher( '/carina/control/hand_brake_cmd', Bool, queue_size=1)
		self.traj_error_pub  = rospy.Publisher('/carina/control/trajectory_error', TrajectoryError, queue_size=1)


	def vehicle_state_callback(self, data):
	  self.current_speed = data.drive.speed


	def ackermann_cmd_callback(self, data):
		# print('ackermann.. ')
		# print('self.speed_constraint ',self.speed_constraint)
		# print('self.energency stop ',self.emergency_stop)
		# print('self.frames_save_speed_constraint ', self.frames_save_speed_constraint)

		stamp=data.header.stamp
		self.reference_speed = data.drive.speed

		if self.current_speed > 2.5:
			curvature = abs(np.tan(data.drive.steering_angle)/2.85) #Axis distance
			max_lateral_acceleration = 4#1#0.05
			max_reference_speed = np.sqrt(max_lateral_acceleration*(1./curvature))
			# print('max_reference_speed: ',max_reference_speed, 'reference_speed: ', reference_speed)
			self.reference_speed = min(max_reference_speed, self.reference_speed)


		if self.speed_constraint>0:
			self.reference_speed = min(self.reference_speed, self.speed_constraint)

			if self.frames_save_speed_constraint > 4:
				self.speed_constraint = -1

		self.frames_save_speed_constraint+=1

		accel = 0.0
		deccel = 0.0
		c, error = self.longitudinal_control.generate_accel(self.current_speed, self.reference_speed)
	   

		if c > 0:
			accel = min(1.0, 2*c)        
			# accel = min(1.0, 2*c)        
		else:
			deccel = min(0.001, 0.0001*abs(c))

		# #maybe the better way to do this is set reference_speed to 0.0, to avoid suddently stop
		if self.emergency_stop:
			# print ("\033[91m[Longitudinal Control]\033[0m EmergencyStop!!!")
			if self.current_speed < 0.1 and self.current_speed>=0.0:
				if self.last_speed_zero is None:
					# self.last_speed_zero = rospy.Time().now()
					self.last_speed_zero = stamp#rospy.Time().now()
				# if (rospy.Time().now() - self.last_speed_zero).to_sec() >= self.timeout_emergency_stop:
				# print('time stoped:  ',(stamp - self.last_speed_zero).to_sec())
				if (stamp - self.last_speed_zero).to_sec() >= self.timeout_emergency_stop:
					self.ignore_emergency_stop = True
					# print ("\033[93m[Longitudinal Control]\033[0m Ignore EmergencyStop ({}/8.0)!!!".format((rospy.Time().now() - self.last_speed_zero).to_sec() - self.timeout_emergency_stop))
					# print ("\033[93m[Longitudinal Control]\033[0m Ignore EmergencyStop ({}/8.0)!!!".format((stamp - self.last_speed_zero).to_sec() - self.timeout_emergency_stop))
				else:
					self.ignore_emergency_stop = False
			if not self.ignore_emergency_stop:
				deccel = 1.0 
				accel  = 0.0
				self.reference_speed = 0.0

			if self.last_speed_zero is not None:
				# if (rospy.Time().now() - self.self.last_speed_zero).to_sec() >= (self.timeout_emergency_stop + 8.0):
				if (stamp - self.last_speed_zero).to_sec() >= (self.timeout_emergency_stop + 1.0):
				# if (stamp - self.last_speed_zero).to_sec() >= (self.timeout_emergency_stop + 10.0):
				# if (rospy.Time().now() - self.last_speed_zero).to_sec() >= (self.timeout_emergency_stop + 3.0):
					self.last_speed_zero = None
					self.ignore_emergency_stop = False

		# print (current_speed, data.drive.speed, accel, deccel)

		if self.current_speed < 2. and self.reference_speed > 2.: 
			accel=1.

		print ('current_speed: ', self.current_speed, '   self.reference_speed: ', self.reference_speed, '   accel: ', accel)#, deccel)

		throttle = Throttle()
		throttle.value = accel

		brake = Brake()
		brake.value = deccel

		steer = SteeringAngle()
		steer.angle = data.drive.steering_angle

		t_error = TrajectoryError()
		t_error.header.stamp = stamp
		t_error.enable[t_error.LONGITUDINAL] = True
		t_error.longitudinal_error = error

		# print('throttle ', throttle)
		self.traj_error_pub.publish(t_error)
		self.throttle_pub.publish(throttle)
		self.brake_pub.publish(brake)
		self.steering_pub.publish(steer)

		hand_brake = Bool()
		hand_brake.data = False

		if self.reference_speed == 0.0:
			hand_brake.data = True
			self.longitudinal_control.integ_error = 0
			# print ("\033[93m[Longitudinal Control]\033[0m Hand Brake!!!")    
		self.hand_brake_pub.publish(hand_brake)
		

	def emergency_stop_cb(self, msg):
		if msg.stop_now or self.previous_emergency_stop:
			self.emergency_stop = True
		else:
			self.emergency_stop = False

		self.previous_emergency_stop=msg.stop_now

	def speed_constraint_cb(self, msg):
		# print ("\033[91m[Longitudinal Control]\033[0m speed constraint!!!") 
		# print (msg.reason)
		if self.speed_constraint < 0:
			self.speed_constraint = msg.speed
		else:
			self.speed_constraint = min(self.speed_constraint, msg.speed)
		self.frames_save_speed_constraint=0


	def shutdown_cb(self, msg):
		if msg.data:
			
			self.current_speed = 0
			self.reference_speed = 0
			self.emergency_stop = False
			self.speed_constraint = -1
			self.last_speed_zero = None
			self.timeout_emergency_stop = 120.0
			#self.timeout_emergency_stop = 90
			self.ignore_emergency_stop = False
			self.longitudinal_control = None

			del self.akermamn_sub
			del self.state_sub
			del self.shutdown_sub
			del self.speed_sub
			del self.stop_sub

			del self.throttle_pub
			del self.brake_pub
			del self.steering_pub
			del self.hand_brake_pub
			del self.traj_error_pub

			print ("Bye!")
			rospy.signal_shutdown("finished route")


def main(args):
	print ("[Longitudinal control Node] running...")
	rospy.init_node("longitudinal_control", anonymous=True)
	l_control = LongitudinalControl()
	rospy.spin()


if __name__ == '__main__':
	main(sys.argv)











