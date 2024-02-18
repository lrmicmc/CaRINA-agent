#!/usr/bin/env python3
import numpy as np
import math
import roslib
import rospy

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
# util
current_speed = 0
reference_speed = 0
emergency_stop = False
speed_constraint = -1
last_speed_zero = None
timeout_emergency_stop = 450.0
#timeout_emergency_stop = 90
ignore_emergency_stop = False

throttle_pub = rospy.Publisher( '/carina/control/throttle_cmd', Throttle, queue_size=1)
brake_pub = rospy.Publisher( '/carina/control/brake_cmd', Brake, queue_size=1)
steering_pub = rospy.Publisher( '/carina/control/steer_cmd', SteeringAngle, queue_size=1)
hand_brake_pub = rospy.Publisher( '/carina/control/hand_brake_cmd', Bool, queue_size=1)
traj_error_pub  = rospy.Publisher('/carina/control/trajectory_error', TrajectoryError, queue_size=1)
longitudinal_control = PID_long_control(0)


# Callbacks definition

def vehicle_state_callback(data):
  global current_speed

  current_speed = data.drive.speed

def ackermann_cmd_callback(data):   
	stamp=data.header.stamp
	global reference_speed, longitudinal_control
	global emergency_stop 
	global current_speed
	global speed_constraint
	global last_speed_zero
	global timeout_emergency_stop
	global ignore_emergency_stop

	reference_speed = data.drive.speed
	curvature = abs(np.tan(data.drive.steering_angle)/2.85) #Axis distance
	max_lateral_acceleration = 5#1#0.05
	max_reference_speed = np.sqrt(max_lateral_acceleration*(1./curvature))

	# print('max_reference_speed: ',max_reference_speed, 'reference_speed: ', reference_speed)

	reference_speed = min(max_reference_speed, reference_speed)
	if speed_constraint>0:
		reference_speed = min(reference_speed, speed_constraint)
		speed_constraint = -1

	accel = 0.0
	deccel = 0.0

	c, error = longitudinal_control.generate_accel(current_speed, reference_speed)
   
	if c > 0:
		accel = min(1.0, 2*c)        
	else:
		deccel = min(0.001, 0.0001*abs(c))


	# #maybe the better way to do this is set reference_speed to 0.0, to avoid suddently stop
	if emergency_stop:
		# print ("\033[91m[Longitudinal Control]\033[0m EmergencyStop!!!")
		 
		if current_speed < 0.1 and current_speed>=0.0:
			if last_speed_zero is None:
				# last_speed_zero = rospy.Time().now()
				last_speed_zero = stamp#rospy.Time().now()


			# if (rospy.Time().now() - last_speed_zero).to_sec() >= timeout_emergency_stop:
			# print('time stoped:  ',(stamp - last_speed_zero).to_sec())
			if (stamp - last_speed_zero).to_sec() >= timeout_emergency_stop:
				ignore_emergency_stop = True
				# print ("\033[93m[Longitudinal Control]\033[0m Ignore EmergencyStop ({}/8.0)!!!".format((rospy.Time().now() - last_speed_zero).to_sec() - timeout_emergency_stop))
				# print ("\033[93m[Longitudinal Control]\033[0m Ignore EmergencyStop ({}/8.0)!!!".format((stamp - last_speed_zero).to_sec() - timeout_emergency_stop))

			else:
				ignore_emergency_stop = False
		

		if not ignore_emergency_stop:

			deccel = 1.0 
			accel  = 0.0
			reference_speed = 0.0

		if last_speed_zero is not None:
			# if (rospy.Time().now() - last_speed_zero).to_sec() >= (timeout_emergency_stop + 8.0):
			if (stamp - last_speed_zero).to_sec() >= (timeout_emergency_stop + 3.0):
			# if (stamp - last_speed_zero).to_sec() >= (timeout_emergency_stop + 10.0):

			# if (rospy.Time().now() - last_speed_zero).to_sec() >= (timeout_emergency_stop + 3.0):

				last_speed_zero = None
				ignore_emergency_stop = False



	# print (current_speed, data.drive.speed, accel, deccel)

	throttle = Throttle()
	throttle.value = accel

	brake = Brake()
	brake.value = deccel

	steer = SteeringAngle()
	steer.angle = data.drive.steering_angle

	t_error = TrajectoryError()
	# t_error.header.stamp = rospy.Time().now()
	t_error.header.stamp = stamp#rospy.Time().now()	
	t_error.enable[t_error.LONGITUDINAL] = True
	t_error.longitudinal_error = error

	traj_error_pub.publish(t_error)
	throttle_pub.publish(throttle)
	brake_pub.publish(brake)
	steering_pub.publish(steer)

	hand_brake = Bool()
	hand_brake.data = False

	if reference_speed == 0.0:
		hand_brake.data = True
		longitudinal_control.integ_error = 0
		# print ("\033[93m[Longitudinal Control]\033[0m Hand Brake!!!")    

	hand_brake_pub.publish(hand_brake)
	

def shutdown_cb(msg):
	if msg.data:
		print ("Bye!")
		rospy.signal_shutdown("finished route")

def emergency_stop_cb(msg):
	global emergency_stop
	if msg.stop_now:
		emergency_stop = True
	else:
		emergency_stop = False

def speed_constraint_cb(msg):
	global speed_constraint

	# print ("\033[91m[Longitudinal Control]\033[0m speed constraint!!!") 
	# print (msg.reason)

	if speed_constraint < 0:
		speed_constraint = msg.speed
	else:
		speed_constraint = min(speed_constraint, msg.speed)


def listener():

  print ("running longitudinal control")
  rospy.init_node('longitudinal_control', anonymous=True)
  rospy.Subscriber("/carina/control/ackermann_cmd", AckermannDriveStamped, ackermann_cmd_callback)
  rospy.Subscriber("/carina/vehicle/state", VehicleState, vehicle_state_callback)
  rospy.Subscriber("/carina/vehicle/shutdown", Bool, shutdown_cb, queue_size=1)
  rospy.Subscriber("/carina/control/speed_constraint", SpeedConstraint, speed_constraint_cb, queue_size=1)
  emergency_stop_sub = rospy.Subscriber('/carina/navigation/emergency_stop', EmergencyStop, emergency_stop_cb)
  rospy.spin()

if __name__ == '__main__':
  listener()















