#!/usr/bin/env python

import numpy as np
import math
import os
import time
from scipy.optimize import minimize
import rospy


cont = np.inf

def norm_ang( theta ):
	if theta > math.pi:
		theta_n = theta - 2*math.pi
	elif theta < -math.pi:
		theta_n = 2*math.pi + theta
	else:
		theta_n = theta
	return theta_n

class MPC_Trajectory_Generator:

	def __init__(self, path, dt=0.5):

		self.x = 0
		self.y = 0
		self.theta = 0
		self.kappa = 0
		self.v = 0
		self.accel = 0
		self.ster_rate = 0
		self.dT = dt
		self.bnds, self.cons = MPC_Trajectory_Generator.initialize_solver(self)
		self.xref = path[:,0]
		self.yref = path[:,1]
		self.thetaref = path[:,3]
		self.kapparef = path[:,2]
		self.vref = np.zeros(3)
		self.time = 0.


	def update_ref(self, way):
		# self.xref = np.concatenate((self.xref, way[:,0]))
		# self.yref = np.concatenate((self.yref, way[:,1]))
		# self.thetaref = np.concatenate((self.thetaref, way[:,3]))
		# self.kapparef = np.concatenate((self.kapparef, way[:,2]))
		self.xref = way[:,0]
		self.yref = way[:,1]
		self.thetaref = way[:,3]
		self.kapparef = way[:,2]
		
	def update_vel_ref(self, v):
		self.vref = v*np.ones(3)

	def initial_guess(self):
		#Compute initial guess for nonlinear optimization
		dt = self.dT
		x_init = np.zeros(21)
		x0_init = self.x
		y0_init = self.y
		theta0_init = self.theta
		kappa0_init = self.kappa
		v0_init = self.v
		a0 = self.accel
		tau0 = self.ster_rate

		for k in (0, 7, 14):

			#Simulate model
			x0_init = x0_init + v0_init*np.cos(theta0_init)*dt + (1./2.)*(np.cos(theta0_init)*a0 - \
				v0_init**2*np.sin(theta0_init)*kappa0_init)*dt**2 - (1./6.)*(3*v0_init*np.sin(theta0_init)*kappa0_init*a0 + \
				v0_init**2*np.sin(theta0_init)*1*tau0 + v0_init**3*np.cos(theta0_init)*kappa0_init**2)*dt**3
			y0_init = y0_init + v0_init*np.sin(theta0_init)*dt + (1./2.)*(np.sin(theta0_init)*a0 + \
				v0_init**2*np.cos(theta0_init)*kappa0_init)*dt**2 + (1./6.)*(3*v0_init*np.cos(theta0_init)*kappa0_init*a0 + \
				v0_init**2*np.cos(theta0_init)*1*tau0 - v0_init**3*np.sin(theta0_init)*kappa0_init**2)*dt**3
			theta0_init = theta0_init + (v0_init*kappa0_init)*dt + (1./2.)*(kappa0_init*a0 + v0_init/1*tau0)*dt**3
			kappa0_init = kappa0_init + tau0*dt
			v0_init = v0_init + a0*dt

			#Store solutions
			x_init[k:k+7] = (x0_init, y0_init, theta0_init, kappa0_init, v0_init, self.ster_rate, self.accel)
		return x_init

	def initialize_solver(self):
		def constraint1(x):
			T = self.dT
			return x[0] + x[4]*np.cos(x[2])*T + (1./2.)*(np.cos(x[2])*x[6] - \
			x[4]**2*np.sin(x[2])*x[3])*T**2 - (1./6.)*(3*x[4]*np.sin(x[2])*x[3]*x[6] + \
			x[4]**2*np.sin(x[2])*1*x[5] + x[4]**3*np.cos(x[2])*x[3]**2)*T**3 - x[7]

		def constraint2(x):
			T = self.dT
			return x[1] + x[4]*np.sin(x[2])*T + (1./2.)*(np.sin(x[2])*x[6] + \
			x[4]**2*np.cos(x[2])*x[3])*T**2 + (1./6.)*(3*x[4]*np.cos(x[2])*x[3]*x[6] + \
			x[4]**2*np.cos(x[2])*1*x[5] - x[4]**3*np.sin(x[2])*x[3]**2)*T**3 - x[8]

		def constraint3(x):
			T = self.dT
			return x[2] + (x[4]*x[3])*T + (1./2.)*(x[3]*x[6] + x[4]/1*x[5])*T**3 - x[9]

		def constraint4(x):
			T = self.dT
			return x[7] + x[11]*np.cos(x[9])*T + (1./2.)*(np.cos(x[9])*x[13] - \
			x[11]**2*np.sin(x[9])*x[10])*T**2 - (1./6.)*(3*x[11]*np.sin(x[9])*x[10]*x[13] + \
			x[11]**2*np.sin(x[9])*1*x[12] + x[11]**3*np.cos(x[9])*x[10]**2)*T**3 - x[14]

		def constraint5(x):
			T = self.dT
			return x[8] + x[11]*np.sin(x[9])*T + (1./2.)*(np.sin(x[9])*x[13] + \
			x[11]**2*np.cos(x[9])*x[10])*T**2 + (1./6.)*(3*x[11]*np.cos(x[9])*x[10]*x[13] + \
			x[11]**2*np.cos(x[9])*1*x[12] - x[11]**3*np.sin(x[9])*x[10]**2)*T**3 - x[15]

		def constraint6(x):
			T = self.dT
			return x[9] + (x[11]*x[10])*T + (1./2.)*(x[10]*x[13] + \
			x[11]/1*x[12])*T**3 - x[16]

		def constraint7(x):
			T = self.dT
			vehicle_x = self.x
			vehicle_head = self.theta
			vehicle_kappa = self.kappa
			vehicle_speed = self.v

			return vehicle_x + vehicle_speed*np.cos(vehicle_head)*T + (1./2.)*(np.cos(vehicle_head)*x[6] - \
			vehicle_speed**2*np.sin(vehicle_head)*vehicle_kappa)*T**2 - (1./6.)*(3*vehicle_speed*np.sin(vehicle_head)*vehicle_kappa*x[6] + \
			vehicle_speed**2*np.sin(vehicle_head)*1*x[5] + vehicle_speed**3*np.cos(vehicle_head)*vehicle_kappa**2)*T**3 - x[0]

		def constraint8(x):
			T = self.dT
			vehicle_y = self.y
			vehicle_head = self.theta
			vehicle_kappa = self.kappa
			vehicle_speed = self.v

			return vehicle_y + vehicle_speed*np.sin(vehicle_head)*T + (1./2.)*(np.sin(vehicle_head)*x[6] + \
			vehicle_speed**2*np.cos(vehicle_head)*vehicle_kappa)*T**2 + (1./6.)*(3*vehicle_speed*np.cos(vehicle_head)*vehicle_kappa*x[6] + \
			vehicle_speed**2*np.cos(vehicle_head)*1*x[5] - vehicle_speed**3*np.sin(vehicle_head)*vehicle_kappa**2)*T**3 - x[1]

		def constraint9(x):
			T = self.dT
			vehicle_head = self.theta
			vehicle_kappa = self.kappa
			vehicle_speed = self.v

			return vehicle_head + (vehicle_speed*vehicle_kappa)*T + (1./2.)*(vehicle_kappa*x[6] + vehicle_speed/1*x[5])*T**3 - x[2]

		def constraint10(x):
			T = self.dT
			vehicle_kappa = self.kappa

			return vehicle_kappa + x[5]*T - x[3]

		def constraint11(x):
			T = self.dT
			vehicle_speed = self.v

			return vehicle_speed + x[6]*T - x[4]

		def constraint12(x):
			T = self.dT
			return x[3] + x[12]*T - x[10]

		def constraint13(x):
			T = self.dT
			return x[4] + x[13]*T - x[11]

		def constraint14(x):
			T = self.dT
			return x[10] + x[19]*T - x[17]

		def constraint15(x):
			T = self.dT
			return x[11] + x[20]*T - x[18]

		#Constants bounds
		kappa_max = 0.52     #Maximum curvature
		tau_max = 0.5     #Maximum curvature rate change
		a_max = 4.       #Maximum acceleration

		#Time step horizon
		T = self.dT

		bx1 = (None, None)
		bx2 = (None, None)
		bx3 = (None, None)
		bx4 = (-kappa_max, kappa_max)
		bx5 = (0, None)
		bx6 = (-tau_max, tau_max)
		bx7 = (-a_max, a_max)

		bnds = (bx1,bx2,bx3,bx4,bx5,bx6,bx7,bx1,bx2,bx3,bx4,bx5,bx6,\
		bx7,bx1,bx2,bx3,bx4,bx5,bx6,bx7)

		con1 = {'type': 'eq', 'fun': constraint1}
		con2 = {'type': 'eq', 'fun': constraint2}
		con3 = {'type': 'eq', 'fun': constraint3}
		con4 = {'type': 'eq', 'fun': constraint4}
		con5 = {'type': 'eq', 'fun': constraint5}
		con6 = {'type': 'eq', 'fun': constraint6}
		con7 = {'type': 'eq', 'fun': constraint7}
		con8 = {'type': 'eq', 'fun': constraint8}
		con9 = {'type': 'eq', 'fun': constraint9}
		con10 = {'type': 'eq', 'fun': constraint10}
		con11 = {'type': 'eq', 'fun': constraint11}
		con12 = {'type': 'eq', 'fun': constraint12}
		con13 = {'type': 'eq', 'fun': constraint13}
		con14 = {'type': 'eq', 'fun': constraint14}
		con15 = {'type': 'eq', 'fun': constraint15}

		cons = ([con1,con2,con3,con4,con5,con6,con7,con8,con9,con10,con11,con12,con13,\
		con14,con15])

		return bnds, cons

	def set_reference_path(self, x_ref, y_ref, theta_ref, kappa_ref, v_ref):
		self.xref = x_ref
		self.yref = y_ref
		self.thetaref = theta_ref
		self.kapparef = kappa_ref
		self.vref = v_ref

	def generate_trajectory(self, x, y, theta, steer_ang, v):

		def objective(x):
			#Define objective function

			#Optimization weights
			Cv = 30.0
			Cx = 10.0
			Cy = 10.0
			Ctheta = 10.0
			Ckappa = 30.0
			ClatAccel = 30.

			return Cv*(x[4]-ref1[4])**2 + Cv*(x[11]-ref2[4])**2 + Cv*(x[18]-ref3[4])**2 +\
				Cx*(x[0]-ref1[0])**2 + Cx*(x[7]-ref2[0])**2 + Cx*(x[14]-ref3[0])**2 + \
				Cy*(x[1]-ref1[1])**2 + Cy*(x[8]-ref2[1])**2 + Cy*(x[15]-ref3[1])**2 + \
				Ctheta*(norm_ang(x[2]-ref1[2]))**2 + Ctheta*(norm_ang(x[9]-ref2[2]))**2 + Ctheta*(norm_ang(x[16]-ref3[2]))**2 + \
				Ckappa*(x[3]-ref1[3])**2 + Ckappa*(x[10]-ref2[3])**2 + Ckappa*(x[17]-ref3[3])**2
				# ClatAccel*((x[4])**2*abs(self.kappa)) + ClatAccel*((x[11])**2*abs(self.kappa)) + ClatAccel*((x[18])**2*abs(self.kappa)) 
				# ClatAccel*((x[4])**2*abs(x[3])) + ClatAccel*((x[11])**2*abs(x[10])) + ClatAccel*((x[18])**2*abs(x[17]))

		#Time step horizon
		T = self.dT

		# Axis distance
		# d=2.85
		d=2.94

		#back axis translation
		# back_axis = 0.
		back_axis = -1.55#-1.70
		# back_axis = -1.45
		#back_axis = -1.049

		self.x = x + (back_axis*np.cos(theta))
		self.y = y + (back_axis*np.sin(theta))
		self.theta = theta
		self.kappa = np.tan(steer_ang)/d
		self.v = v
		last_time_gps = rospy.Time().now()

		#Compute initial guess solution
		x_init = self.initial_guess()

		# self.vref = np.ones(3)*min(np.sqrt(1.5/abs(self.kappa+1e-10)), self.vref[0])
		if abs(self.kappa) > 0.10:
			self.time = rospy.Time().now().to_sec()

		if rospy.Time().now().to_sec() - self.time < 10.:
			self.vref = 4.*np.ones(3)


		#Compute closest points ref1, ref2 and ref3
		#Three references because we have horizon = 4
		allDistances = np.sqrt((x_init[0]-self.xref)**2+(x_init[1]-self.yref)**2)
		n1 = np.argmin(allDistances)
		ref1 = (self.xref[n1], self.yref[n1], self.thetaref[n1], self.kapparef[n1], self.vref[0])

		allDistances = np.sqrt((x_init[7]-self.xref)**2+(x_init[8]-self.yref)**2)
		n2 = np.argmin(allDistances)
		ref2 = (self.xref[n2], self.yref[n2], self.thetaref[n2], self.kapparef[n2], self.vref[1])

		allDistances = np.sqrt((x_init[14]-self.xref)**2+(x_init[15]-self.yref)**2)
		n3 = np.argmin(allDistances)
		ref3 = (self.xref[n3], self.yref[n3], self.thetaref[n3], self.kapparef[n3], self.vref[2])

		#Compute optimal solution
		solution = minimize(objective, x_init,method='SLSQP',\
							bounds=self.bnds,constraints=self.cons)

		x = solution.x

		self.ster_rate = x[5]
		self.accel = x[6]

		#Compute lateral error
		allDistances = np.sqrt((self.x-self.xref)**2+(self.y-self.yref)**2)
		n = np.argmin(allDistances)
		ref = (self.xref[n], self.yref[n], self.thetaref[n], self.kapparef[n], self.vref[0])
		coef_b = 1
		coef_a = -np.tan(self.thetaref[n])
		coef_c = np.tan(self.thetaref[n])*self.xref[n]-self.yref[n]
		lat_error = abs(coef_a*self.x + coef_b*self.y + coef_c)/np.sqrt(coef_a**2+coef_b**2)

		#print(self.xref[n], self.yref[n], self.x, self.y)

		#Compute heading error, curvature error and velocity error
		theta_error = ref[2] - self.theta
		kappa_error = ref[3] - self.kappa
		v_error = ref[4] - self.v

		input_curv_rate = self.ster_rate
		input_accel = self.accel


		#Update curvature and speed
		# dt = (rospy.Time().now() - last_time_gps).to_nsec()/1e9
		# dt = max(0.1, dt)
		# dt = 0.5
		dt = 0.3
		
		kappa_input = self.kappa + input_curv_rate*(dt)
		velocity_input = self.v + input_accel*(dt)

		# if abs(kappa_input) > 0.52:
		# 	kappa_input = np.sign(kappa_input)*0.52

		kappa_input = kappa_input + 1e-10
		steeringAngle = math.atan(kappa_input*d)
		acceLatMax = 2.
		speed = min(max(velocity_input, 1.), ref[4])

		# input_steer_rate = math.atan(input_curv_rate*d)
		# inpute_speed_rate = input_accel

		'''
		print('lat_error: ' + str(np.round(lat_error,2)) + ' ' +\
		'theta_error: ' + str(np.round(norm_ang(theta_error)*180./math.pi,2)) +\
		 ' ' +'steeringAngle: ' + str(np.round(np.rad2deg(steeringAngle),3)) + ' ' +'speed: ' +\
		 str(np.round(v,2)) )
		'''

		return steeringAngle, speed, np.rad2deg(norm_ang(theta_error)), lat_error, kappa_error, ref[3]
