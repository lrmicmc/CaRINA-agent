#!/usr/bin/env python3

from leaderboard.autoagents.ros1_agent import ROS1Agent
from leaderboard.autoagents.autonomous_agent import Track
from std_msgs.msg import  Bool
import numpy as np
import os
import yaml

# import roslibpy
# import time


def get_entry_point():
	return 'CarinaAgent'

class CarinaAgent(ROS1Agent):
	def get_ros_entrypoint(self):
		return {
			"package": "carina_master",
			"launch_file": "carina.launch",
			"parameters": {}
		}

	def setup(self, path_to_conf_file):
		print('Setup')
		with open(path_to_conf_file) as f:
			parameters = yaml.full_load(f)
		print("Configuration parameters: ")
		for k, v in parameters.items():
			print(k, v)

		########## stereo camera parameters ##########
		self.fov=parameters["cameras"]["stereo"]["fov"]#122#cameras fov
		# self.fov=30.6             #cameras fov  argo
		self.baseline=parameters["cameras"]["stereo"]["baseline"]#0.24
		self.image_width=parameters["cameras"]["stereo"]["width"]#1600 
		self.image_height=parameters["cameras"]["stereo"]["height"]#1600

		self.back_image_width=parameters["cameras"]["mono"]["cam_back"]["width"]#1600 
		self.back_image_height=parameters["cameras"]["mono"]["cam_back"]["height"]#1600		# self.baseline=0.2986      #argo
		##############################################
		# print(self.fov, self.baseline, self.image_width, self.image_height)

		self.create_dataset_path=parameters["dataset"]["create_dataset_path"]#False
		self.create_dataset_images=parameters["dataset"]["create_dataset_images"]#False
		self.create_dataset_depth=parameters["dataset"]["create_dataset_depth"]#False
		self.create_occupancy_grid=parameters["dataset"]["create_occupancy_grid"]#False
		self.spectator=parameters["dataset"]["spectator"]#False
		self.create_dataset_obstacles=parameters["dataset"]["create_dataset_obstacles"]#False
		# print('create_dataset',self.create_dataset_path, self.create_dataset_images, self.create_dataset_depth, self.create_occupancy_grid, self.create_dataset_obstacles)

		# self.CREATE_DATASET=False  #---->
		self.track = Track.SENSORS
		track_env = os.environ.get('CHALLENGE_TRACK_CODENAME')
		if track_env=='SENSORS':
			self.track = Track.SENSORS
			self.lat_ref=0.0
			self.lon_ref=0.0
			self.datum=''
		elif track_env=='MAP':
			self.track = Track.MAP
			self.lat_ref=None
			self.lon_ref=None
			self.datum=None
		elif track_env=='SENSORS':
			self.track = Track.SENSORS_QUALIFIER
			self.lat_ref=0.0
			self.lon_ref=0.0
			self.datum=''
		elif track_env=='MAP':
			self.track = Track.MAP_QUALIFIER
			self.lat_ref=None
			self.lon_ref=None
			self.datum=None
		elif track_env=='DATASET':
			self.track = Track.DATASET
			self.lat_ref=0
			self.lon_ref=0
			self.datum=''
			# self.CREATE_DATASET=True  #---->

		print(track_env)
		print(self.track)
		
		# self.client = roslibpy.Ros(host='localhost', port=9090)
		# self.client.run(100)

		# self.shutdown_pub = roslibpy.Topic(self.client, '/carina/vehicle/shutdown', 'std_msgs/Bool')

	def sensors(self):
		
		sensors = []

		if not (self.track == Track.SENSORS  or self.track == Track.SENSORS_QUALIFIER): #self.track == Track.DATASET:
			OpenDRIVE={'type': 'sensor.opendrive_map', 'reading_frequency': 1, 'id': 'OpenDRIVE'}
			sensors.append(OpenDRIVE)

		CameraLeft={'type': 'sensor.camera.rgb', 'x':0, 'y':0.0, 'z':2.8, 'roll':0.0, 'pitch':0.0, 'yaw':0.0,
						'width':self.image_width, 'height': self.image_height, 'fov':self.fov, 'sensor_tick': 0.0, 'id': 'CameraLeft'}
		sensors.append(CameraLeft)	

		CameraRight={'type': 'sensor.camera.rgb', 'x':0., 'y':self.baseline, 'z':2.8, 'roll':0.0, 'pitch':0.0, 'yaw':0.0,
						'width':self.image_width, 'height': self.image_height, 'fov':self.fov, 'sensor_tick': 0.0, 'id': 'CameraRight'}
		sensors.append(CameraRight)

		CameraBack={'type': 'sensor.camera.rgb', 'x':0., 'y':self.baseline, 'z':2.8, 'roll':0.0, 'pitch':0.0, 'yaw':-np.pi,
						'width': self.back_image_width, 'height': self.back_image_height, 'fov':self.fov, 'sensor_tick': 0.0, 'id': 'CameraBack'}
		sensors.append(CameraBack)

		LIDAR={'type': 'sensor.lidar.ray_cast', 'x': 0., 'y': 0.0, 'z': 2.25, 'roll': 0.0, 'pitch': 0.0,
						'yaw': 0.0, 'channels':64,'points_per_second':350000, 'upper_fov':5.0,
						 'lower_fov':-20.0, 'range': 10000, 'sensor_tick': 0.0, 'rotation_frequency':30.0, 'id': 'LIDAR'}
		sensors.append(LIDAR)

		SPEED={'type': 'sensor.speedometer',  'reading_frequency': 20, 'id': 'SPEED'}   
		sensors.append(SPEED)		                          

		IMU={'type': 'sensor.other.imu', 'x': 0.0, 'y': 0.0, 'z': 0.0, 'roll': 0.0, 'pitch': 0.0, 'yaw': 0.0, 'id': 'IMU'}
		sensors.append(	IMU)	

		GPS={'type': 'sensor.other.gnss', 'x': 0.0, 'y': 0.0, 'z': 0.0, 'reading_frequency': 20, 'id': 'GPS'}
		sensors.append(GPS)	


		# if self.track == Track.DATASET:
		if not (self.track == Track.MAP or self.track == Track.SENSORS  or self.track == Track.MAP_QUALIFIER  or self.track == Track.SENSORS_QUALIFIER):
			if self.create_dataset_depth==True:
				# self.fov=30.6
				depthCameraLeft={'type': 'sensor.camera.depth', 'x':0., 'y':0.0, 'z':2.8, 'roll':0.0, 'pitch':0.0, 'yaw':0.0,
								'width':self.image_width, 'height': self.image_height, 'fov':self.fov, 'sensor_tick': 0.0, 'id': 'depthCameraLeft'}
				sensors.append(depthCameraLeft)	

				depthCameraRight={'type': 'sensor.camera.depth', 'x':0., 'y':self.baseline, 'z':2.8, 'roll':0.0, 'pitch':0.0, 'yaw':0.0,
								'width':self.image_width, 'height': self.image_height, 'fov':self.fov, 'sensor_tick': 0.0, 'id': 'depthCameraRight'}
				sensors.append(depthCameraRight)


			if self.create_dataset_images==True:
				CameraSemantic={'type': 'sensor.camera.semantic_segmentation', 'x':0., 'y':0.0, 'z':2.8, 'roll':0.0, 'pitch':0.0, 'yaw':0.0,
							'width':self.image_width, 'height': self.image_height, 'fov':self.fov, 'sensor_tick': 0.0, 'id': 'CameraSemantic'}
				sensors.append(CameraSemantic)

				CameraInstances={'type': 'sensor.camera.instance_segmentation', 'x':0., 'y':0.0, 'z':2.8, 'roll':0.0, 'pitch':0.0, 'yaw':0.0,
							'width':self.image_width, 'height': self.image_height, 'fov':self.fov, 'sensor_tick': 0.0, 'id': 'CameraInstances'}
				sensors.append(CameraInstances)

			if self.spectator:

				CameraSpectator={'type': 'sensor.camera.rgb', 'x':0.0, 'y':0.0, 'z':130., 'roll':0.0, 'pitch':-np.pi/2, 'yaw':0.0,
							'width':1400, 'height': 1400, 'fov':130.0, 'sensor_tick': 0.0, 'id': 'CameraSpectator'}
				sensors.append(CameraSpectator)

			if self.create_occupancy_grid:

				CameraAerial={'type': 'sensor.camera.rgb', 'x':((87.5/3)/2), 'y':0.0, 'z':87.50/2, 'roll':0.0, 'pitch':-np.pi/2, 'yaw':0.0,
							'width':700, 'height': 700, 'fov':90.0, 'sensor_tick': 0.0, 'id': 'CameraAerial'}
				sensors.append(CameraAerial)

				CameraAerialSeg={'type': 'sensor.camera.semantic_segmentation', 'x':((87.5/3)/2), 'y':0.0, 'z':87.50/2, 'roll':0.0, 'pitch':-np.pi/2, 'yaw':0.0,
							'width':700, 'height': 700, 'fov':90.0, 'sensor_tick': 0.0, 'id': 'CameraAerialSeg'}
				sensors.append(CameraAerialSeg)


			if self.create_dataset_path==True:
				gps_gt={'type': 'sensor.other.gnss', 'x': 0.0, 'y': 0.0, 'z': 0.0, 'reading_frequency': 20, 'id': 'gps_gt'}
				sensors.append(gps_gt)

				imu_gt={'type': 'sensor.other.imu', 'x': 0.0, 'y': 0.0, 'z': 0.0, 'roll': 0.0, 'pitch': 0.0, 'yaw': 0.0, 'id': 'imu_gt'}
				sensors.append(imu_gt)

			if self.create_dataset_obstacles==True:
				Markers={'type': 'sensor.pseudo.markers',   'x': 0., 'y': 0.0, 'z': 2.25, 'roll': 0.0, 'pitch': 0.0,
								'yaw': 0.0,'reading_frequency': 20, 'id': 'Markers'}   
				sensors.append(Markers)

				Objects={'type': 'sensor.pseudo.objects',   'x': 0., 'y': 0.0, 'z': 2.25, 'roll': 0.0, 'pitch': 0.0,
								'yaw': 0.0,'reading_frequency': 20, 'id': 'Objects'}   
				sensors.append(Objects)	

				actor_list_bjects={'type': 'sensor.pseudo.actor_list',   'x': 0., 'y': 0.0, 'z': 2.25, 'roll': 0.0, 'pitch': 0.0,
								'yaw': 0.0,'reading_frequency': 20, 'id': 'actor_list_bjects'}   
				sensors.append(actor_list_bjects)	
	
				tfl={'type': 'sensor.pseudo.traffic_lights',   'x': 0., 'y': 0.0, 'z': 2.25, 'roll': 0.0, 'pitch': 0.0,
								'yaw': 0.0,'reading_frequency': 20, 'id': 'traffic_lights'}   
				sensors.append(tfl)
		# print(sensors)
		return sensors

	# def run_step(self, input_data, timestamp):
	# 	pass
	# 	# return control


	# def destroy(self):
	# 	# signal_shutdown = Bool()
	# 	# signal_shutdown.data = True
	# 	# self.shutdown_pub.publish(signal_shutdown)


	# 	self.shutdown_pub.publish(roslibpy.Message({'data': True}))
	# 	print('Sending shutdown_pub...')
	# 	# time.sleep(1)
	# 	# self.shutdown_pub.unadvertise()
	# 	# self.client.terminate()


	# 	print('Is ROS connected?', self.client.is_connected)
	# 	if self.client.is_connected:
	# 		# self.client.terminate()
	# 		self.client.close()
	# 	print('Is ROS connected?', self.client.is_connected)

