#!/usr/bin/env python3
import numpy as np
import message_filters
from sensor_msgs.msg import Image, CameraInfo
# from sensor_msgs.msg import Image
import rospy
from cv_bridge import CvBridge, CvBridgeError
import cv2

class SGM(object):

	def __init__ (self):

		# self.timeout_emergency_stop = 90

		# self.akermamn_sub=rospy.Subscriber("/carina/control/ackermann_cmd", AckermannDriveStamped, self.ackermann_cmd_callback)
		# self.throttle_pub = rospy.Publisher( '/carina/control/throttle_cmd', Throttle, queue_size=1)


		self.image_left_sub = message_filters.Subscriber( '/carina/sensor/camera/left/image_raw', Image)
		self.image_right_sub = message_filters.Subscriber('/carina/sensor/camera/right/image_raw', Image)

		self.info_left_sub = message_filters.Subscriber( '/carina/sensor/camera/left/camera_info', CameraInfo)
		self.info_right_sub = message_filters.Subscriber('/carina/sensor/camera/right/camera_info', CameraInfo)

		self.ts = message_filters.ApproximateTimeSynchronizer([self.image_left_sub, self.image_right_sub, self.info_left_sub, self.info_right_sub], 10, 0.2, allow_headerless=True)

		self.ts.registerCallback(self.callback_stereo_images)

		self.cvbridge = CvBridge()














	def callback_stereo_images(self, left_image, right_image, left_camera_info, right_camera_info):
		try:
			img0 = self.cvbridge.imgmsg_to_cv2(left_image, "bgr8")
		except CvBridgeError as e:
			print (e)

		try:
			img1 = self.cvbridge.imgmsg_to_cv2(right_image, "bgr8")
		except CvBridgeError as e:
			print (e)

		left_camera_info_K = np.array(left_camera_info.K).reshape([3, 3])
		left_camera_info_D = np.array(left_camera_info.D)
		# left_rgb_undist = cv2.undistort(img0, left_camera_info_K, left_camera_info_D)



		# Defining the SGM parameters (please check the OpenCV documentation for details).
		# We found this parameters empirically and based on the Argoverse Stereo data. 
		max_disp = 192
		win_size = 10
		uniqueness_ratio = 15
		speckle_window_size = 200
		speckle_range = 2
		block_size = 11
		P1 = 8 * 3 * win_size ** 2
		P2 = 32 * 3 * win_size ** 2

		# Defining the Weighted Least Squares (WLS) filter parameters.
		lmbda = 0.1
		sigma = 1.0

		# Defining the SGM left matcher.
		left_matcher = cv2.StereoSGBM_create(
		    minDisparity=0,
		    numDisparities=max_disp,
		    blockSize=block_size,
		    P1=P1,
		    P2=P2,
		    disp12MaxDiff=max_disp,
		    uniquenessRatio=uniqueness_ratio,
		    speckleWindowSize=speckle_window_size,
		    speckleRange=speckle_range,
		    mode=cv2.STEREO_SGBM_MODE_SGBM_3WAY,
		)

		# Defining the SGM right matcher needed for the left-right consistency check in the WLS filter.
		right_matcher = cv2.ximgproc.createRightMatcher(left_matcher)

		# Defining the WLS filter.
		wls_filter = cv2.ximgproc.createDisparityWLSFilter(matcher_left=left_matcher)
		wls_filter.setLambda(lmbda)
		wls_filter.setSigmaColor(sigma)

		# Computing the disparity maps.
		left_disparity = left_matcher.compute(img0, img1)
		right_disparity = right_matcher.compute(img1, img0)

		# Applying the WLS filter.
		left_disparity_pred = wls_filter.filter(left_disparity, img0, None, right_disparity)

		# Recovering the disparity map.
		# OpenCV produces a disparity map as a signed short obtained by multiplying subpixel shifts with 16.
		# To recover the true disparity values, we need to divide the output by 16 and convert to float.
		left_disparity_pred = np.float32(left_disparity_pred) / 16.0

		# OpenCV will also set negative values for invalid disparities where matches could not be found.
		# Here we set all invalid disparities to zero.
		left_disparity_pred[left_disparity_pred < 0] = 0

		print("stereo")



if __name__ == '__main__':
	print ("[SGM Node] running...")
	rospy.init_node("stereo_sgm", anonymous=True)
	sgm = SGM()
	rospy.spin()