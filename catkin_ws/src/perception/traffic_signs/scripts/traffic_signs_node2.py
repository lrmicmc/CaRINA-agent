#!/usr/bin/env python3


import rospy
from TrafficSignDetector2 import TrafficSignDetector


if __name__ == '__main__':
	print ("[Traffic Signs Detector 2 Node] running...")
	rospy.init_node('traffic_sign_detector_node2', anonymous=True)

	t_detector = TrafficSignDetector()

	rospy.spin()

