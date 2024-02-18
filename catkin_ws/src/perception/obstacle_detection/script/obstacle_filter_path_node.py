#!/usr/bin/env python3
import rospy

from obstacle_filter_path import *


if __name__ == '__main__':
	rospy.init_node("obstacle_filter_path_node", anonymous=True)
	print ("[Filter Obstacle path Node] running...")
	obst = ObstacleFilterPath()

	rospy.spin()
