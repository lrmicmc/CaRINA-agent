#!/usr/bin/env python3
from collision_detection import *

if __name__ == '__main__':
	rospy.init_node("collision_detection_node",anonymous=True)
	print("[CollisionDetection node] running...")
	repl = CollisionDetection()
	rospy.spin()