#!/usr/bin/env python

#ros
import rospy

import numpy as np
import time

from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped

if __name__ == '__main__':
	
	rospy.init_node('test', anonymous=True)
	pub = rospy.Publisher('/carina/navigation/waypoints', Path, queue_size=1)

	way = np.load('waypoints.npy')

	path = Path()
	path.header.stamp = rospy.Time().now()
	path.poses = []

	print "creating message"
	for w in way:
		p = PoseStamped()
		p.pose.position.x = w[0]
		p.pose.position.y = w[1]
		p.pose.position.z = w[2]

		path.poses.append(p)

	time.sleep(2)
	pub.publish(path)

	print "done!"
	rospy.spin()



