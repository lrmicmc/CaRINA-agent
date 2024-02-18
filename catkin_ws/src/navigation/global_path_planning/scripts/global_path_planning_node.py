#!/usr/bin/env python3

import rospy
from global_path_planning import *


if __name__ == '__main__':
    print ("[Global Path Planning Node] running...")

    rospy.init_node("global_path_planning_node", anonymous=True)

    global_planning = GlobalPathPlanning()
    rospy.spin()
