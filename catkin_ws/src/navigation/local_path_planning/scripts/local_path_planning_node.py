#!/usr/bin/env python3

import rospy
from local_path_planning import *


if __name__ == '__main__':
    print ("[Local Path Planning Node] running...")

    rospy.init_node("local_path_planning_node", anonymous=True)

    local_planning = LocalPathPlanning()
    rospy.spin()
