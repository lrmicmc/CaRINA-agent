#!/usr/bin/env python3

import rospy
from global_plan_monitor import *


if __name__ == '__main__':
    print ("[Global Plan Monitor] running...")

    rospy.init_node("global_plan_monitor_node", anonymous=True)

    global_plan_monitor = GlobalPlanMonitor()
    rospy.spin()