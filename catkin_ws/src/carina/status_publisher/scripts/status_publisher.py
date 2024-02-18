#!/usr/bin/env python3
import rospy
from std_msgs.msg import  Bool, String
# from carla_msgs.msg import CarlaEgoVehicleControl
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped

# status_pub = rospy.Publisher('carla/hero/status', Bool, queue_size=10)

# def opendrive_callback(data):
#     rospy.loginfo("I string opendrive %s",data.data)
    # status_signal = Bool()
    # status_signal.data = True
    # status_pub.publish(status_signal)
class StatusPublisher(object):

    def __init__ (self):

        # print('Setup')
        self.status_pub = rospy.Publisher('carla/hero/status', Bool, queue_size=10)
        self.current_pose_sub = rospy.Subscriber('/carina/localization/pose', PoseWithCovarianceStamped, self.pose_cb, queue_size=1)

        # rospy.Timer(rospy.Duration(1), self.publish_status)
        while not rospy.is_shutdown():
            # rospy.loginfo(rospy.Time().now())
            # rospy.loginfo(rospy.Time().now()).to_sec()
            self.publish_status()


    def pose_cb(self, msg):
        # print('pose received')
        self.current_pose = msg
        # print ("Bye!")
        rospy.signal_shutdown("finished status publisher")
   
    # def publish_tf(self):
    # def publish_status(self, event):
    def publish_status(self):

# def status_pub():

    # status_pub = rospy.Publisher('carla/hero/status', Bool, queue_size=10)
    # opendrive=rospy.Subscriber('/carla/hero/OpenDRIVE', String, opendrive_callback)
    # status_verif_pub = rospy.Publisher('carla/hero/status_verif', Bool, queue_size=10)
    # control_pub= rospy.Publisher('/carla/hero/vehicle_control_cmd',CarlaEgoVehicleControl, queue_size=10)
        # rospy.init_node('status_publisher', anonymous=True)
        # rate = rospy.Rate(1) # 10hz

        # while not rospy.is_shutdown():
        # rospy.loginfo("status: True" )
        status_signal = Bool()
        status_signal.data = True
        self.status_pub.publish(status_signal)




        # status_verif_pub.publish(status_signal)
        # control=CarlaEgoVehicleControl()
        # control.
        # rospy.sleep(5)
        # rate.sleep()

# if __name__ == '__main__':
#     try:
#         status_pub()
#     except rospy.ROSInterruptException:
#         pass



if __name__ == '__main__':
    rospy.init_node("status_publisher_node",anonymous=True)
    print("[status_publisher_node] running...")
    repl = StatusPublisher()
    rospy.spin()
