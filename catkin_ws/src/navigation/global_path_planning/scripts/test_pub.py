import rospy
from msgs_mapping.msg import HDMap as HDMapMsg
import numpy as np
import matplotlib.pyplot as plt
from nav_msgs.msg import Path
from std_msgs.msg import Bool
from geometry_msgs.msg import PoseStamped
from msgs_mapping.msg import HDMap as HDMapMsg

import time

def main():
    hdmap_pub = rospy.Publisher('/carina/map/hdmap', HDMapMsg, queue_size=1)
    waypoints_pub = rospy.Publisher('/carina/navigation/waypoints', Path, queue_size=1)
    
    path_sub = rospy.Subscriber('/carina/navigation/path', Path, plot_cb, queue_size=1)
   
    time.sleep(2)
    print("publishing hd message")
    map_file = "/home/shanks/Documents/Projects/[Project][Carla]/maps/Town04.xodr"
    with open(map_file, 'r') as f:
        map_content = f.read()
    
    hd_msgs = HDMapMsg()
    hd_msgs.header.stamp =rospy.Time().now()
    hd_msgs.header.frame_id="map"
    hd_msgs.XML_HDMap = map_content
    
    hdmap_pub.publish(hd_msgs)
    
    time.sleep(2)
    print("publishing waypoints")
    way_file = "/home/shanks/Documents/Projects/[Project][Carla]/waypoints/Town04_0.npy"
    waypoints = np.load(way_file)    
    
    ros_path = Path()
    ros_path.header.stamp = rospy.Time().now()
    ros_path.header.frame_id = 'map'
    ros_path.poses = []
    for p in waypoints:
        ps = PoseStamped()
        ps.pose.position.x = p[0] 
        ps.pose.position.y = p[1] 
        #ps.pose.orientation.x = 0
        #ps.pose.orientation.y = 0
        #ps.pose.orientation.z = 0
        #ps.pose.orientation.w = 1
        ros_path.poses.append(ps)
    waypoints_pub.publish(ros_path)
    
def plot_cb(msg):
    
    print("Received Path")
    
    path = [[p.pose.position.x, p.pose.position.y] for p in msg.poses]
    path = np.array(path)
    
    plt.plot(path[:,0], path[:, 1], 'g.-')
    plt.show()
    
if __name__ == "__main__":
    
    print("hello")
    rospy.init_node("test_node", anonymous=True)
    main()
    
    rospy.spin()