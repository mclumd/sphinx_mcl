#!/usr/bin/env python
import rospy
import os
import re
from std_msgs.msg import String
from ar_track_alvar_msgs.msg import AlvarMarker, AlvarMarkers
from geometry_msgs.msg import Point
from geometry_msgs.msg import PointStamped
from std_msgs.msg import String


pub_pos = rospy.Publisher('/julia_loc', Point, queue_size=10)

def tag_callback(data):
    global pub_pos
    for marker in data.markers:
	if marker.id==10 or marker.id==12 or marker.id==20:
            juliaPos = Point() 
            juliaPos.x = marker.pose.pose.position.x
            juliaPos.y = marker.pose.pose.position.y
            juliaPos.z = marker.pose.pose.position.z
            pub_pos.publish(juliaPos)
            #rospy.sleep(0)



if __name__ == "__main__":
    rospy.init_node('julia_pose')
    rospy.Subscriber("/ar_pose_marker", AlvarMarkers, tag_callback)
    rospy.spin()
