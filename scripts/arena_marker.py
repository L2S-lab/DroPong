#!/usr/bin/env python3
'''

   Sylvain BERTRAND, 2023
   (all variables in SI unit)
   
   Publish markers for simulation of Pong game
   
'''

import rospy
from visualization_msgs.msg import Marker



# node init
# --------------
rospy.init_node('arena_marker', anonymous=False)


# node frequency
# -----------------
frequency = 10.0
Ts = 1.0/frequency
markersPubRate = rospy.Rate(frequency)



marker = Marker()
marker.header.frame_id = "world" #frame_id
marker.ns = "arena"
marker.header.stamp = rospy.Time.now()
marker.id = 0
marker.type = Marker.CUBE
marker.action = 0  #ADD
marker.pose.orientation.x = 0.0
marker.pose.orientation.y = 0.0
marker.pose.orientation.z = 0.0
marker.pose.orientation.w = 1.0
marker.pose.position.x = 0
marker.pose.position.y = 0
marker.pose.position.z = 0
marker.color.r = 0
marker.color.g = 0
marker.color.b = 1
marker.color.a = 0.1
marker.scale.x = 2
marker.scale.y = 4
marker.scale.z = 0.02

pubArenaMarker = rospy.Publisher('/arena_marker', Marker, queue_size=10)



# main node loop
# ---------------

# -----------------------------------------------------------------------------
if __name__ == '__main__':
# -----------------------------------------------------------------------------
	while not rospy.is_shutdown():
		marker.header.stamp = rospy.Time.now()
		# msg publication
		pubArenaMarker.publish(marker)
		markersPubRate.sleep()
# -----------------------------------------------------------------------------

