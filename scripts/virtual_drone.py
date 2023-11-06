#!/usr/bin/python3
'''
   Sylvain BERTRAND, 2023 

   Virtual drone (pose from single integrator dynamics) from Twist msg 
   (all variables in SI unit)

'''

import rospy
import math
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist, PoseStamped, Point, Vector3, Quaternion
import tf
#from tf.transformations import quaternion_from_euler

from visualization_msgs.msg import Marker


# node init
# --------------
rospy.init_node('virtual_drone', anonymous=True)


# node frequency
# -----------------
nodeFreq = 10.0
Ts = 1.0/nodeFreq
nodeRate = rospy.Rate(nodeFreq)


# parameters
# ------------

# ids of frames
frame_id = rospy.get_param('~frame_id', 'world')
child_frame_id = rospy.get_param('~child_frame_id', 'odom')

# initialize pose of robot
x = rospy.get_param('~x0',0.0) # initial position (x0,y0,z0) (m)
y = rospy.get_param('~y0',0.0)
z = rospy.get_param('~z0',0.0)
psi = rospy.get_param('~psi0',0.0) # initial yaw angle (psi0) (rad)


# global variables
# -----------------
vx = vy = vz = 0.0
omegax = omegay = omegaz = 0.0



# publishers
# ----------------
pubOdom = rospy.Publisher('odom', Odometry, queue_size=10)
pubPose = rospy.Publisher('pose', PoseStamped, queue_size=10)
pubMarker = rospy.Publisher('marker', Marker, queue_size=10)

msgOdom = Odometry()
msgPose = PoseStamped()


marker = Marker()
marker.header.frame_id = frame_id
marker.header.stamp = rospy.Time.now()
marker.ns = "drone"
marker.id = 0
marker.type = Marker.MESH_RESOURCE
marker.mesh_resource = "package://pong_robots/meshes/visual/dji_tello.dae"
marker.action = 0  #ADD
marker.pose.orientation.x = 0.0  #TO BE MODIFIED
marker.pose.orientation.y = 0.0
marker.pose.orientation.z = 0.0
marker.pose.orientation.w = 1.0
marker.scale.x = 1.0
marker.scale.y = 1.0
marker.scale.z = 1.0
marker.color.a = 0.7
marker.color.r = 1.0
marker.color.g = 0.0
marker.color.b = 0.0



# -----------------------------------------------------------------------------
def callBackTwist(data):
# -----------------------------------------------------------------------------
    global vx, vy, vz, omegax, omegay, omegaz
    vx = data.linear.x
    vy = data.linear.y
    vz = data.linear.z
    omegax = data.angular.x
    omegay = data.angular.y
    omegaz = data.angular.z
# -----------------------------------------------------------------------------


# subscribers
# ------------
rospy.Subscriber("cmd_vel", Twist, callBackTwist)


# main node loop
# ---------------
# -----------------------------------------------------------------------------
if __name__ == '__main__':
# -----------------------------------------------------------------------------
    
    tfbr = tf.TransformBroadcaster()
    
    while not rospy.is_shutdown():
        

        # update virtual robot pose
        x += Ts*vx
        y += Ts*vy
        z += Ts*vz
        psi += Ts*omegaz
        
        #str = "(x=%s  y=%s theta=%s)"%(x, y, theta)
        #rospy.loginfo(str)
        
        
        t = rospy.Time.now()
        
        q = tf.transformations.quaternion_from_euler(0.0, 0.0, psi)
        position = Point(x, y, z)
        quat = Quaternion(q[0], q[1], q[2], q[3])
        
        # fill messages
        # ----------------
        
        # pose
        msgPose.header.seq += 1
        msgPose.header.stamp = t
        msgPose.header.frame_id = frame_id
        msgPose.pose.position = position
        msgPose.pose.orientation = quat
        
        # odometry
        msgOdom.header.seq +=1 
        msgOdom.header.stamp = t
        msgOdom.header.frame_id = frame_id
        msgOdom.child_frame_id = child_frame_id
        msgOdom.pose.pose.position = position
        msgOdom.pose.pose.orientation = quat
        msgOdom.twist.twist.linear = Vector3(vx, vy, vz)
        msgOdom.twist.twist.angular = Vector3(0.0, 0.0, omegaz)

        # marker
        marker.header.stamp = t
        marker.pose.position = position
        marker.pose.orientation = quat
        
        
        # msgs publication
        pubPose.publish(msgPose)
        pubOdom.publish(msgOdom)
        pubMarker.publish(marker)
    
        # tf broadcast
        tfbr.sendTransform( (x, y, z), q, t, child_frame_id, frame_id)
    
        
        # wait one step
        nodeRate.sleep()
# -----------------------------------------------------------------------------

