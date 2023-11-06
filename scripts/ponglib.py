#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist, PoseStamped

import pong_algo as pong_algo
import time

global robot1_pose, robot2_pose, drone_pose, simu, pubVel

robot1_pose = [0.,0.,0.]
robot2_pose = [0.,0.,0.]
drone_pose = [0.,0.,0.]

def goto(x_ref,y_ref,z_ref):
    global pubVel
    epsilon_prec = 0.0
    filtered_derivative = 0.0
    freq = 30
    Ts = 1.0/freq  
    commandRate = rospy.Rate(freq) 
    integralX = 0.0
    integralY = 0.0
    integralZ = 0.0
    vel_msg = Twist()
    curr_t = time.time()
    while time.time()-curr_t < 10:
        x = drone_pose[0]
        y = drone_pose[1]
        z = drone_pose[2]

        epsilonX = x-x_ref
        epsilonY = y-y_ref
        epsilonZ = z-z_ref

        if epsilonX <0.1 and epsilonY<0.1 and epsilonZ<0.1:
            return True
        integralX = integralX + Ts*epsilonX
        integralY = integralY + Ts*epsilonY
        integralZ = integralZ + Ts*epsilonZ

        vel_msg.linear.x = 0.0
        vel_msg.linear.y = 0.0
        vel_msg.linear.z = 0.0

        vel_msg.linear.x = -0.8*(epsilonX) #- 0.1*integralX       # ok parfait
        vel_msg.linear.y = -0.8*(epsilonY) #- 0.1*integralY       # ok parfait
        vel_msg.linear.z = -1.2*(epsilonZ) - 0.1*integralZ       # ok parfait

        # Command sending
        pubVel.publish(vel_msg)
        commandRate.sleep()
    return False

def callBackRobot1Pose(data):
    global robot1_pose
    robot1_pose[0]=data.pose.position.x
    robot1_pose[1]=data.pose.position.y
    robot1_pose[2]=data.pose.position.z

def callBackRobot2Pose(data):
    global robot2_pose
    robot2_pose[0]=data.pose.position.x
    robot2_pose[1]=data.pose.position.y
    robot2_pose[2]=data.pose.position.z

def callBackDronePose(data):
    global drone_pose
    drone_pose[0]=data.pose.position.x
    drone_pose[1]=data.pose.position.y
    drone_pose[2]=data.pose.position.z

def subs(simu):
    global pubVel
    if simu:
        rospy.Subscriber("/robot_1/pose", PoseStamped, callBackRobot1Pose)
        rospy.Subscriber("/robot_2/pose", PoseStamped, callBackRobot2Pose)
        rospy.Subscriber("/drone/pose", PoseStamped, callBackDronePose)
        pubVel = rospy.Publisher('/drone/cmd_vel', Twist, queue_size=10)

    else:
        rospy.Subscriber("/natnet_ros/s1_1/pose", PoseStamped, callBackRobot1Pose)
        rospy.Subscriber("/natnet_ros/s1_2/pose", PoseStamped, callBackRobot2Pose)
        rospy.Subscriber("/natnet_ros/rmtt_1/pose", PoseStamped, callBackDronePose)
        pubVel = rospy.Publisher('/cmd_vel', Twist, queue_size=10)

def main():
    global robot1_pose, robot2_pose, drone_pose, pubVel, simu
    freq = 30.0
    Ts = 1/freq
    rate = rospy.Rate(freq)
    vel_msg = Twist()
    simu = rospy.get_param("simu",False)
    subs(simu)
    #rospy.Rate(0.1).sleep()
    while not rospy.is_shutdown():
        
        #rospy.loginfo("%s",drone_pose)
        vel_msg.linear.x,vel_msg.linear.y,vel_msg.linear.z,vel_msg.angular.z, reset = pong_algo.controller(
                                                                drone_pose,robot1_pose,robot2_pose)

        #if reset:
        #    done = goto(0,0,1)
        #    rospy.loginfo("RESET: ",done)

        pubVel.publish(vel_msg)
        rate.sleep()


if __name__ == '__main__':
    try:
        rospy.init_node('ponglib', anonymous=True)
        main()
    except:
        print("Something went wrong")
