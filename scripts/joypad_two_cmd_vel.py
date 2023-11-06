#!/usr/bin/python3 
'''
   Sylvain BERTRAND, 2023

   Publish Two Twists with linear from a unique joypad inputs
   (all variables in SI unit)

'''

import rospy
import numpy as np
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Joy


# node init
# --------------
rospy.init_node('joypad_two_cmd_vel', anonymous=False)



# parameters
# ------------
Vmax = rospy.get_param('Vmax',0.50) # maximum speed (m/s)
Vmin = -Vmax
OmegaMax = rospy.get_param('OmegaMax',1.57) # maximum angular speed (rad/s)
OmegaMin = -OmegaMax


# global variables
# -----------------
buttons = None
cmdVelMsg1 = Twist()
cmdVelMsg2 = Twist()


# node frequency
# -----------------
frequency = 10.0
Ts = 1.0/frequency
cmdPubRate = rospy.Rate(frequency)


# publishers
# ----------------
pubCmdVel1 = rospy.Publisher('cmd_vel1', Twist, queue_size=10)
pubCmdVel2 = rospy.Publisher('cmd_vel2', Twist, queue_size=10)


# -----------------------------------------------------------------------------
def callBackJoy(data):
# -----------------------------------------------------------------------------
	global cmdVelMsg1, cmdVelMsg2

	V1 = Vmax * data.axes[1]
	V2 = Vmax * data.axes[4]	
	Omega1 = OmegaMax * data.axes[0]
	Omega2 = OmegaMax * data.axes[3]
	
	# apply saturations
	V1 = np.fmax( np.fmin(V1, Vmax) , Vmin)
	V2 = np.fmax( np.fmin(V2, Vmax) , Vmin)
	Omega1 = np.fmax( np.fmin(Omega1, OmegaMax) , OmegaMin)
	Omega2 = np.fmax( np.fmin(Omega2, OmegaMax) , OmegaMin)

	cmdVelMsg1.linear.x = V1
	cmdVelMsg1.angular.z = 0.0#Omega1

	cmdVelMsg2.linear.x = V2
	cmdVelMsg2.angular.z = 0.0#Omega2

	'''
	global buttons
	for i in range(0, len(data.buttons)):
		if buttons == None or data.buttons[i] != buttons[i]:
			if i == 0 and data.buttons[i] == 1: # and flag_land != None:
				mas_control_enabled = False
			if i == 1 and data.buttons[i] == 1:  ##emergency
				mas_control_enabled = False
			if i == 2 and data.buttons[i] == 1: # and flag_takeoff != None:
				mas_control_enabled = False
						
        buttons = data.buttons 
	'''

#-----------------------------------------------------------------------------




# subscribers
# ------------
rospy.Subscriber("joy", Joy, callBackJoy)



# main node loop
# ---------------

# -----------------------------------------------------------------------------
if __name__ == '__main__':
# -----------------------------------------------------------------------------
	while not rospy.is_shutdown():
		# msg publication
		pubCmdVel1.publish(cmdVelMsg1)
		pubCmdVel2.publish(cmdVelMsg2)
		cmdPubRate.sleep()
# -----------------------------------------------------------------------------
