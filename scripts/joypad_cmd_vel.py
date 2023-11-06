#!/usr/bin/python3 
'''
   Sylvain BERTRAND, 2022

   Publish Twist from joypad inputs
   (all variables in SI unit)

'''

import rospy
import numpy as np
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Joy


# node init
# --------------
rospy.init_node('joypad_cmd_vel', anonymous=False)



# parameters
# ------------
Vmax = rospy.get_param('Vmax',0.50) # maximum speed (m/s)
Vmin = -Vmax
OmegaMax = rospy.get_param('OmegaMax',1.57) # maximum angular speed (rad/s)
OmegaMin = -OmegaMax


# global variables
# -----------------
buttons = None
cmdVelMsg = Twist()


# node frequency
# -----------------
frequency = 10.0
Ts = 1.0/frequency
cmdPubRate = rospy.Rate(frequency)


# publishers
# ----------------
pubCmdVel = rospy.Publisher('cmd_vel', Twist, queue_size=10)


# -----------------------------------------------------------------------------
def callBackJoy(data):
# -----------------------------------------------------------------------------
	global cmdVelMsg

	V = Vmax * data.axes[1]	
	Omega = OmegaMax * data.axes[3]

	# apply saturations
	V = np.fmax( np.fmin(V, Vmax) , Vmin)
	Omega = np.fmax( np.fmin(Omega, OmegaMax) , OmegaMin)

	cmdVelMsg.linear.x = V
	cmdVelMsg.angular.z = Omega

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
		pubCmdVel.publish(cmdVelMsg)
		cmdPubRate.sleep()
# -----------------------------------------------------------------------------
