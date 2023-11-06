#!/usr/bin/python3

from __future__ import print_function

import threading

#import roslib; roslib.load_manifest('teleop_keyboard')
import rospy

from geometry_msgs.msg import Twist

import sys
from select import select

if sys.platform == 'win32':
    import msvcrt
else:
    import termios
    import tty


#TwistMsg = Twist()
moveBindings=dict()
speedBindings=dict()
turnBindings=dict()
T1X = 0
T2X = 0


def print_msg(front,left,right,back,stop,speed_inc,speed_dec,ns,turn_ccw,turn_cw):
    msg ="""\n
---------------------------
Moving around {7}:
    {5}   {0}   {6}
    {1}   {4}   {2}  
    {8}   {3}   {9}
Increase/Decrease max speed by 0.5 in X-Y-Theta all,
Keeps less than speed limit
---------------------------
{7}:
    {5} : up (+)  {6} : down (-)  {4} : stop (0)
    {8} : turn CCW  {9} : turn CW
anything else : stop
CTRL-C to quit
""".format(front,left,right,back,stop,speed_inc,speed_dec,ns,turn_ccw,turn_cw)
    rospy.loginfo_once(msg)

def bind_keys(front,left,right,back,stop,speed_inc,speed_dec,turn_ccw,turn_cw):
    global moveBindings, speedBindings, turnBindings
    moveBindings={
            front:(0.01,0),
            left:(0,0.01),
            right:(0,-0.01),
            back:(-0.01,0),
        }

    turnBindings={
        turn_ccw: 0.01,
        turn_cw: -0.01,
    }

    speedBindings={
            speed_inc: 0.5,
            speed_dec: -0.5,
        }

class PublishThread(threading.Thread):
    def __init__(self, rate, ns):
        super(PublishThread, self).__init__()
        self.publisher = rospy.Publisher('/'+ns+'/cmd_vel', Twist, queue_size = 10)
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        self.speed = 0.0
        self.condition = threading.Condition()
        self.done = False

        # Set timeout to None if rate is 0 (causes new_message to wait forever
        # for new data to publish)
        if rate != 0.0:
            self.timeout = 1.0 / rate
        else:
            self.timeout = None

        self.start()

    def wait_for_subscribers(self):
        i = 0
        while not rospy.is_shutdown() and self.publisher.get_num_connections() == 0:
            if i == 4:
                rospy.loginfo_once("Waiting for subscriber to connect to {}".format(self.publisher.name))
            rospy.sleep(0.5)
            i += 1
            i = i % 5
        if rospy.is_shutdown():
            raise Exception("Got shutdown request before subscribers connected")

    def update(self, x, y, theta, speed):
        self.condition.acquire()
        self.x = x
        self.y = y
        self.theta = theta
        self.speed = speed

        # Notify publish thread that we have a new message.
        self.condition.notify()
        self.condition.release()

    def stop(self):
        self.done = True
        self.update(0, 0, 0, 0)
        self.join()

    def run(self):
        twist = Twist()

        while not self.done:

            self.condition.acquire()
            # Wait for a new message or timeout.
            self.condition.wait(self.timeout)

            # Copy state into twist message.
            twist.linear.x = self.x 
            twist.linear.y = self.y 
            twist.linear.z = 0
            twist.angular.x = 0
            twist.angular.y = 0
            twist.angular.z = self.theta

            self.condition.release()

            # Publish.
            self.publisher.publish(twist)

        # Publish stop message when thread exits.
        twist.linear.x = 0
        twist.linear.y = 0
        twist.linear.z = 0
        twist.angular.x = 0
        twist.angular.y = 0
        twist.angular.z = 0
        self.publisher.publish(twist)


def getKey(settings, timeout):
    if sys.platform == 'win32':
        # getwch() returns a string on Windows
        key = msvcrt.getwch()
    else:
        tty.setraw(sys.stdin.fileno())
        # sys.stdin.read() returns a string on Linux
        rlist, _, _ = select([sys.stdin], [], [], timeout)
        if rlist:
            key = sys.stdin.read(1)
        else:
            key = ''
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key

def saveTerminalSettings():
    if sys.platform == 'win32':
        return None
    return termios.tcgetattr(sys.stdin)

def restoreTerminalSettings(old_settings):
    if sys.platform == 'win32':
        return
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_settings)

def vels(speed):
    return "currently:\tspeed %s" % (speed)

def joyCallback(data):
    global T1X,T2X
    T1X = data.axes[1]
    T2X = data.axes[4]

if __name__=="__main__":
    settings = saveTerminalSettings()

    rospy.init_node('teleop_keyboard', anonymous=False)
    rate = rospy.Rate(10) # ROS Rate at 10Hz
    speed = 1.5
    use_joy = rospy.get_param("use_joy",False)
    front = str(rospy.get_param("~front"))
    left = str(rospy.get_param("~left"))
    right = str(rospy.get_param("~right"))
    back = str(rospy.get_param("~back"))
    stop = str(rospy.get_param("~stop",'s'))
    speed_inc = str(rospy.get_param("~speed_inc"))
    speed_dec = str(rospy.get_param("~speed_dec"))
    turn_ccw = str(rospy.get_param("~turn_ccw"))
    turn_cw = str(rospy.get_param("~turn_cw"))
    speed_limit = rospy.get_param("~speed_lim", 2.0)
    
    repeat = rospy.get_param("~repeat_rate", 0.0)
    key_timeout = rospy.get_param("~key_timeout", 0.5)

    ns=str(rospy.get_namespace().strip('/'))
    if use_joy:
        from sensor_msgs.msg import Joy
        sub = rospy.Subscriber("/joy",Joy,joyCallback)

    pub_thread = PublishThread(repeat,ns)
    rate.sleep()

    x = 0
    y = 0
    theta = 0
    status = 0

    try:
        bind_keys(front,left,right,back,stop,speed_inc,speed_dec,turn_ccw,turn_cw)
        if not use_joy:
            print_msg(front,left,right,back,stop,speed_inc,speed_dec,ns,turn_ccw,turn_cw)
        pub_thread.wait_for_subscribers()
        rospy.loginfo_once("Subscriber connected {}".format(ns))
        pub_thread.update(x, y, theta, speed)

        print(vels(speed))
        while(1):
            if not use_joy:
                key = getKey(settings, key_timeout)
                #key='1'
                if key in moveBindings.keys():
                    x = min(speed, x+moveBindings[key][0]) if x>0 else max(-speed, x+moveBindings[key][0])
                    y = min(speed, y+moveBindings[key][1]) if y>0 else max(-speed, y+moveBindings[key][1])
                elif key in turnBindings.keys():
                    theta = min(speed, theta+turnBindings[key]) if theta>0 else max(-speed, theta+turnBindings[key])
                elif key in speedBindings.keys():
                    speed = min(abs(speed_limit), abs(speed + speedBindings[key]))
                    if speed == speed_limit:
                        print("Linear speed limit reached!")
                    print(vels(speed))
                    #if (status == 14):
                    #    print_msg(front,left,right,back,stop,speed_inc,speed_dec,ns)
                    #status = (status + 1) % 15
                else:
                    # Skip updating cmd_vel if key timeout and robot already
                    # stopped.
                    if key == '' and x == 0 and y == 0 and theta == 0:
                        continue
                    x = 0
                    y = 0
                    theta = 0
                    if (key == '\x03'):
                        break

            pub_thread.update(x, y, theta, speed)

    except Exception as e:
        print(e)

    finally:
        pub_thread.stop()
        restoreTerminalSettings(settings)