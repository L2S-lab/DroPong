import rospy
import random


global vx,vy, score1, score2, over
vx=0.2
vy=0.2
score1=0
score2=0
over= False
reset=False
def main(drone_pose:list(), robot1_pose:list(), robot2_pose:list()): 
#def main(drone_pose, robot1_pose, robot2_pose): 

    vz=0
    yaw_rate=0
    global vx,vy, score1, score2, over, reset
    #if reset:
    #    reset=False
    #--------------------------------------#

    epsilon = 0.2
    fond = 2.3
    ligne1 = 1.8
    border1 = 0.8
    hit1 = False
    hit2 = False

    if abs(drone_pose[1]-ligne1)<0.1: #Robot1 is the one on the line of +2meters
        if abs(robot1_pose[0]-drone_pose[0])<epsilon:
            hit1 = True    
    
    if abs(drone_pose[1]+ligne1)<0.1: #Robot2 is on the line -2 meters
        if abs(robot2_pose[0]-drone_pose[0])<epsilon:
            hit2 = True  

    if drone_pose[0]>= border1 and vx>0: #border : 1 meter
        r = random.uniform(0.8,1.25)
        vx = -vx*r
        vy = vy
    
    elif drone_pose[0]<= -border1 and vx<0: #border : -1 meter
        r = random.uniform(0.8,1.25)
        vx = -vx*r
        vy = vy

    elif drone_pose[1] >= fond and vy>0:
        rospy.loginfo_once("***** OUT !  +1 Pt for Player 2 *****")
        score2+=1
        r = random.uniform(0.8,1.25)
        vx = vx 
        vy = -vy*r
    elif drone_pose[1]<= -fond and vy<0:
        rospy.loginfo_once("***** OUT !  +1 Pt for Player 1 *****")
        score1+=1 #Player 1 wins a point when the drone goes in the back of robot2
        r = random.uniform(0.8,1.25)
        vx = vx 
        vy = -vy*r
    
    elif hit1: 
        if vy>0:#Racket bounce
            r = random.uniform(0.8,1.25)
            vy = -vy*r
            vx = vx
        else: #the drone is behind the racket (player 2 just scored)
            pass

    elif hit2: 
        if vy<0:#Racket bounce
            r = random.uniform(0.8,1.25)
            vy = -vy*r
            vx = vx
        else: #the drone is behind the racket (player 1 just scored)
            pass

    else:
        pass  
    #--------------------------------------#
    #rospy.loginfo("vx:%s  vy:%s  vz:%s",vx,vy,vz)
    if not over:
        rospy.loginfo_throttle_identical(1,"Player1: %s      Player2: %s",score1,score2)
    if score1 ==3 or score2==3: #End of the game
        vx,vy,vz,yaw_rate=0,0,0,0
        over = True
        if score1==3:
            rospy.loginfo_once("\n=========GAME OVER=========\n++++++++++++++++++++++++++++++++++++++++++++++\n\tPlayer-1 is the winner\n++++++++++++++++++++++++++++++++++++++++++++++")
        if score2==3:
            rospy.loginfo_once("\n=========GAME OVER=========\n++++++++++++++++++++++++++++++++++++++++++++++\n\tPlayer-2 is the winner\n++++++++++++++++++++++++++++++++++++++++++++++")
    
        #rospy.loginfo("vx: %s      vy: %s",vx,vy)
    #if over:
    #    reset=True
    return vx,vy,vz,yaw_rate, reset



# ======== ! DO NOT MODIFY ! ============
def controller(drone_pose,robot1_pose,robot2_pose):
# =======================================
    vx,vy,vz,yaw_rate, reset = main(drone_pose,robot1_pose,robot2_pose)
    
    return  vx,vy,vz,yaw_rate, reset
    
# ====================================   
