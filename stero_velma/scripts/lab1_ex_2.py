#!/usr/bin/env python

import roslib; roslib.load_manifest('velma_task_cs_ros_interface')

import rospy
import copy
import PyKDL

from velma_common import *
from rcprg_ros_utils import exitError
from control_msgs.msg import FollowJointTrajectoryResult


# zero pos of torso
q_map_0 = {'torso_0_joint':0, 'right_arm_0_joint':-0.3, 'right_arm_1_joint':-1.8,
     'right_arm_2_joint':1.25, 'right_arm_3_joint':0.85, 'right_arm_4_joint':0, 'right_arm_5_joint':-0.5,
     'right_arm_6_joint':0, 'left_arm_0_joint':0.3, 'left_arm_1_joint':1.8, 'left_arm_2_joint':-1.25,
     'left_arm_3_joint':-0.85, 'left_arm_4_joint':0, 'left_arm_5_joint':0.5, 'left_arm_6_joint':0}

# left max pos of torso
q_map_left_torso = {'torso_0_joint':1.55, 'right_arm_0_joint':-0.3, 'right_arm_1_joint':-1.8,
     'right_arm_2_joint':1.25, 'right_arm_3_joint':0.85, 'right_arm_4_joint':0, 'right_arm_5_joint':-0.5,
     'right_arm_6_joint':0, 'left_arm_0_joint':0.3, 'left_arm_1_joint':1.8, 'left_arm_2_joint':-1.25,
     'left_arm_3_joint':-0.85, 'left_arm_4_joint':0, 'left_arm_5_joint':0.5, 'left_arm_6_joint':0}

# right max pos of torso
q_map_right_torso = {'torso_0_joint':-1.55, 'right_arm_0_joint':-0.3, 'right_arm_1_joint':-1.8,
     'right_arm_2_joint':1.25, 'right_arm_3_joint':0.85, 'right_arm_4_joint':0, 'right_arm_5_joint':-0.5,
     'right_arm_6_joint':0, 'left_arm_0_joint':0.3, 'left_arm_1_joint':1.8, 'left_arm_2_joint':-1.25,
      'left_arm_3_joint':-0.85, 'left_arm_4_joint':0, 'left_arm_5_joint':0.5, 'left_arm_6_joint':0}

def exitError(code):
	if code == 0:
		print "OK"
		exit(0)
	print "ERROR:", code
	exit(code)

def clearAround(head_tilt, head_pan, head_pan_last,start_pos,velma) :
    global q_map_0
    global q_map_left_torso
    global q_map_right_torso
    
    print "moving head tilt"
    q_dest = (head_pan_last, head_tilt)
    velma.moveHead(q_dest, 2.0, start_time=0.5)
    if velma.waitForHead() != 0:
        exitError(6)
    rospy.sleep(0.5)

    print "moving head pan"
    q_dest = (head_pan, head_tilt)
    velma.moveHead(q_dest, 3.0, start_time=0.5)
    if velma.waitForHead() != 0:
        exitError(6)
    rospy.sleep(0.5)

    if not isHeadConfigurationClose( velma.getHeadCurrentConfiguration(), q_dest, 0.1 ):
        exitError(7)

    if start_pos == 'left' :
        print "Turning torso left."
        velma.moveJoint(q_map_left_torso, 6.0, start_time=0.5, position_tol=15.0/180.0*math.pi)
        velma.waitForJoint()
    elif start_pos == 'right' :
        print "Turning torso right."
        velma.moveJoint(q_map_right_torso, 6.0, start_time=0.5, position_tol=15.0/180.0*math.pi)
        velma.waitForJoint()
    rospy.sleep(1.0)

    

if __name__ == "__main__":

    rospy.init_node('head_test', anonymous=False)

    rospy.sleep(0.5)

    print "Running python interface for Velma..."
    velma = VelmaInterface()
    print "Waiting for VelmaInterface initialization..."
    if not velma.waitForInit(timeout_s=10.0):
        print "Could not initialize VelmaInterface\n"
        exitError(1)
    print "Initialization ok!\n"

    diag = velma.getCoreCsDiag()
    if not diag.motorsReady():
        print "Motors must be homed and ready to use for this test."
        exitError(1)
    print "Motors must be enabled every time after the robot enters safe state."
    print "If the motors are already enabled, enabling them has no effect."
    print "Enabling motors..."
    if velma.enableMotors() != 0:
        exitError(2)

    print "Moving to the current position..."
    js_start = velma.getLastJointState()
    velma.moveJoint(js_start[1], 0.5, start_time=0.5, position_tol=15.0/180.0*math.pi)
    error = velma.waitForJoint()
    if error != 0:
        print "The action should have ended without error, but the error code is", error
        exitError(3)
    
    #move fingers left hand
    dest_q = [0.5*math.pi,0.5*math.pi,0.5*math.pi,0]
    print "move left:", dest_q
    velma.moveHandLeft(dest_q, [1,1,1,1], [2000,2000,2000,2000], 1000, hold=True)
    if velma.waitForHandLeft() != 0:
        exitError(6)
    rospy.sleep(0.5)

    #move fingers right hand
    dest_q = [0.5*math.pi,0.5*math.pi,0.5*math.pi,0]
    print "move left:", dest_q
    velma.moveHandRight(dest_q, [1,1,1,1], [2000,2000,2000,2000], 1000, hold=True)
    if velma.waitForHandLeft() != 0:
        exitError(6)
    rospy.sleep(0.5)

    print "moving head to position: 0"
    q_dest = (0,0)
    velma.moveHead(q_dest, 3.0, start_time=0.5)
    if velma.waitForHead() != 0:
        exitError(4)
    rospy.sleep(0.5)
    if not isHeadConfigurationClose( velma.getHeadCurrentConfiguration(), q_dest, 0.1 ):
        exitError(5)

    #clearAround(head_tilt, head_pan,head_pan_last, start_pos,velma)
    print("clearing top")    
    clearAround(-0.7, 1.55 ,0  ,'left',velma)
    clearAround(-0.7, -1.55,1.55,'right',velma)
    
    print("clearing above mid") 
    clearAround(0.1, 1.55,-1.55 ,'left',velma)

    print("clearing under mid") 
    clearAround(0.7, -1.55,1.55,'right',velma)

    print("clearing bot") 
    clearAround(1.1,    0 , -1.55,'left',velma)   
    clearAround(1.1, 1.55,0,'nothing',velma)


    #Return to default pos
    print "Moving to position 0 (slowly)."
    velma.moveJoint(q_map_0, 7.0, start_time=0.5, position_tol=15.0/180.0*math.pi)
    velma.waitForJoint()

    print "moving head to position: 0"
    q_dest = (0,0)
    velma.moveHead(q_dest, 3.0, start_time=0.5)
    if velma.waitForHead() != 0:
        exitError(14)
    if not isHeadConfigurationClose( velma.getHeadCurrentConfiguration(), q_dest, 0.1 ):
        exitError(15)

    exitError(0)
