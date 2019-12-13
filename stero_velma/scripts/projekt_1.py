#!/usr/bin/env python

import roslib; roslib.load_manifest('velma_task_cs_ros_interface')

import rospy
import copy
import PyKDL
import math
from velma_common import *
from rcprg_ros_utils import exitError
from control_msgs.msg import FollowJointTrajectoryResult


# starting position of velma
q_map_0 = {'torso_0_joint':0, 'right_arm_0_joint':-0.3, 'right_arm_1_joint':-1.8,
     'right_arm_2_joint':1.25, 'right_arm_3_joint':0.85, 'right_arm_4_joint':0, 'right_arm_5_joint':-0.5,
     'right_arm_6_joint':0, 'left_arm_0_joint':0.3, 'left_arm_1_joint':1.8, 'left_arm_2_joint':-1.25,
     'left_arm_3_joint':-0.85, 'left_arm_4_joint':0, 'left_arm_5_joint':0.5, 'left_arm_6_joint':0}

#prepare left arm for grabbing jar
q_map_1 = {'torso_0_joint':0, 'right_arm_0_joint':-0.3, 'right_arm_1_joint':-1.8,
     'right_arm_2_joint':1.25, 'right_arm_3_joint':0.85, 'right_arm_4_joint':0, 'right_arm_5_joint':-0.5,
     'right_arm_6_joint':0, 'left_arm_0_joint': 0.4, 'left_arm_1_joint':1.8, 'left_arm_2_joint':-1.25,
     'left_arm_3_joint':-2.0, 'left_arm_4_joint':0, 'left_arm_5_joint':1.6, 'left_arm_6_joint':0}

def exitError(code):
	if code == 0:
		print "OK"
		exit(0)
	print "ERROR:", code
	exit(code)

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

    print "moving head to position: 0"
    q_dest = (0,0)
    velma.moveHead(q_dest, 3.0, start_time=0.5)
    if velma.waitForHead() != 0:
        exitError(4)
    rospy.sleep(0.5)
    if not isHeadConfigurationClose( velma.getHeadCurrentConfiguration(), q_dest, 0.1 ):
        exitError(5)

    #open fingers left hand
    print("opening fingers")
    dest_q = [0,0,0,0]
    velma.moveHandLeft(dest_q, [1,1,1,1], [2000,2000,2000,2000], 1000, hold=True)
    if velma.waitForHandLeft() != 0:
        exitError(6)
    rospy.sleep(0.5)
   
    # VELMA IS READY FOR MOVING 
    

    print "Preparing for grabing jar."
    velma.moveJoint(q_map_1, 5.0, start_time=0.5, position_tol=15.0/180.0*math.pi)
    velma.waitForJoint()
    
    #Left arm is ready 

    jarPos = velma.getTf("B", "jar")
    jarX, jarY, jarZ = jarPos.p
    print("pozycja sloika : ",jarPos.p)
    print("sloik X : ",jarX," sloik Y : ",jarY," sloik Z : ",jarZ)

    print("podchodze do celu")
    P2_global = PyKDL.Vector(jarX-0.25,jarY,jarZ+0.1)
    T_B_Trd = PyKDL.Frame(PyKDL.Rotation.RotZ(math.pi), P2_global)
    velma.moveCartImpLeft([T_B_Trd], [4.0], None, None, None, None, PyKDL.Wrench(PyKDL.Vector(5,5,5), PyKDL.Vector(5,5,5)), start_time=0.5)
    if velma.waitForEffectorLeft() != 0:
         exitError(17)
    rospy.sleep(0.5)

    #close fingers left hand and grab jar
    fin_angle = 0.43*math.pi
    dest_q = [fin_angle,fin_angle,fin_angle,0]
    print "move left:", dest_q
    velma.moveHandLeft(dest_q, [1,1,1,1], [2000,2000,2000,2000], 1000, hold=True)
    if velma.waitForHandLeft() != 0:
        exitError(6)
    rospy.sleep(0.5)

        
    print("Taking jar.")
    current_pos = velma.getLastJointState()
    current_pos[1]['left_arm_3_joint'] = current_pos[1]['left_arm_3_joint'] - 0.15 if current_pos[1]['left_arm_3_joint'] - 0.15 > -2.08 else -2.08
    velma.moveJoint(current_pos[1], 3.0, start_time=0.5, position_tol=15.0/180.0*math.pi)
    velma.waitForJoint()
    rospy.sleep(0.5)

    destPos = velma.getTf("B", "dest") # dostan pozycje stolu
    destX, destY, destZ = destPos.p
    #turn torso towards table dest
    print("turn torso towards table dest")
    # to do lepszy warunek bo ten jest niepelny
    current_pos[1]['torso_0_joint'] = math.atan2(destY,destX) if math.atan2(destY,destX) < math.pi/2 else math.pi/2 - 0.01

    
    velma.moveJoint(current_pos[1], 5.0, start_time=0.5, position_tol=15.0/180.0*math.pi)
    velma.waitForJoint()
    rospy.sleep(0.5)

    #now im ready to put back jar on the table
    print("podchodze do celu")
    Pdest_global = PyKDL.Vector(destX-0.53,destY+0.58,destZ+1.2)
    T_B_dest = PyKDL.Frame(PyKDL.Rotation.RotZ(math.pi), Pdest_global)
    velma.moveCartImpLeft([T_B_dest], [4.0], None, None, None, None, PyKDL.Wrench(PyKDL.Vector(5,5,5), PyKDL.Vector(5,5,5)), start_time=0.5)
    if velma.waitForEffectorLeft() != 0:
         exitError(17)
    rospy.sleep(0.5)

    #release grip
    print("opening fingers")
    dest_q = [0,0,0,0]
    print "move left:", dest_q
    velma.moveHandLeft(dest_q, [1,1,1,1], [2000,2000,2000,2000], 1000, hold=True)
    if velma.waitForHandLeft() != 0:
        exitError(6)
    rospy.sleep(0.5)

    #returning to default pose
    print "Preparing for grabing jar."
    velma.moveJoint(q_map_1, 5.0, start_time=0.5, position_tol=15.0/180.0*math.pi)
    velma.waitForJoint()