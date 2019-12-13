#!/usr/bin/env python

import roslib; roslib.load_manifest('velma_task_cs_ros_interface')

import rospy
import copy
import PyKDL
import math
from velma_common import *
from rcprg_planner import *
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

def intersection(p1,p2,p3,p4) :
    x1,y1,z1 = p1
    x2,y2,z2 = p2
    x3,y3,z3 = p3
    x4,y4,z4 = p4
    special = 0
    a1 = 0
    a3 = 0
    b1 = 0
    b3 = 0    
    x = 0
    y = 0
    z = z1
    
    if ifEqual(x1,x2) :
        x = x1
        special = 1
    else :
        a1=(y1-y2)/(x1-x2)
        b1=y1-a1*x1
    if ifEqual(x3,x4) :
        x = x3
        special = 1
    else :
        a3=(y3-y4)/(x3-x4)
        b3=y3-a3*x3

    if ifEqual(a1,a3):
        print("lines are parallel")        
        return 0
   
    vector = None
    if special != 0 :
        if ifEqual(x,x1) :
            y = a3 * x + b3
        elif ifEqual(x,x3) :
            y = a1 * x + b1
        vector = PyKDL.Vector(x,y,z) 
    else :        
        x = (b3-b1)/(a1-a3)
        y = a1*x + b1
        vector = PyKDL.Vector(x,y,z)
    print("vector : ",vector)
    return vector

def getDestPoint() :
    saveSpaceX = 0.6
    saveSpaceY = 0.3
    above_table = 0.3
    basePos = velma.getTf("dest", "B") # dostan pozycje stolu
    destX, destY, destZ = basePos.p
    P0 = PyKDL.Vector(0, 0, 1 + above_table )    
    P1 = PyKDL.Vector(-saveSpaceX,-saveSpaceY,1 + above_table )
    P2 = PyKDL.Vector( saveSpaceX,-saveSpaceY,1 + above_table )
    P3 = PyKDL.Vector( saveSpaceX, saveSpaceY,1 + above_table )
    P4 = PyKDL.Vector(-saveSpaceX, saveSpaceY,1 + above_table )  
    intersection = 0
    result = isPointInRange(P0,basePos,P1,P2)
    if result != 0 :
        resX,resY,resZ = result
        if resX * destX >= 0 :
            if resY * destY >= 0 :
                intersection = result
    result = isPointInRange(P0,basePos,P2,P3)
    if result != 0 :
        resX,resY,resZ = result
        if resX * destX >= 0 :
            if resY * destY >= 0 :
                intersection = result
    result = isPointInRange(P0,basePos,P4,P3)
    if result != 0 :
        resX,resY,resZ = result
        if resX * destX >= 0 :
            if resY * destY >= 0 :
                intersection = result
    result = isPointInRange(P0,basePos,P1,P4)
    if result != 0 :
        resX,resY,resZ = result
        if resX * destX >= 0 :
            if resY * destY >= 0 :
                intersection = result
    destPos = velma.getTf("B","dest")
    return destPos * intersection


def isPointInRange(P0,basePos,a,b) :
    x , y, z = intersection(P0,basePos.p, a,b)
    if (ifEqual(a.y(),b.y()) and a.x() <= x and b.x() >= x) or ( ifEqual(a.x(),b.x()) and a.y() <= y and b.y() >= y ) :
        return PyKDL.Vector(x,y,z)
    else :
        return 0

def ifEqual(a,b) :
    if abs(a-b) < 10**(-6) :
        return True
    else :
        return False


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
   
    # preparing planner
    p = Planner(velma.maxJointTrajLen())
    if not p.waitForInit():
         print "could not initialize PLanner"
         exitError(2)
    oml = OctomapListener("/octomap_binary")
    rospy.sleep(1.0)
    octomap = oml.getOctomap(timeout_s=5.0)
    p.processWorld(octomap)
    

    # define a function for frequently used routine in this test
    def planAndExecute(q_dest):
        print "Planning motion to the goal position using set of all joints..."
        print "Moving to valid position, using planned trajectory."
        goal_constraint = qMapToConstraints(q_dest, 0.01, group=velma.getJointGroup("impedance_joints"))
        for i in range(20):
            rospy.sleep(0.5)
            js = velma.getLastJointState()
            print "Planning (try", i, ")..."
            traj = p.plan(js[1], [goal_constraint], "impedance_joints", max_velocity_scaling_factor=0.1, planner_id="RRTConnect")
            if traj == None:
                continue
            print "Executing trajectory..."
            if not velma.moveJointTraj(traj, start_time=0.5):
                exitError(5)
            if velma.waitForJoint() == 0:
                break
            else:
                print "The trajectory could not be completed, retrying..."
                continue
        rospy.sleep(0.5)
        js = velma.getLastJointState()
        if not isConfigurationClose(q_dest, js[1]):
            exitError(6)
 
    
    # VELMA IS READY FOR MOVING 
    

    print "Preparing for grabing jar."
    planAndExecute(q_map_1)
    
    #Left arm is ready 

    jarFrame = velma.getTf("B", "jar")
    jarX, jarY, jarZ = jarFrame.p
    actual_pos = velma.getLastJointState()
    rotZ = actual_pos[1]['torso_0_joint'] + math.pi
        
    print("podchodze do celu")
    Rot = PyKDL.Rotation.RotZ(rotZ)
    P2_global = PyKDL.Vector(jarX-0.25,jarY,jarZ+0.1)
    T_B_Trd = PyKDL.Frame(Rot, P2_global)
    velma.moveCartImpLeft([T_B_Trd], [4.0], None, None, None, None, PyKDL.Wrench(PyKDL.Vector(5,5,5), PyKDL.Vector(5,5,5)), start_time=0.5)
    if velma.waitForEffectorLeft() != 0:
         exitError(17)
    rospy.sleep(0.5)


    #close fingers left hand and grab jar
    print("Taking jar.")    
    fin_angle = 0.43*math.pi
    dest_q = [fin_angle,fin_angle,fin_angle,0]
    velma.moveHandLeft(dest_q, [1,1,1,1], [2000,2000,2000,2000], 1000, hold=True)
    if velma.waitForHandLeft() != 0:
        exitError(6)
    rospy.sleep(0.5)

    print("lifting hand up")
    P2_global = PyKDL.Vector(jarX-0.25,jarY,jarZ+0.3)
    T_B_Trd = PyKDL.Frame(Rot, P2_global)
    velma.moveCartImpLeft([T_B_Trd], [4.0], None, None, None, None, PyKDL.Wrench(PyKDL.Vector(5,5,5), PyKDL.Vector(5,5,5)), start_time=0.5)
    if velma.waitForEffectorLeft() != 0:
         exitError(17)
    rospy.sleep(0.5)

    print("turn torso towards table dest")
    current_pos = velma.getLastJointState()
    destPos = velma.getTf("B", "dest") # dostan pozycje stolu
    destX, destY, destZ = destPos.p
    #turn torso towards table dest
    # to do lepszy warunek bo ten jest niepelny
    print("atan2 : ",math.atan2(destY,destX))
    if math.atan2(destY,destX) < math.pi/2 and math.atan2(destY,destX) > -math.pi/2 :
        current_pos[1]['torso_0_joint'] = math.atan2(destY,destX)
    elif math.atan2(destY,destX) > math.pi/2 :
        current_pos[1]['torso_0_joint'] = math.pi/2 - 0.03
    else :
        current_pos[1]['torso_0_joint'] = -math.pi/2 + 0.03 
    #turning torso
    planAndExecute(current_pos[1])


    #now im ready to put back jar on the table
    print("podchodze do celu do oddania")
    temp = getDestPoint()
    tempX , tempY, tempZ = temp
    print("sin : ",math.sin(math.atan2(destX,destY)))
    print("cos : ",math.cos(math.atan2(destX,destY)))
    tempX = tempX - math.sin(math.atan2(destX,destY)) * 0.3
    tempY = tempY - math.cos(math.atan2(destX,destY)) * 0.3
    #destGlobal = PyKDL.Vector(destX-0.53,destY+0.58,destZ+1.2)
    destGlobal = PyKDL.Vector(tempX,tempY,tempZ)
    actual_pos = velma.getLastJointState()
    rotZ = actual_pos[1]['torso_0_joint']
    Rot = PyKDL.Rotation.RotZ(rotZ + math.pi)
    print("target", destGlobal)
    T_B_dest = PyKDL.Frame(Rot, destGlobal)
    velma.moveCartImpLeft([T_B_dest], [4.0], None ,None , None, None, PyKDL.Wrench(PyKDL.Vector(5,5,5), PyKDL.Vector(5,5,5)), start_time=0.5)
    if velma.waitForEffectorLeft() != 0:
         exitError(17)
    rospy.sleep(0.5)

    nadFrame = velma.getTf("B", "Wl")
    print("koncowka : ",nadFrame.p)

    #release grip
    print("opening fingers")
    dest_q = [0,0,0,0]
    velma.moveHandLeft(dest_q, [1,1,1,1], [2000,2000,2000,2000], 1000, hold=True)
    if velma.waitForHandLeft() != 0:
        exitError(6)
    rospy.sleep(0.5)

    #go back a little bit
    print("im taking my hand")
    tempX = tempX - math.sin(math.atan2(destX,destY)) * 0.25
    tempY = tempY - math.cos(math.atan2(destX,destY)) * 0.25
    destGlobal = PyKDL.Vector(tempX,tempY,tempZ)
    T_B_dest = PyKDL.Frame(Rot, destGlobal)
    velma.moveCartImpLeft([T_B_dest], [4.0], None ,None , None, None, PyKDL.Wrench(PyKDL.Vector(5,5,5), PyKDL.Vector(5,5,5)), start_time=0.5)
    if velma.waitForEffectorLeft() != 0:
         exitError(17)
    rospy.sleep(0.5)

    #returning to default pose
    print "returning to start pos"
    planAndExecute(q_map_1)
    velma.waitForJoint()