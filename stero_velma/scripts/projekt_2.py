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
q_map_start = {'torso_0_joint':0, 'right_arm_0_joint':-0.4, 'right_arm_1_joint':-1.8,
     'right_arm_2_joint':1.25, 'right_arm_3_joint':2.0, 'right_arm_4_joint':0, 'right_arm_5_joint':-1.6,
     'right_arm_6_joint':0, 'left_arm_0_joint': 0.3, 'left_arm_1_joint':1.8, 'left_arm_2_joint':-1.25,
     'left_arm_3_joint':-0.85, 'left_arm_4_joint':0, 'left_arm_5_joint':0.5, 'left_arm_6_joint':0}

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

    #prepare fingers right hand
    print("preparing fingers right hand")
    finger_angle = math.pi/1.75
    dest_q = [finger_angle,finger_angle,finger_angle,math.pi]
    velma.moveHandRight(dest_q, [1,1,1,1], [2000,2000,2000,2000], 1000, hold=True)
    if velma.waitForHandRight() != 0:
        exitError(6)
    rospy.sleep(0.5)

    #close fingers left hand
    print("closing fingers left hand")
    dest_q = [finger_angle,finger_angle,finger_angle,math.pi]
    velma.moveHandLeft(dest_q, [1,1,1,1], [2000,2000,2000,2000], 1000, hold=True)
    if velma.waitForHandLeft() != 0:
        exitError(6)
    rospy.sleep(0.5)
   
    """
    when not working on computer with velma planner

    # preparing planner
    p = Planner(velma.maxJointTrajLen())
    if not p.waitForInit():
         print "could not initialize PLanner"
         exitError(2)
    oml = OctomapListener("/octomap_binary")
    rospy.sleep(1.0)
    octomap = oml.getOctomap(timeout_s=5.0)
    p.processWorld(octomap)
    """

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
    
    # step musi byc odpowiednio maly i miekka musi byc lapa
    def grabHandle(dist_ang,dist_hor,dist_ver):
        T_B_L_door = velma.getTf("B", "right_door")
        T_B_Cab = velma.getTf("B" , "cabinet")
        ldX,ldY,ldZ = T_B_L_door.p
        cabYaw,cabPitch,cabRoll = T_B_Cab.M.GetEulerZYX()
        ldYaw ,ldPitch ,ldRoll  = T_B_L_door.M.GetEulerZYX()
        if ldYaw + dist_ang >= cabYaw + math.pi/2 : # potencjalnie potrzebuje lepszych warunkow na graniczny kat - cabYaw czasami jest dodatni przy obroceniu szafki
            ldYaw = cabYaw + math.pi/2
        else :
            ldYaw = ldYaw + dist_ang
        Rot = PyKDL.Rotation.RotZ(ldYaw + math.pi) 
        tempX = ldX + math.sin(ldYaw) * (0.2835 + dist_hor) + math.cos(ldYaw) * (0.3 + dist_ver)
        tempY = ldY - math.cos(ldYaw) * (0.2835 + dist_hor) + math.sin(ldYaw) * (0.3 + dist_ver)
        Trans = PyKDL.Vector(tempX,tempY,ldZ + 0.08)
        dest_cab = PyKDL.Frame(Rot,Trans)
        return dest_cab
    
    def resetPoseTol():
        print("reseting to real position")
        tolerance = makeTwist(1,1,1,1,1,1)    
        dest_reset = velma.getTf("B", "Wr")
        velma.moveCartImpRight([dest_reset], [5.0], None ,None , [makeWrench(30,30,1000,100,100,100)], [1], PyKDL.Wrench(PyKDL.Vector(5,5,5), PyKDL.Vector(5,5,5)), start_time=0.5,path_tol = tolerance)
        if velma.waitForEffectorRight() != 0:      
            exitError(17)
        rospy.sleep(0.5)

    def makeWrench(lx,ly,lz,rx,ry,rz):
        return PyKDL.Wrench(PyKDL.Vector(lx,ly,lz), PyKDL.Vector(rx,ry,rz))

    def makeTwist(lx,ly,lz,rx,ry,rz):  
        return PyKDL.Twist( PyKDL.Vector(lx,ly,lz), PyKDL.Vector(rx,ry,rz))    
    # VELMA IS READY FOR MOVING 
        

    print("Preparing velma for the task (torso and right hand)")
    T_B_Cab = velma.getTf("B", "cabinet")   
    cabX,cabY,cabZ = T_B_Cab.p
    cab_angle = math.atan2(cabY,cabX)
    if cab_angle < math.pi/2 and cab_angle > -math.pi/2 :
        q_map_start['torso_0_joint'] = cab_angle
    elif cab_angle > math.pi/2 :
        q_map_start['torso_0_joint'] = math.pi/2 - 0.03
    else :
        q_map_start['torso_0_joint'] = -math.pi/2 + 0.03 
    
    
    """
    when not working on computer with planner    
    planAndExecute(q_map_start)
    """
    velma.moveJoint(q_map_start, 5.0, start_time=0.5, position_tol=30.0/180.0*math.pi)
    velma.waitForJoint()
   

    print("moving hand to cabinet")
    tolerance = makeTwist(0.2,0.2,1,1,1,1)
    """
    without tolerance
    dest_cab = grabHandle(0,-0.0735,-0.05)
    """
    dest_cab = grabHandle(0,-0.0835,-0.4)
    velma.moveCartImpRight([dest_cab], [15.0], None ,None , [makeWrench(30,30,1000,100,100,100)], [1], PyKDL.Wrench(PyKDL.Vector(5,5,5), PyKDL.Vector(5,5,5)), start_time=0.5,path_tol = tolerance)
    if velma.waitForEffectorRight() != 0:
        print("dojechalem do szafki")      
        #exitError(17)
    rospy.sleep(0.5)
        

    resetPoseTol()


    print("grabbing handle")
    tolerance = makeTwist(0.2,0.2,1,1,1,1)    
    """
    without tolerance    
    dest_cab = grabHandle(0,0.01,0)
    """
    #dest_cab = grabHandle(0,0.3,-0.05)
    T_B_Wrist = velma.getTf("B", "Wr")
    T_B_L_door = velma.getTf("B", "right_door")
    ldYaw ,ldPitch ,ldRoll  = T_B_L_door.M.GetEulerZYX()
    wristX,wristY,wristZ = T_B_Wrist.p    
    Rot = PyKDL.Rotation.RotZ(ldYaw + math.pi)
    Trans = PyKDL.Vector(wristX + math.sin(ldYaw) * 0.6 ,wristY - math.cos(ldYaw) * 0.6 ,wristZ)
    dest_cab = PyKDL.Frame(Rot,Trans)
    velma.moveCartImpRight([dest_cab], [15.0], None ,None , [makeWrench(30,30,1000,100,100,100)], [1], PyKDL.Wrench(PyKDL.Vector(5,5,5), PyKDL.Vector(5,5,5)), start_time=0.5,path_tol = tolerance)
    if velma.waitForEffectorRight() != 0:
        print("dojechalem do uchwytu")        
        #exitError(17)
    rospy.sleep(0.5)


    resetPoseTol()
    

    for i in range(0,6):
        print("dzialam ",i)
        dest_cab = grabHandle(0.4,0,0)
        velma.moveCartImpRight([dest_cab], [3.0], None ,None , [makeWrench(60,60,1000,100,100,100)], [1], PyKDL.Wrench(PyKDL.Vector(5,5,5), PyKDL.Vector(5,5,5)), start_time=0.5)
        if velma.waitForEffectorRight() != 0:       
            exitError(17)
        rospy.sleep(0.5)        


    print("pulling doors")
    tolerance = makeTwist(0.1,0.1,1,1,1,1)
    dest_cab = grabHandle(0,0,0.2)
    velma.moveCartImpRight([dest_cab], [6.0], None ,None , [makeWrench(100,100,1000,100,100,100)], [1], PyKDL.Wrench(PyKDL.Vector(5,5,5), PyKDL.Vector(5,5,5)), start_time=0.5,path_tol=tolerance)
    if velma.waitForEffectorRight() != 0:
        exitError(17)
    rospy.sleep(0.5)


    resetPoseTol()

    
    print("taking my hand from handle")
    dest_cab = grabHandle(0,-0.0985,0.05)
    velma.moveCartImpRight([dest_cab], [4.0], None ,None , [makeWrench(100,100,1000,100,100,100)], [1], PyKDL.Wrench(PyKDL.Vector(5,5,5), PyKDL.Vector(5,5,5)), start_time=0.5)
    if velma.waitForEffectorRight() != 0:
        exitError(17)
    rospy.sleep(0.5)
    
    print("taking my hand from door")
    dest_cab = grabHandle(0,-0.0935,0.2)
    velma.moveCartImpRight([dest_cab], [4.0], None ,None , [makeWrench(100,100,1000,100,100,100)], [1], PyKDL.Wrench(PyKDL.Vector(5,5,5), PyKDL.Vector(5,5,5)), start_time=0.5)
    if velma.waitForEffectorRight() != 0:
        exitError(17)
    rospy.sleep(0.5)

    
    print("returning to default position")
    """
    when not working on computer with planner
        
    planAndExecute(q_map_start)
    """

    velma.moveJoint(q_map_start, 5.0, start_time=0.5, position_tol=30.0/180.0*math.pi)
    velma.waitForJoint()
