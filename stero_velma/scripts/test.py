#!/usr/bin/env python

import roslib; roslib.load_manifest('velma_task_cs_ros_interface')

import rospy
import copy
import PyKDL
import math
from velma_common import *
from rcprg_ros_utils import exitError
from control_msgs.msg import FollowJointTrajectoryResult

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
    print(result)
    result = isPointInRange(P0,basePos,P2,P3)
    if result != 0 :
        resX,resY,resZ = result
        if resX * destX >= 0 :
            if resY * destY >= 0 :
                intersection = result
    print(result)
    result = isPointInRange(P0,basePos,P4,P3)
    if result != 0 :
        resX,resY,resZ = result
        if resX * destX >= 0 :
            if resY * destY >= 0 :
                intersection = result
    print(result)
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
    print(getDestPoint())
    print(intersection(PyKDL.Vector(1,1,2),PyKDL.Vector(3,3,2),PyKDL.Vector(4,0,2),PyKDL.Vector(0,4,2)))
    