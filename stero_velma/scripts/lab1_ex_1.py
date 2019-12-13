#!/usr/bin/env python

import roslib; roslib.load_manifest('velma_task_cs_ros_interface')

import rospy
import math
import PyKDL
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped

from velma_common.velma_interface import *
from control_msgs.msg import FollowJointTrajectoryResult

def exitError(code):
	if code == 0:
		print "OK"
		exit(0)
	print "ERROR:", code
	exit(code)

if __name__ == "__main__":

    # init node
	rospy.init_node('test_jimp')

	# Square coordinates calculation
	P1 = PyKDL.Vector(0, 0, 0)
	P2 = PyKDL.Vector(0.25, 0, 0)
	P3 = PyKDL.Vector(0.25, -0.25, 0)
	P4 = PyKDL.Vector(0, -0.25, 0)

	T_trans = PyKDL.Vector(0.1, -0.67, 1.42)
	T_rot = PyKDL.Rotation.RotX(math.pi/4)

	T = PyKDL.Frame(T_rot, T_trans)

	P1_global = T*P1;
	P2_global = T*P2;
	P3_global = T*P3;
	P4_global = T*P4;

	figure = Path()
	figure.header.stamp = rospy.Time.now()
	figure.header.frame_id = 'map'
	figure.header.seq = 1

	points = [P1_global, P2_global, P3_global, P4_global, P1_global]
	id_seq = 0
	# Initialize Path
	for point in points:

		id_seq += 1

		#initialize intermediete pose
		pose = PoseStamped()
		pose.header.stamp = rospy.Time.now()
		pose.header.frame_id = "map"
		
		# set pose's sequence number
		pose.header.seq = id_seq  

		# set pose's position    
		pose.pose.position.x = point.x()
		pose.pose.position.y = point.y()
		pose.pose.position.z = point.z()

		# append pose
		figure.poses.append(pose)


	# every joint in position 0
	q_map_0 = {'torso_0_joint':0, 'right_arm_0_joint':0, 'right_arm_1_joint':0,
         'right_arm_2_joint':0, 'right_arm_3_joint':0, 'right_arm_4_joint':0, 'right_arm_5_joint':0,
         'right_arm_6_joint':0, 'left_arm_0_joint':0, 'left_arm_1_joint':0, 'left_arm_2_joint':0,
         'left_arm_3_joint':0, 'left_arm_4_joint':0, 'left_arm_5_joint':0, 'left_arm_6_joint':0 }

	# starting position
	q_map_starting = {'torso_0_joint':0, 'right_arm_0_joint':-0.3, 'right_arm_1_joint':-1.8,
         'right_arm_2_joint':1.25, 'right_arm_3_joint':0.85, 'right_arm_4_joint':0, 'right_arm_5_joint':-0.5,
         'right_arm_6_joint':0, 'left_arm_0_joint':0.3, 'left_arm_1_joint':1.8, 'left_arm_2_joint':-1.25,
         'left_arm_3_joint':-0.85, 'left_arm_4_joint':0, 'left_arm_5_joint':0.5, 'left_arm_6_joint':0 }

	# goal position
	q_map_goal = {'torso_0_joint':0, 'right_arm_0_joint':-0.3, 'right_arm_1_joint':-1.8,
         'right_arm_2_joint':-1.25, 'right_arm_3_joint':1.57, 'right_arm_4_joint':0, 'right_arm_5_joint':-0.5,
         'right_arm_6_joint':0, 'left_arm_0_joint':0.3, 'left_arm_1_joint':1.8, 'left_arm_2_joint':-1.25,
         'left_arm_3_joint':-0.85, 'left_arm_4_joint':0, 'left_arm_5_joint':0.5, 'left_arm_6_joint':0 }

	# intermediate position
	q_map_intermediate = {'torso_0_joint':0, 'right_arm_0_joint':-0.3, 'right_arm_1_joint':-1.6,
         'right_arm_2_joint':-1.25, 'right_arm_3_joint':-0.85, 'right_arm_4_joint':0, 'right_arm_5_joint':-0.5,
         'right_arm_6_joint':0, 'left_arm_0_joint':0.3, 'left_arm_1_joint':1.8, 'left_arm_2_joint':-1.25,
         'left_arm_3_joint':-0.85, 'left_arm_4_joint':0, 'left_arm_5_joint':0.5, 'left_arm_6_joint':0 }

	# Initialize velma
	velma = VelmaInterface()
	if not velma.waitForInit(timeout_s=10.0):
		print "Could not initialize VelmaInterface\n"
		exitError(1)
	print "Initialization ok!\n"
	
	if velma.enableMotors() != 0:
		exitError(2)
	
	rospy.sleep(0.5)

	diag = velma.getCoreCsDiag()
	if not diag.motorsReady():
		print "Motors must be homed and ready to use for this test."
		exitError(1)

	print "Moving to position 0 (slowly)."
	velma.moveJoint(q_map_starting, 5.0, start_time=0.5, position_tol=30.0/180.0*math.pi)
	velma.waitForJoint()

	print "Switch to jnt_imp mode (no trajectory)..."
	velma.moveJointImpToCurrentPos(start_time=0.5)
	error = velma.waitForJoint()
	if error != 0:
		print "The action should have ended without error, but the error code is", error
		exitError(3)
 
	print "Checking if the starting configuration is as expected..."
    
	rospy.sleep(0.5)	

	js = velma.getLastJointState()
	if not isConfigurationClose(q_map_starting, js[1], tolerance=0.3):
		print "This test requires starting pose:"
		print q_map_starting
		exitError(10)


	# Start task
	
	# move in jint_imp
	# print "Moving to position 0 (this motion is too fast and should cause error condition, that leads to safe mode in velma_core_cs)."
	# velma.moveJoint(q_map_0, 20, start_time=0.5, position_tol=0.2, velocity_tol=1)
	# error = velma.waitForJoint()
	# if error != FollowJointTrajectoryResult.PATH_TOLERANCE_VIOLATED:
	# 	print "The action should have ended with PATH_TOLERANCE_VIOLATED error status, but the error code is", error
 	# #   exitError(4)
 	T_B_Tr = P1  #velma.getTf("B", "Tr")

 	T_B_Trd = PyKDL.Frame(PyKDL.Rotation.Quaternion( 0.0 , 0.0 , 0.0 , 1.0 ), P1_global)
 	velma.moveCartImpRight([T_B_Trd], [4.0], None, None, None, None, PyKDL.Wrench(PyKDL.Vector(5,5,5), PyKDL.Vector(5,5,5)), start_time=0.5)
	if velma.waitForEffectorRight() != 0:
         exitError(17)
	rospy.sleep(0.5)

  	T_B_Trd = PyKDL.Frame(PyKDL.Rotation.Quaternion( 0.0 , 0.0 , 0.0 , 1.0 ), P2_global)
 	velma.moveCartImpRight([T_B_Trd], [4.0], None, None, None, None, PyKDL.Wrench(PyKDL.Vector(5,5,5), PyKDL.Vector(5,5,5)), start_time=0.5)
	if velma.waitForEffectorRight() != 0:
         exitError(17)
	rospy.sleep(0.5)

  	T_B_Trd = PyKDL.Frame(PyKDL.Rotation.Quaternion( 0.0 , 0.0 , 0.0 , 1.0 ), P3_global)
 	velma.moveCartImpRight([T_B_Trd], [4.0], None, None, None, None, PyKDL.Wrench(PyKDL.Vector(5,5,5), PyKDL.Vector(5,5,5)), start_time=0.5)
	if velma.waitForEffectorRight() != 0:
         exitError(17)
	rospy.sleep(0.5)

  	T_B_Trd = PyKDL.Frame(PyKDL.Rotation.Quaternion( 0.0 , 0.0 , 0.0 , 1.0 ), P4_global)
 	velma.moveCartImpRight([T_B_Trd], [4.0], None, None, None, None, PyKDL.Wrench(PyKDL.Vector(5,5,5), PyKDL.Vector(5,5,5)), start_time=0.5)
	if velma.waitForEffectorRight() != 0:
         exitError(17)
	rospy.sleep(0.5)

	T_B_Trd = PyKDL.Frame(PyKDL.Rotation.Quaternion( 0.0 , 0.0 , 0.0 , 1.0 ), P1_global)
 	velma.moveCartImpRight([T_B_Trd], [4.0], None, None, None, None, PyKDL.Wrench(PyKDL.Vector(5,5,5), PyKDL.Vector(5,5,5)), start_time=0.5)
	if velma.waitForEffectorRight() != 0:
         exitError(17)
	rospy.sleep(0.5)









