#!/usr/bin/env python

import rospy
import math
import PyKDL
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped

# Square coordinates calculation
P1 = PyKDL.Vector(0, 0, 0)
P2 = PyKDL.Vector(0.25, 0, 0)
P3 = PyKDL.Vector(0.25, -0.25, 0)
P4 = PyKDL.Vector(0, -0.25, 0)

T_trans = PyKDL.Vector(0.1, -0.67, 1.42)
T_rot = PyKDL.Rotation.RotX(math.pi/4)

if __name__ == "__main__":

    # init node
	rospy.init_node('figure_broadcaster')

	T = PyKDL.Frame(T_rot, T_trans)

	P1_global = T*P1;
	P2_global = T*P2;
	P3_global = T*P3;
	P4_global = T*P4;

	path_pub = rospy.Publisher('figure', Path, queue_size=10)

	rospy.sleep(2)

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

	r = rospy.Rate(50) # 10hz
	while not rospy.is_shutdown():
	   # publish figure
	   path_pub.publish(figure)
	   r.sleep()
			