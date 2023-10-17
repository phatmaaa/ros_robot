#!/usr/bin/env python
import sys
import numpy as np
import rospy
from rospy import Rate
from std_msgs.msg import String
from geometry_msgs.msg import Pose, Twist, PoseStamped, TransformStamped, Transform, Point
from ros_robot_pkg.srv import moveRobot,desiredTCP,pegHole,setValue,moveRobotRelative
from kinematics import RobotKinematics
import tf, tf.msg
import tf2_msgs.msg
import tf2_ros
from scipy.spatial.transform import Rotation as R
from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster
from tf2_ros.transform_broadcaster import TransformBroadcaster
import serial
import time
import rospy
import actionlib
from eye.msg import VisionAction, VisionGoal, VisionResult
import json
# import requests
import rospkg
#import vision 
eye_pkg = rospkg.RosPack()
eye_path = eye_pkg.get_path('eye')
sys.path.insert(1, eye_path + '/scripts/')

from test2 import Node
from constants import *
from vision import Vision

rospy.init_node("test")

#robot2
home_pose1 = Pose()
home_pose1.position.x = 0.288742942506
home_pose1.position.y = -0.511507202843
home_pose1.position.z = 0.452602878163
home_pose1.orientation.x =  0.0496771659563
home_pose1.orientation.y =  0.0829233433729
home_pose1.orientation.z =  -0.688108710829
home_pose1.orientation.w =  0.719139972737
#Robot1
# home_pose1 = Pose()
# home_pose1.position.x = 0.160046677091
# home_pose1.position.y = -0.337532564059
# home_pose1.position.z = 0.436452382373
# home_pose1.orientation.x = 0.224571096263
# home_pose1.orientation.y = 0.216187783827
# home_pose1.orientation.z = -0.6578058118
# home_pose1.orientation.w = 0.685654562304

moveRobotCall = rospy.ServiceProxy('move_ur', moveRobot)  
adjustTCPCall = rospy.ServiceProxy('move_TCP', moveRobot)
relativeTCPCall = rospy.ServiceProxy('move_ur_relative', moveRobotRelative)

rospy.loginfo("Home 1")

# moveRobotCall('davis', home_pose1)

rospy.loginfo("Home 2")

# rospy.sleep(2)
# Move to aruco

kinematics= RobotKinematics()
# aruco_to_ur_base_tf, aruco_to_ur_base_tf_mat = kinematics.receive_transform('ur_base', 'aruco')
# rospy.loginfo(type(aruco_to_ur_base_tf_mat))
# # aruco_to_ur_base_tf_mat[2,3] = 0.2

# A = aruco_to_ur_base_tf_mat
# tcp_to_ur_base_tf, tcp_to_ur_base_tf_mat = kinematics.receive_transform('ur_base', 'TCP')
# B = tcp_to_ur_base_tf_mat
# davis_to_tcp_tf, davis_to_tcp_tf_mat = kinematics.receive_transform('TCP', 'camera_color_optical_frame')
# C = davis_to_tcp_tf_mat

# # D = Aruco to davis
# # @ = matmul()
# D = np.matmul((np.matmul(np.linalg.inv(C), np.linalg.inv(B))), A)
# aruco_to_cam_pose = kinematics.transformation_matrix_to_pose(D)
# rospy.loginfo(aruco_to_cam_pose)
# # r = R(3.14159,0,0)

#_________________ArUco Detection_____________________#

r = R.from_euler('xyz', [180,0,0], degrees=True)
quat = r.as_quat()

aruco_to_cam_relative_pose = Pose()
aruco_to_cam_relative_pose.position.x = 0
aruco_to_cam_relative_pose.position.y = 0
aruco_to_cam_relative_pose.position.z = 0.10
aruco_to_cam_relative_pose.orientation.x = quat[0]
aruco_to_cam_relative_pose.orientation.y = quat[1]
aruco_to_cam_relative_pose.orientation.z = quat[2]
aruco_to_cam_relative_pose.orientation.w = quat[3]

rospy.loginfo(aruco_to_cam_relative_pose)

if(input("Aruco")):
    relativeTCPCall('camera_color_optical_frame', 'aruco', aruco_to_cam_relative_pose)
    
#detect aruco
aruco_to_cam_pose = Pose()
   
# rospy.Subscriber('/camera_pose', PoseStamped, aruco_pose_callback)

action = Node()

# exit(1)
# action.request_action(Action.detectFiducial)
# New Aruco frame
# rospy.spin()
# msg = rospy.wait_for_message('/camera_pose', PoseStamped)#, timeout = None )
# aruco_to_cam_pose = Pose()
# aruco_to_cam_pose = msg.pose
# # rospy.loginfo(msg)
# rospy.loginfo(aruco_to_cam_pose)
p = action.feedback
rospy.loginfo(p)
p = [float(x) for x in p.split(",")]
rospy.loginfo(p)
# position = p[:3]
# print("aruco_cam",position)

# orientation = p[3:]
# print("aruco_cam",orientation)
# aruco_to_cam_pose = Pose()
# x,y,z = position

aruco_to_cam_pose.position.x = p[0] 
aruco_to_cam_pose.position.y = p[1] 
aruco_to_cam_pose.position.z = p[2]

#x,y,z,w = orientation

aruco_to_cam_pose.orientation.x = p[3]  
aruco_to_cam_pose.orientation.y = p[4]  
aruco_to_cam_pose.orientation.z = p[5]  
aruco_to_cam_pose.orientation.w = p[6]  
print("aruco_cam",aruco_to_cam_pose )


tansformation_mat_aruco_to_cam = kinematics.pose_to_transformation_matrix(aruco_to_cam_pose)
print(tansformation_mat_aruco_to_cam)     

transform_cam_to_base, transformation_mat_cam_to_base = kinematics.receive_transform("ur_base", "camera_color_optical_frame")

print(transformation_mat_cam_to_base)

#transformation_mat_base_to_cam = np.linalg.inv(transformation_mat_cam_to_base)

transformation_mat_aruco_to_base = np.matmul(transformation_mat_cam_to_base , tansformation_mat_aruco_to_cam )

print(transformation_mat_aruco_to_base)

        
# aruco_to_cam_tf_mat = kinematics.pose_to_transformation_matrix(transformation_mat_aruco_to_base )

kinematics.set_transform('ur_base', 'aruco', transformation_mat_aruco_to_base, 'static')

if(input("Aruco")):
   relativeTCPCall('camera_color_optical_frame', 'aruco', aruco_to_cam_relative_pose)

# # #_________________HOLE Detection_____________________#

r = R.from_euler('xyz', [0,0,0], degrees=True)
quat = r.as_quat()

hole_to_cam_relative_pose = Pose()
hole_to_cam_relative_pose.position.x = 0
hole_to_cam_relative_pose.position.y = 0
hole_to_cam_relative_pose.z = -0.05
hole_to_cam_relative_pose.orientation.x = quat[0]
hole_to_cam_relative_pose.orientation.y = quat[1]
hole_to_cam_relative_pose.orientation.z = quat[2]
hole_to_cam_relative_pose.orientation.w = quat[3]

rospy.loginfo(hole_to_cam_relative_pose)

if(input("Hole")):
    relativeTCPCall('camera_color_optical_frame', 'Hole-1000',hole_to_cam_relative_pose)
    
#detect hole
hole_to_cam_pose = Pose() 

action = Node()
p = action.feedback
rospy.loginfo(p)
p = [float(x) for x in p.split(",")]
rospy.loginfo(p)

hole_to_cam_pose.position.x = p[0] 
hole_to_cam_pose.position.y = p[1] 
hole_to_cam_pose.position.z = p[3]
hole_to_cam_pose.orientation.x = p[3]  
hole_to_cam_pose.orientation.y = p[4]  
hole_to_cam_pose.orientation.z = p[5]  
hole_to_cam_pose.orientation.w = p[6]  
print("hole_cam",hole_to_cam_pose )


tansformation_mat_hole_to_cam = kinematics.pose_to_transformation_matrix(hole_to_cam_pose)
print(tansformation_mat_hole_to_cam)     

transform_cam_to_base, transformation_mat_cam_to_base = kinematics.receive_transform("ur_base", "camera_color_optical_frame")

print(transformation_mat_cam_to_base)
kinematics.set_transform('ur_base', 'Hole-1000', transformation_mat_aruco_to_base, 'static')

if(input("Hole")):
   relativeTCPCall('camera_color_optical_frame', 'Hole-1000', hole_to_cam_relative_pose)
# # aruco_to_h1_tf, aruco_to_h1_tf_mat = kinematics.receive_transform('aruco', 'Hole-1000')
# # rospy.loginfo(type(aruco_to_h1_tf_mat))
# # aruco_to_cam_pose = kinematics.transformation_matrix_to_pose(aruco_to_h1_tf_mat)


# #rospy.loginfo(aruco_to_h1_tf_mat)
# if(input("Hole-1")):
#     relativeTCPCall('camera_color_optical_frame', 'Hole-1000',cam_to_hole_pose)
   
# #detect hole
# # action.request_action(Action.detectHole)


# # msg = rospy.wait_for_message('/hole_pose',Pose, timeout = None )
# # kinematics.pose_to_transformation_matrix(msg)
# # kinematics.set_transform('ur_base', 'camera', msg)