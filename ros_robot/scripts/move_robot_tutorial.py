#!/usr/bin/env python
import numpy as np
from sensor_msgs.msg import Image, CameraInfo
import rospy
from rospy import Rate
from std_msgs.msg import String,Int32,Int32MultiArray,MultiArrayLayout,MultiArrayDimension,Float32MultiArray,Float32, Float64MultiArray, Int32MultiArray, Bool
from geometry_msgs.msg import Vector3, Pose, Twist, PoseStamped, TransformStamped, Transform, Point
from ros_robot_pkg.srv import moveRobot,desiredTCP,pegHole,setValue
import tf, tf.msg
import tf2_ros
from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster
from tf2_ros.transform_broadcaster import TransformBroadcaster
import geometry_msgs.msg
####################################
rospy.init_node('move_robot_tutorial')

listener = tf.TransformListener()
moveRobotCall = rospy.ServiceProxy('move_ur', moveRobot)   
adjustTCPCall = rospy.ServiceProxy('move_TCP', moveRobot)
br=tf2_ros.StaticTransformBroadcaster()

##To Move wrt ur_base frame##
#home position:
new_pose = Pose()
new_pose.orientation.x = 0
new_pose.orientation.y = 1
new_pose.orientation.z = 0
new_pose.orientation.w = 0
new_pose.position.x = -0.2
new_pose.position.y = -1.14
new_pose.position.z = 0.85
moveRobotCall('davis', new_pose)

##using tf_tree to broadcast a frame##
t_0=geometry_msgs.msg.TransformStamped()
t_0.header.stamp=rospy.Time()
t_0.header.frame_id="/ur_base"
t_0.child_frame_id="/Object"
t_0.transform.translation.x=-0.2
t_0.transform.translation.y=-1.14
t_0.transform.translation.z=0.45
t_0.transform.rotation.x=0
t_0.transform.rotation.y=1
t_0.transform.rotation.z=0
t_0.transform.rotation.w=0
br.sendTransform([t_0])

# Define the motion of the camera with respect to the object
positive_motion = new_pose.position ([0, 0.1, 0])
negative_motion = new_pose.position ([0, -0.1, 0])

# Move the camera to the positive y-axis by 10 cm
adjustTCPCall ([0, 0, 0, 0, 0, 0])
adjustTCPCall ('davis', positive_motion)
listener.waitForTransform("/Object", "/davis", rospy.Time(), rospy.Duration(1))
Transformation = listener.lookupTransform("/Object", "/davis", rospy.Time())
(trans,rot) = Transformation
print("Translation_meters :   "+str(trans))

# Move the camera to the negative y-axis by 10 cm
adjustTCPCall ([0, 0, 0, 0, 0, 0])
adjustTCPCall ('davis', negative_motion)
listener.waitForTransform("/Object", "/davis", rospy.Time(), rospy.Duration(1))
Transformation = listener.lookupTransform("/Object", "/davis", rospy.Time())
(trans,rot) = Transformation
print("Translation_meters :   "+str(trans))

# Object movements with respect to the base frame
base_frame1 = np.dot (positive_motion, new_pose)
base_frame2 = np.dot (negative_motion, new_pose)
print (base_frame1)
print (base_frame2)