#!/usr/bin/env python
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
from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster
from tf2_ros.transform_broadcaster import TransformBroadcaster
import serial
import time

def talker():
    pub = rospy.Publisher('/camera_pose', Pose, queue_size=1)
    rospy.init_node('pose_test', anonymous=True)
    rate = rospy.Rate(1) # 1hz
    
    pose = Pose()
    pose.position.x = 0.111519940196
    pose.position.y = -0.716364057388
    pose.position.z = 0.517137940997
    pose.orientation.x = 0.9972556046
    pose.orientation.y = 0.00042619554545
    pose.orientation.z = 0.041151279378
    pose.orientation.w = 0.0615438839872
    
    rospy.loginfo(pose)
    pub.publish(pose)
    rate.sleep()
    rospy.sleep(1)

    pose = Pose()
    pose.position.x = 0.111519940196
    pose.position.y = -0.816364057388
    pose.position.z = 0.517137940997
    pose.orientation.x = 0.9972556046
    pose.orientation.y = 0.00042619554545
    pose.orientation.z = 0.041151279378
    pose.orientation.w = 0.0615438839872
    
    rospy.loginfo(pose)
    pub.publish(pose)
    rate.sleep()
    rospy.sleep(1)
    

if __name__ == '__main__':
    
    while not rospy.is_shutdown():
        
        try:
            talker()
        except Exception as e:
            print(e)