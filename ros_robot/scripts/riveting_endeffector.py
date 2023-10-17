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
from scipy.spatial.transform import Rotation as R
from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster
from tf2_ros.transform_broadcaster import TransformBroadcaster
import serial
import time


#ADD a class
class Robotic_Manipulation:

    #ADD a constructor
    def __init__(self):
        #Constructor method of the `Robotic_Manipulation` class. 
        # It initializes various attributes 
        # and sets up ROS publishers, subscribers, and services.
        self.vel = 0.5
        self.acc = 0.5
        self.stop_acc = 0.3

        self.cmd_velocity_vector = []
        self.move_vel = False

        self.item_height = 0.11
        
        self.cam_to_aruco_matrix_trans = []
        self.cam_to_aruco_matrix_rot = []
        self.base_to_aruco_matrix_trans = []
        self.base_to_aruco_matrix_rot = []
        self.kinematics = RobotKinematics()
        #self.current_TCP = 'TCP'

        self.robot_pose = PoseStamped()
        self.aruco_pose = PoseStamped()
        self.hole_pose = PoseStamped()
        self.tool = PoseStamped()
        self.transformation_mat_aruco_to_base=None
        #self.current_TCP = 'camera'
        self.tr = TransformStamped()
        # self.set_TCP('camera')

        #How to use in all methods, Do I make them Global, add them in the constructor
        self.moveRobotCall = rospy.ServiceProxy('move_ur', moveRobot)  
        self.adjustTCPCall = rospy.ServiceProxy('move_TCP', moveRobot)
        self.relativeTCPCall = rospy.ServiceProxy('move_ur_relative', moveRobotRelative)
        #self.pub_tf = rospy.Publisher("/tf", tf2_msgs.msg.TFMessage, queue_size=1)

        #NOTE: Coordinate systems in ROS are always in 3D, and are right-handed, with X forward, Y left, and Z up. 
        self.br=tf2_ros.StaticTransformBroadcaster()
        self.tf_broadcaster = TransformBroadcaster()
        self.listener = tf.TransformListener() #listener is created -> starts receiving tf transformations over the wire, and buffers them for up to 10 seconds. 
        self.tf_buffer = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.tf_buffer) #listener is created -> starts receiving tf transformations over the wire, and buffers them for up to 10 seconds. 
                
        ##using tf_tree to broadcast a frame##
        
        
        # rospy.Subscriber("/aruco_pose", Pose, robot.callback)
        self.sub = rospy.Subscriber('/camera_pose' ,  PoseStamped, self.callback)
        self.hole_pub = rospy.Subscriber('/hole_pose', PoseStamped, self.callback2)
        self.home_pub = rospy.Publisher('/home_pose',  Pose, queue_size=1)

        self.home_pose1 = Pose()
        self.home_pose1.position.x = 0.149875247131
        self.home_pose1.position.y = -0.263940677361
        self.home_pose1.position.z = 0.450416981704
        self.home_pose1.orientation.x = 0.379002377123
        self.home_pose1.orientation.y = 0.925231495985
        self.home_pose1.orientation.z = -0.0004228853181
        self.home_pose1.orientation.w = -0.0174269371845

        # self.home_pose2 = Pose()
        # self.home_pose2.position.x = 0.1
        # self.home_pose2.position.y = -0.5
        # self.home_pose2.position.z = 0.6
        # self.home_pose2.orientation.x = 0.9672556046
        # self.home_pose2.orientation.y = 0.00052619554545
        # self.home_pose2.orientation.z = 0.061151279378
        # self.home_pose2.orientation.w = 0.0615438839872

    def move_to_home(self):
        # Move the robot to the home position
        # self.home_pub.publish(self.home_pose1)

        # home_msg = self.kinematics.pose_to_transformation_matrix(self.home_pose1)
        # print(home_msg)
        # self.kinematics.transformation_matrix_to_tf_transform(home_msg)
        
        # self.kinematics.set_transform('ur_base', 'TCP', home_msg)
        rospy.loginfo("HOME1")
        self.moveRobotCall('davis', self.home_pose1)
        rospy.loginfo("HOME2")
        # rospy.sleep(2)
        # self.home_pub.publish(self.home_pose2)

        # home_msg = self.kinematics.pose_to_transformation_matrix(self.home_pose2)
        # print(home_msg)
        # self.kinematics.transformation_matrix_to_tf_transform(home_msg)
        
        # self.kinematics.set_transform('ur_base', 'TCP', home_msg)
        # rospy.sleep(2)
        
  
    def get_arucoFrame_to_baseFrame(self):
        # Move the robot from camera frame to base frame, iterativly
        
        ##we have  pose [xyz+xyzw] of camera to aruco
        #get the translation and rotation matrix of the camera (xyz)
        transform_aruco_to_cam, tansformation_mat_aruco_to_cam = self.kinematics.receive_transform("camera", "aruco")
        
        #we have base to cam
        #get the translation and rotation matrix of the base (xyz)
        transform_cam_to_base, transformation_mat_cam_to_base = self.kinematics.receive_transform("ur_base", "camera")
        transformation_mat_aruco_to_base = np.matmul(tansformation_mat_aruco_to_cam , transformation_mat_cam_to_base)
        
        return(transformation_mat_aruco_to_base)
       



    
    def callback(self, msg):
        # The camera pose will be a msg
        rospy.loginfo(self.aruco_pose)
        self.aruco_pose = msg
        
        #to do the offset
        camera_transformation_matrix = np.eye(4)
        camera_transformation_matrix[:3, :3] = R.from_rotvec(msg[3:])
        #camera_transformation_matrix[0, 3] = msg[0] 
        #camera_transformation_matrix[1, 3] = msg[1]
        camera_transformation_matrix[2, 3] = msg[2] + 0.1
        rospy.Rate(1) # 1hz
        rospy.sleep(1)
        rospy.loginfo("Pose Recieved -- %s", self.aruco_pose)# print on the terminal
            
    def callback2(self, msg):
        # The camera pose will be a msg
        rospy.loginfo(self.hole_pose)
        self.hole_pose = msg
        
                
        #to do the offset
        camera_transformation_matrix = np.eye(4)
        camera_transformation_matrix[:3, :3] = R.from_rotvec(msg[3:])
        #camera_transformation_matrix[0, 3] = msg[0] 
        #camera_transformation_matrix[1, 3] = msg[1]
        camera_transformation_matrix[2, 3] = msg[2] - 0.1
        
        rospy.Rate(1) # 1hz
        rospy.sleep(1)
        rospy.loginfo("Pose Recieved -- %s", self.hole_pose)# print on the terminal
   
   

    def move_to_aruco(self):
        #  position and orientation from the camera
        #self.aruco_pose = rospy.wait_for_message("/aruco_pose", Pose)
        #rospy.loginfo("Pose Recieved -- \n%s", self.aruco_pose)# print on the terminal
        # self.kinematics.set_transform('ur_base', 'camera', self.aruco_pose)## --?--
        # self.kinematics.receive_transform(ur_base)
        self.moveRobotCall('camera',self.aruco_pose)
        # To get pose of camera to aruco
        camera_to_aruco = np.matmul(self.transformation_mat_aruco_to_base , self.aruco_pose)
        self.kinematics.pose_to_transformation_matrix(camera_to_aruco)
        self.kinematics.transformation_matrix_to_tf_transform(camera_to_aruco)
       
        # Move the robot to the workpiece position
       
        
        self.kinematics.set_transform('ur_base', 'camera', self.aruco_pose)
        
        self.moveRobotCall('camera', self.aruco_pose)
        
        self.kinematics.set_transform('ur_base', 'camera', self.aruco_pose)
        
        
    def move_to_hole(self):

        hole_to_base = np.matmul(self.transformation_mat_aruco_to_base , self.hole_pose)
        self.kinematics.pose_to_transformation_matrix(hole_to_base)
        self.kinematics.transformation_matrix_to_tf_transform(hole_to_base)
       
        # Move 
        self.kinematics.set_transform('aruco', 'hole', self.hole_pose)
        
        self.adjustTCPCall('tool' , self.aruco_pose)
        self.moveRobotCall('tool', self.aruco_pose)
   
        
    def start_riveting(robot):
        # Start the riveting process
        arduino = serial.Serial('COM5', 9600, timeout=1)

        time.sleep(2)
        while True:

            time.sleep(0.2)

            var = input("Enter 'r' to release and 'p' to press: ").strip().lower()

            # The strip() and lower() functions are used on the input to remove leading/trailing spaces

            # and convert it to lowercase for case-insensitive comparisons

            print("You entered:", var)

            if var == 'r':

                arduino.write(b'r')

                #The arduino.write() function requires bytes as input, so the string 'r' and 'p'

                # are converted to bytes using the b'r' and b'p' notation, respectively.

                print("RELEASED")

                time.sleep(1)

            elif var == 'p':

                arduino.write(b'p')

                print("PRESSED")

                time.sleep(1)

                robot.start_riveting()
                
    


if __name__ == '__main__':
    
    rospy.init_node('subscriberNode' , anonymous=True)
    robot = Robotic_Manipulation()
    # pose = Pose()
    # pose.position.x = -0.05
    # pose.position.y = 0
    # pose.position.z = 0
    # pose.orientation.x = 0
    # pose.orientation.y = 0
    # pose.orientation.z = 0
    # pose.orientation.w = 1
    # robot.adjustTCPCall('camera', pose)

    while not rospy.is_shutdown():
    
        robot.move_to_home()
        
        # robot.get_arucoFrame_to_baseFrame()
        # rospy.sleep(5)
        # robot.move_to_aruco()
        # rospy.sleep(5)
        # robot.move_to_hole()
        rospy.sleep(0.1)
        # rospy.spin() 

        
