#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy
import random
import os

from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image
from sensor_msgs.msg import Imu
from sensor_msgs.msg import LaserScan
from sensor_msgs.msg import JointState
from nav_msgs.msg import Odometry
from std_msgs.msg import String
from cv_bridge import CvBridge, CvBridgeError

import tf

import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
import actionlib_msgs

from utils import readCsv
from camera import processImage, showImage

# Ref: https://hotblackrobotics.github.io/en/blog/2018/01/29/action-client-py/

class ConnechBot():
    def __init__(self, 
                 use_lidar=False, use_camera=False, use_imu=False,
                 use_odom=False, use_joint_states=False):
        
        # velocity publisher
        self.vel_pub = rospy.Publisher('cmd_vel', Twist,queue_size=1)
        self.client = actionlib.SimpleActionClient('move_base',MoveBaseAction)

        # lidar scan subscriber
        if use_lidar:
            self.scan = LaserScan()
            self.lidar_sub = rospy.Subscriber('scan', LaserScan, self.lidarCallback)

        # camera subscribver
        # please uncoment out if you use camera
        if use_camera:
            # for convert image topic to opencv obj
            self.img = None
            self.bridge = CvBridge()
            self.image_sub = rospy.Subscriber('image_raw', Image, self.imageCallback)
            self.image_pub = rospy.Publisher('output_image', Image, queue_size=1)

        # imu subscriber
        if use_imu:
            self.imu_sub = rospy.Subscriber('imu', Imu, self.imuCallback)

        # odom subscriber
        if use_odom:
            self.odom_sub = rospy.Subscriber('odom', Odometry, self.odomCallback)

        # joint_states subscriber
        if use_joint_states:
            self.odom_sub = rospy.Subscriber('joint_states', JointState, self.jointstateCallback)

    def setGoal(self, pose2d):
        self.client.wait_for_server()

        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = "map"
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.pose.position.x = pose2d[0]
        goal.target_pose.pose.position.y = pose2d[1]

        # Euler to Quartanion
        q=tf.transformations.quaternion_from_euler(0,0,pose2d[2])
        goal.target_pose.pose.orientation.x = q[0]
        goal.target_pose.pose.orientation.y = q[1]
        goal.target_pose.pose.orientation.z = q[2]
        goal.target_pose.pose.orientation.w = q[3]

        self.client.send_goal(goal)
        wait = self.client.wait_for_result()
        if not wait:
            rospy.logerr("Action server not available!")
            rospy.signal_shutdown("Action server not available!")
        else:
            return self.client.get_result()        

    def strategy(self):
        r = rospy.Rate(5) # change speed 5fps

        goals = readCsv(os.path.dirname(__file__) + "/input/strategy2.csv")
        for i in range(3):
            for goal in goals:
                self.setGoal(goal)
                
                

    # http://wiki.ros.org/ja/tf/Tutorials/tf%20and%20Time%20%28Python%29
    # https://makemove.hatenablog.com/entry/2014/09/23/182742
    # tfで座標情報を取得（基本 /map → /baselink）
    def listen_connechbot_pose(self, frame1, frame2):
        trans = []                  # x, y, z を格納
        rot = []                    # x, y, z, w （quaternion）を格納
        try:
            (trans, rot) = self.listener.lookupTransform(frame1, frame2, rospy.Time(0))
            return True, trans, rot
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            #continue
            return False, trans, rot

    # lidar scan topic call back sample
    # update lidar scan state
    def lidarCallback(self, data):
        self.scan = data
        rospy.loginfo(self.scan)

    # camera image call back sample
    # comvert image topic to opencv object and show
    def imageCallback(self, data):
        try:
            in_img = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            rospy.logerr(e)
        out_img = processImage(in_img)
        # self.image_pub.publish(self.bridge.cv2_to_imgmsg(out_img, 'mono8'))

        # Show processed image on a Window
        showImage(out_img)

    # imu call back sample
    # update imu state
    def imuCallback(self, data):
        self.imu = data
        rospy.loginfo(self.imu)

    # odom call back sample
    # update odometry state
    def odomCallback(self, data):
        self.pose_x = data.pose.pose.position.x
        self.pose_y = data.pose.pose.position.y
        rospy.loginfo("odom pose_x: {}".format(self.pose_x))
        rospy.loginfo("odom pose_y: {}".format(self.pose_y))

    # jointstate call back sample
    # update joint state
    def jointstateCallback(self, data):
        self.wheel_rot_r = data.position[0]
        self.wheel_rot_l = data.position[1]
        rospy.loginfo("joint_state R: {}".format(self.wheel_rot_r))
        rospy.loginfo("joint_state L: {}".format(self.wheel_rot_l))

if __name__ == '__main__':
    rospy.init_node('connechRun')
    bot = ConnechBot(use_lidar=True, use_camera=True, use_imu=True,
                     use_odom=True, use_joint_states=True)
    bot.strategy()
