#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy
import random
import os
import math
import cv2
import tf
import angles

from geometry_msgs.msg import Twist, Pose, Point, Quaternion, PoseWithCovarianceStamped
from sensor_msgs.msg import Image
from sensor_msgs.msg import Imu
from sensor_msgs.msg import LaserScan
from sensor_msgs.msg import JointState
from nav_msgs.msg import Odometry
from std_msgs.msg import String
from cv_bridge import CvBridge, CvBridgeError

import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
import actionlib_msgs

from utils import readCsv
from camera import processImage, showImage

# Ref: https://hotblackrobotics.github.io/en/blog/2018/01/29/action-client-py/
# Respect seigot

class ConnechBot():
    def __init__(self, 
                 use_lidar=False, use_camera=False, use_imu=False,
                 use_odom=True, use_joint_states=False):

        def get_goallist(self):
            goalpoints = readCsv(os.path.dirname(__file__) + "/input/strategy3.csv")
            return goalpoints
        
        self.goals = get_goallist(self)

        self.yellow_detected = False

        # velocity publisher
        self.vel_pub = rospy.Publisher('cmd_vel', Twist,queue_size=1)
        self.client = actionlib.SimpleActionClient('move_base',MoveBaseAction)
        self.move_state = self.client.get_state()
        self.listener = tf.TransformListener()
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
            # self.image_pub = rospy.Publisher('output_image', Image, queue_size=1)

        # imu subscriber
        if use_imu:
            self.imu_sub = rospy.Subscriber('imu', Imu, self.imuCallback)

        # odom subscriber
        if use_odom:
            self.listener = tf.TransformListener()
            self.enemy_position = Odometry()
            self.enemy_info = [0.0, 0.0, 0.0]
            self.detect_counter = 0
            self.goal_pointID = 0
            self.escape_pointID = -1
            rospy.Subscriber('enemy_position', Odometry, self.enemylocationCallback)
            self.odom_sub = rospy.Subscriber('odom', Odometry, self.odomCallback)

        # joint_states subscriber
        if use_joint_states:
            self.odom_sub = rospy.Subscriber('joint_states', JointState, self.jointstateCallback)
            
    # Respect seigot
    def get_rosparam(self):
        self.robot_namespace = rospy.get_param('~robot_namespace')

    def get_move_state(self):
        self.move_state = self.client.get_state()
        return self.move_state

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

    def canselGoal(self):
        # stop navigating
        self.client.cancel_all_goals()
        return
    
    def patrol(self):
        r = rospy.Rate(5) # change speed 5fps

        if self.escape_pointID == -1:
            if self.goal_pointID == len(self.goals) - 1:     
                self.goal_pointID = 0           # reset self.goal_pointID
            goal = self.goal_pointID
            self.goal_pointID = self.goal_pointID + 1
        else:
            goal = self.escape_pointID + 1
            self.goal_pointID = goal            # set next goalpoint
            self.escape_pointID = -1            # reset self.escape_pointID

        self.setGoal(self.goals[goal])

    def escape(self):
        escape_goals = self.setEscapepoint()
        rospy.loginfo(self.goals[escape_goals])
        self.setGoal(self.goals[escape_goals])

    def setEscapepoint(self):
        enemy_x = self.enemy_info[0]            # detecting enemybot x
        enemy_y = self.enemy_info[1]            # detecting enemybot y
        enemy_direction = self.enemy_info[2]    # detecting enemybot direction [deg]
        
        # select escape point
        if enemy_x > 0 and enemy_y > 0:
            if enemy_x >= enemy_y*0.5:
                self.escape_pointID = 18
            else:
                self.escape_pointID = 16

        elif enemy_x < 0 and enemy_y > 0:
            if enemy_x >= -enemy_y*0.5:
                self.escape_pointID = 16
            else:
                self.escape_pointID = 12
        elif enemy_x < 0 and enemy_y < 0:
            if enemy_x >= enemy_y*0.5:
                pass
                self.escape_pointID = 5
            else:
                self.escape_pointID = 12
        elif enemy_x > 0 and enemy_y < 0:
            if enemy_x >= -enemy_y*0.5:
                self.escape_pointID = 18
            else:
                self.escape_pointID = 1
        return self.escape_pointID

    def detect_enemy(self):
        state, distance, direction_deff = self.detect_enemylocation()
        return state, distance, direction_deff

    def listen_selflocation(self, frame1, frame2):
        trans = []
        rot = []   # quatarnion
        try:
            (trans, rot) = self.listener.lookupTransform(frame1, frame2, rospy.Time(0))
            return True, trans, rot
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            #continue
            return False, trans, rot

    # Respect seigot
    def detect_enemylocation(self):
        # data management
        time_width = 5
        counter_width = 4
        time_diff = rospy.Time.now().to_sec() - self.enemy_position.header.stamp.to_sec()
        if time_diff > time_width:
            self.detect_counter = 0
            return False, 0.0, 0.0
        else:
            self.detect_counter = self.detect_counter+1
            if self.detect_counter < counter_width:
                return False, 0.0, 0.0

        # set flame
        map_frame = "map"
        link_frame = "base_link"
        
        # get self position
        valid, trans, rot = self.listen_selflocation(map_frame, link_frame)
        if valid == False:
            # rospy.loginfo("Here tf False")
            return False, 0.0, 0.0

        # Calculating the distance from enemybot 
        dx = self.enemy_position.pose.pose.position.x - trans[0]
        dy = self.enemy_position.pose.pose.position.y - trans[1]
        distance = math.sqrt( pow(dx, 2) + pow(dy, 2) )
        
        # Calculating the direction from enemybot
        direction = math.atan2(dx, dy)                          # radians
        deg_direction = (direction)*180/3.14159                 # radians to degree

        roll, pitch, yaw = tf.transformations.euler_from_quaternion(rot)
        direction_diff = direction - yaw                        # radians
        
        self.enemy_info = [dx, dy, deg_direction]               # set enemybot imformation     

        rospy.loginfo("distance: {}".format(distance))
        rospy.loginfo("direction: {}".format(deg_direction))
        # rospy.loginfo("direction_deff: {}".format(direction_diff))
        return True, distance, direction_diff

    # lidar scan topic call back sample
    # update lidar scan state
    def lidarCallback(self, data):
        self.scan = data
        # rospy.loginfo(self.scan)

    # camera image call back sample
    # comvert image topic to opencv object and show
    def imageCallback(self, data):
        try:
            in_img = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            rospy.logerr(e)

        # color detection
        self.yellow_flag, yellow_img = processImage(in_img, "yellow")
        # self.blue_flag, blue_img = processImage(in_img, "blue")
        # self.green_flag, green_img = processImage(in_img, "green")
        # self.red_flag, red_img = processImage(in_img, "red")
        # rospy.loginfo("YELLOW: {}".format(self.yellow_flag))
        # rospy.loginfo("blue: {}".format(self.blue_flag))
        # rospy.loginfo("green: {}".format(self.green_flag))
        # rospy.loginfo("red: {}".format(self.red_flag))

        # Show processed image on a Window
        showImage(yellow_img)
        # showImage(blue_img)
        # showImage(green_img)
        # showImage(red_img)

        if self.yellow_flag:
            print("YELLOW IS DETECTED!!!")
            if not self.yellow_detected:
                self.client.cancel_all_goals()
                self.yellow_detected = True
            twist = Twist()
            twist.angular.z = -10
            self.vel_pub.publish(twist)
        else:
            self.yellow_detected = False

    # imu call back sample
    # update imu state
    def imuCallback(self, data):
        self.imu = data
        # rospy.loginfo(self.imu)

    # odom call back sample
    # update odometry state
    def odomCallback(self, data):
        self.pose_x = data.pose.pose.position.x
        self.pose_y = data.pose.pose.position.y
        # rospy.loginfo("odom pose_x: {}".format(self.pose_x))
        # rospy.loginfo("odom pose_y: {}".format(self.pose_y))

    # jointstate call back sample
    # update joint state
    def jointstateCallback(self, data):
        self.wheel_rot_r = data.position[0]
        self.wheel_rot_l = data.position[1]
        # rospy.loginfo("joint_state R: {}".format(self.wheel_rot_r))
        # rospy.loginfo("joint_state L: {}".format(self.wheel_rot_l))

    def enemylocationCallback(self, position):
        self.enemy_position = position
        # rospy.loginfo("enemypos_x: {}".format(self.enemy_position.pose.pose.position.x))
        # rospy.loginfo("enemypos_y: {}".format(self.enemy_position.pose.pose.position.y))

    def set_status(self):
        # get enemy_detector result
        state, distance, direction_deff = self.detect_enemy()

        # non-detecting enemybot
        if state == False:
            rospy.loginfo("PATROL_MODE")
            self.patrol()
        else:
            if distance > 0.8: 
                rospy.loginfo("PATROL_MODE")
                self.patrol()    
            # detected enemybot
            else:
                rospy.loginfo("ESCAPE_MODE")
                self.canselGoal()
                self.escape()

if __name__ == '__main__':
    rospy.init_node('connechRun')
    node = ConnechBot()
    bot = ConnechBot(use_lidar=True, use_camera=True, use_imu=True, use_odom=True, use_joint_states=True)
    rate = rospy.Rate(30)
    while not rospy.is_shutdown():
        node.set_status()
        rate.sleep()
