#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy
import random
import os

from geometry_msgs.msg import Twist

import tf

import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
import actionlib_msgs

from utils import readCsv

# Ref: https://hotblackrobotics.github.io/en/blog/2018/01/29/action-client-py/

#from std_msgs.msg import String
#from sensor_msgs.msg import Image
#from cv_bridge import CvBridge, CvBridgeError
#import cv2


class NaviBot():
    def __init__(self):
        
        # velocity publisher
        self.vel_pub = rospy.Publisher('cmd_vel', Twist,queue_size=1)
        self.client = actionlib.SimpleActionClient('move_base',MoveBaseAction)
        self.pi = 3.1415

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

        goals = readCsv(os.path.dirname(__file__) + "/input/strategy.csv")
        for i in range(3):
            for goal in goals:
                self.setGoal(goal)

if __name__ == '__main__':
    rospy.init_node('navirun')
    bot = NaviBot()
    bot.strategy()
