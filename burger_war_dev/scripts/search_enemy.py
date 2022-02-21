#!/usr/bin/env python
# -*- coding: utf-8 -*-

#respect team Rabbits 
#respect seigot/adelie7273
#https://github.com/seigot/burger_war_dev/blob/main/burger_war_dev/scripts/enemy_detector.py

import rospy
import sys
import math
import tf
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Quaternion
import roslib.packages
from obstacle_detector.msg import Obstacles
from std_msgs.msg          import Float32

class SearchEnemy:
    def __init__(self):
        self.tf_broadcaster  = tf.TransformBroadcaster()
        self.tf_listener     = tf.TransformListener()
        self.sub_obstacles   = rospy.Subscriber('obstacles', Obstacles, self.obstacles_callback)
        self.pub_robot2enemy = rospy.Publisher('robot2enemy', Float32, queue_size=10)
        self.pub_enemy_position = rospy.Publisher('enemy_position', Odometry, queue_size=10)
        self.robot_namespace = rospy.get_param('~robot_namespace', '')
        self.enemy_pos = Odometry()
        # self.enemy_pos.header.frame_id = self.robot_namespace+'/map'
        self.enemy_pos.header.frame_id = '/map'

    def obstacles_callback(self, msg):

        closest_enemy_len = sys.float_info.max
        closest_enemy_x   = 0
        closest_enemy_y   = 0

        for num in range(len(msg.circles)):

            temp_x = msg.circles[num].center.x
            temp_y = msg.circles[num].center.y

            #\u30d5\u30a3\u30fc\u30eb\u30c9\u5185\u306e\u30aa\u30d6\u30b8\u30a7\u30af\u30c8\u3067\u3042\u308c\u3070\u30d1\u30b9
            if self.is_point_enemy(temp_x, temp_y) == False:
                continue

            #\u6575\u306e\u5ea7\u6a19\u3092TF\u3067broadcast
            # enemy_frame_name = self.robot_namespace + '/enemy_' + str(num)
            # map_frame_name   = self.robot_namespace + "/map"
            enemy_frame_name = '/enemy_' + str(num)
            map_frame_name   = "/map"
            self.tf_broadcaster.sendTransform((temp_x,temp_y,0), (0,0,0,1), rospy.Time.now(), enemy_frame_name, map_frame_name)

            #\u30ed\u30dc\u30c3\u30c8\u304b\u3089\u6575\u307e\u3067\u306e\u8ddd\u96e2\u3092\u8a08\u7b97
            try:
                # target_frame_name = self.robot_namespace + '/enemy_' + str(num)
                # source_frame_name = self.robot_namespace + "/base_footprint"
                target_frame_name = '/enemy_' + str(num)
                source_frame_name = "/base_footprint"
                (trans,rot) = self.tf_listener.lookupTransform(source_frame_name, target_frame_name, rospy.Time(0))
            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                continue

            len_robot2enemy = math.sqrt(pow(trans[0],2) + pow(trans[1],2))

            if closest_enemy_len > len_robot2enemy:
                closest_enemy_len = len_robot2enemy
                closest_enemy_x   = temp_x
                closest_enemy_y   = temp_y

        #\u6575\u3092\u691c\u51fa\u3057\u3066\u3044\u308b\u5834\u5408\u3001\u305d\u306e\u5ea7\u6a19\u3068\u8ddd\u96e2\u3092\u51fa\u529b
        if closest_enemy_len < sys.float_info.max:

            # map_frame_name   = self.robot_namespace + "/map"
            # enemy_frame_name = self.robot_namespace + "/enemy_closest"
            map_frame_name   = "/map"
            enemy_frame_name = "/enemy_closest"
            self.tf_broadcaster.sendTransform((closest_enemy_x,closest_enemy_y,0), (0,0,0,1), rospy.Time.now(), enemy_frame_name, map_frame_name)

            self.enemy_pos.header.stamp = rospy.Time.now()
            self.enemy_pos.pose.pose.position.x = closest_enemy_x
            self.enemy_pos.pose.pose.position.y = closest_enemy_y
            yaw = math.atan2(msg.circles[num].velocity.y, msg.circles[num].velocity.x)
            q = tf.transformations.quaternion_from_euler(0, 0, yaw)
            quaternion = Quaternion(x=q[0], y=q[1], z=q[2], w=q[3])
            self.enemy_pos.pose.pose.orientation = quaternion
            self.pub_enemy_position.publish(self.enemy_pos)

            #\u30ed\u30dc\u30c3\u30c8\u304b\u3089\u6575\u307e\u3067\u306e\u8ddd\u96e2\u3092publish
            self.pub_robot2enemy.publish(closest_enemy_len)

    def is_point_enemy(self, point_x, point_y):

#             *        \u2190 +x
#           \uff0f  \uff3c      \u2191 +y
#         \uff0f      \uff3c
#       \uff0f  2    3  \uff3c
#     \uff0f              \uff3c
#    *        5        *
#     \uff3c              \uff0f
#       \uff3c  1    4  \uff0f
#         \uff3c      \uff0f
#           \uff3c  \uff0f
#             *
#    1 ~ 4 : conner obstacle | position (x, y) = (±0.53, ±0.53)
#    5     : center obstacle | position (x, y) = ( 0, 0)

        #\u30d5\u30a3\u30fc\u30eb\u30c9\u5185\u306e\u7269\u4f53\u3067\u306a\u3044\u3001\u6575\u3068\u5224\u5b9a\u3059\u308b\u95be\u5024\uff08\u534a\u5f84\uff09
        thresh_corner = 0.20
        thresh_center = 0.35

        #\u30d5\u30a3\u30fc\u30eb\u30c9\u5185\u304b\u30c1\u30a7\u30c3\u30af
        if   point_y > (-point_x + 1.55):
            return False
        elif point_y < (-point_x - 1.55):
            return False
        elif point_y > ( point_x + 1.55):
            return False
        elif point_y < ( point_x - 1.55):
            return False

        #\u30d5\u30a3\u30fc\u30eb\u30c9\u5185\u306e\u7269\u4f53\u3067\u306a\u3044\u304b\u30c1\u30a7\u30c3\u30af
        p1 = math.sqrt(pow((point_x + 0.53), 2) + pow((point_y - 0.53), 2))
        p2 = math.sqrt(pow((point_x + 0.53), 2) + pow((point_y + 0.53), 2))
        p3 = math.sqrt(pow((point_x - 0.53), 2) + pow((point_y + 0.53), 2))
        p4 = math.sqrt(pow((point_x - 0.53), 2) + pow((point_y - 0.53), 2))
        p5 = math.sqrt(pow(point_x         , 2) + pow(point_y         , 2))

        if p1 < thresh_corner or p2 < thresh_corner or p3 < thresh_corner or p4 < thresh_corner or p5 < thresh_center:
            return False
        else:
            return True

if __name__ == '__main__':
    rospy.init_node('search_enemy')
    ed = SearchEnemy()
    rospy.loginfo("Enemy Serch Start.")
    rospy.spin()
