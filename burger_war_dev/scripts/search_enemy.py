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

            # judgment of inside / outside the field
            if self.is_point_enemy(temp_x, temp_y) == False:
                continue

            # enemy_frame_name = self.robot_namespace + '/enemy_' + str(num)
            # map_frame_name   = self.robot_namespace + "/map"
            enemy_frame_name = '/enemy_' + str(num)
            map_frame_name   = "/map"
            self.tf_broadcaster.sendTransform((temp_x,temp_y,0), (0,0,0,1), rospy.Time.now(), enemy_frame_name, map_frame_name)

            # calculate the distance to the enemybot
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

        # Publish an enemybot position and distance
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
            self.pub_robot2enemy.publish(closest_enemy_len)

    def is_point_enemy(self, point_x, point_y):

#    1 ~ 4 : conner obstacle | position (x, y) = (±0.53, ±0.53)
#    5     : center obstacle | position (x, y) = ( 0, 0)

        # 
        thresh_corner = 0.20
        thresh_center = 0.35

        #Threshhold of enemy / object
        if   point_y > (-point_x + 1.55):
            return False
        elif point_y < (-point_x - 1.55):
            return False
        elif point_y > ( point_x + 1.55):
            return False
        elif point_y < ( point_x - 1.55):
            return False
            
        p1 = math.sqrt(pow((point_x - 0.53), 2) + pow((point_y - 0.53), 2))
        p2 = math.sqrt(pow((point_x - 0.53), 2) + pow((point_y + 0.53), 2))
        p3 = math.sqrt(pow((point_x + 0.53), 2) + pow((point_y - 0.53), 2))
        p4 = math.sqrt(pow((point_x + 0.53), 2) + pow((point_y + 0.53), 2))
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
