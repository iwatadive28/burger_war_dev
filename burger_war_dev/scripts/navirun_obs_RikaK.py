#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy
import random
import rosparam
from math import radians, degrees, atan2

import tf

from geometry_msgs.msg import Twist

import tf
import tf

import numpy as np
import math

pi = math.pi
PI = math.pi

import actionlib
from actionlib import SimpleActionClient, GoalStatus
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
import actionlib_msgs

from geometry_msgs.msg import PoseWithCovarianceStamped
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry   

from std_msgs.msg import Bool
from std_msgs.msg import Float64

from Enemy_detector_obs import EnemyDetector

# Ref: https://hotblackrobotics.github.io/en/blog/2018/01/29/action-client-py/

#from std_msgs.msg import String
#from sensor_msgs.msg import Image
#from cv_bridge import CvBridge, CvBridgeError
#import cv2


TARGET_TH = (
    (-PI/4, -PI/4, -PI/2, -PI/2, -PI*3/4, -PI*3/4, -PI*3/4, -PI*3/4),
    (-PI/4, -PI/4, -PI/3, -PI/2, -PI*3/5, -PI*3/4, -PI*3/4, -PI*3/4),
    (-PI/4, -PI/4, -PI/6,     0,   -PI/2, -PI*3/4,     -PI,  PI*3/4),
    (-PI/4, -PI/5,     0,     0,      PI,  PI*6/10,  PI*3/4,    PI/2),
    (    0,     0,  PI/2,  PI/2,      PI,  PI*3/4,  PI*3/4,    PI/2),
    (    0,  PI/4,  PI/3,  PI/2,  PI*5/6,  PI*3/4,  PI*3/4,  PI*3/4),
    ( PI/4,  PI/4,  PI/4,  PI/3,  PI*5/6,    PI/2,  PI*3/4,  PI*3/4),
    ( PI/4,  PI/4,  PI/4,  PI/4,    PI/3,    PI/2,  PI*3/4,  PI*3/4),
)
WIDTH = 1.2 * (2 **0.5) # [m]

def get_goals(my_color = 'r', rotation="CW"):
    if my_color == 'b':
        symbol = -1
        th     = pi
    else:
        symbol = 1
        th     = 0

    # rotation = 'CW'  # 回転方向を変える @en

    if rotation == 'CW':
        symbol_2 = -1
    else:
        symbol_2 = 1

    #print("get_goals", my_color, symbol, th, symbol_2)
         
    # 12x3 (x,y,yaw) 
    goal_xyyaw = np.array([ 
            [symbol * -0.8  , symbol * 0     , np.mod(0      + th ,2*pi) ], # (1） 
            [symbol * -0.8  , symbol * -0.2  ,-np.mod(-pi/4  + th ,2*pi) ], 
            [symbol * -0.8  , symbol * 0     , np.mod(pi/2   + th ,2*pi) ],
            [symbol * -0.8  , symbol *  0.2  , np.mod(pi/3   + th ,2*pi) ],
            [symbol * -0.5  , symbol *  0.2  , np.mod(0      + th ,2*pi) ],
            [symbol * 0     , symbol * 0.4   , np.mod(pi*4/5 + th ,2*pi) ], # (2)
            [symbol * 0     , symbol * 0.4   , np.mod(-pi/2  + th ,2*pi) ],
            [symbol * 0.2   , symbol * 0.4   , np.mod( 0     + th ,2*pi) ],
            [symbol * 0.2   , symbol * 0.4   ,-np.mod(pi/3   + th ,2*pi) ],
            [symbol * 0.5   , symbol *   0   , np.mod(pi     + th ,2*pi) ], #（3）
            [symbol * 0.5   , symbol *   0   , np.mod(-pi/2  + th ,2*pi) ],
            [symbol * 0.2   , symbol * -0.4  , np.mod( pi    + th ,2*pi) ],
            [symbol * 0.2   , symbol * -0.4  , np.mod(-pi/3  + th ,2*pi) ],
            [symbol * 0     , symbol * -0.4  , np.mod( 0     + th ,2*pi) ], # (4)
            [symbol * 0     , symbol * -0.4  , np.mod(-pi*4/5+ th ,2*pi) ],
            [symbol * 0     , symbol * -0.4  , np.mod( pi    + th ,2*pi) ],
            [symbol * -0.5  , symbol * 0     , np.mod( pi    + th ,2*pi) ], # (5) 
            [symbol * -1.40 , symbol * 0     , np.mod( 0     + th ,2*pi) ]
        ])     
    return goal_xyyaw 

def num2mvstate(i):
    return ["PENDING", "ACTIVE", "RECALLED", "REJECTED", "PREEMPTED", "ABORTED", "SUCCEEDED", "LOST"][i]
  
class NaviBot():
    def __init__(self):

        self.robot_namespace = rospy.get_param('~robot_namespace')
        print(self.robot_namespace)
        # velocity publisher
        self.vel_pub = rospy.Publisher('cmd_vel', Twist,queue_size=1)
        self.client  = actionlib.SimpleActionClient('move_base',MoveBaseAction)
        #self.tfBuffer = tf.Buffer()

        self.enemy_detector = EnemyDetector()
        self.listener = tf.TransformListener()
        self.goals = get_goals()
        # robot state 'inner' or 'outer'
        self.state = 'inner' 
        # robot wheel rot 
        self.wheel_rot_r = 0
        self.wheel_rot_l = 0
        self.pose_x = 0
        self.pose_y = 0
        self.th = 0

        self.k = 0.5
        self.near_wall_range = 0.2  # [m]
        self.speed = 0.07

        self.goals = get_goals()
        self.goalcounter = 0
        self.goalcounter_prev = -1

        self.pose_twist = Twist()
        self.pose_twist.linear.x = self.speed; self.pose_twist.linear.y = 0.; self.pose_twist.linear.z = 0.
        self.pose_twist.angular.x = 0.; self.pose_twist.angular.y = 0.; self.pose_twist.angular.z = 0.

        self.is_near_wall = False
        self.is_second_lap = False

        self.is_near_enemy = False
        self.enemy_direction = None
        self.enemy_dist = 0
        self.enemy_rad = 0
        self.near_enemy_twist = Twist()
        self.near_enemy_twist.linear.x = self.speed 
        self.near_enemy_twist.linear.y = 0.
        self.near_enemy_twist.linear.z = 0.
        self.near_enemy_twist.angular.x = 0. 
        self.near_enemy_twist.angular.y = 0.
        self.near_enemy_twist.angular.z = 0.
        self.is_initialized_pose = True
        
        # lidar scan
        self.scan = []
        self.odom = Odometry()
        self.odom_prev = Odometry()
        self.odom_diff = Odometry()        

        # publisher
        self.vel_pub  = rospy.Publisher('cmd_vel', Twist, queue_size=1)
        self.odom_pub = rospy.Publisher('odom', Odometry, queue_size= 1)
        
        # subscriber
        self.is_enemy_detecting = False
        rospy.Subscriber('enemy_detected', Bool, self.save_detected )
        rospy.Subscriber('enemy_dist', Float64, self.save_detected2 )
        rospy.Subscriber('enemy_rad', Float64, self.save_detected3 )
        rospy.Subscriber('odom', Odometry, self.save_detected4 )

    def save_detected(self, data):
        self.is_enemy_detecting = data.data
        #print("data_updated")
    def save_detected2(self, data):
        self.enemy_dist = data.data
        #print("data_updated")
    def save_detected3(self, data):
        self.enemy_rad = data.data
        #print("data_updated")
    def save_detected4(self, data):
        self.odom = data
        self.odom.pose.pose.position.x += self.odom_diff.pose.pose.position.x
        self.odom.pose.pose.position.y += self.odom_diff.pose.pose.position.y
        #print("data_updated")

    def setGoal(self,x,y,yaw):
        self.client.wait_for_server()

        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = "map"
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.pose.position.x = x
        goal.target_pose.pose.position.y = y

        # Euler to Quartanion
        q=tf.transformations.quaternion_from_euler(0,0,yaw)        
        goal.target_pose.pose.orientation.x = q[0]
        goal.target_pose.pose.orientation.y = q[1]
        goal.target_pose.pose.orientation.z = q[2]
        goal.target_pose.pose.orientation.w = q[3]

        self.client.send_goal(goal)
        
        def active_cb():
            # rospy.loginfo("active_cb. Goal pose is now being processed by the Action Server...")
            return

        def feedback_cb( feedback):
            #To print current pose at each feedback:
            #rospy.loginfo("Feedback for goal "+str(self.goal_cnt)+": "+str(feedback))
            # rospy.loginfo("feedback_cb. Feedback for goal pose received{}".format(feedback))
            return

        def done_cb(status, result):
            if status is not GoalStatus.PREEMPTED:
                self.goalcounter += 1
                if self.goalcounter == len(self.goals):
                    self.is_second_lap = True
                self.goalcounter %= len(self.goals)

                # 2周目の場合は、最初の方のマーカーをとばす
                if self.is_second_lap and (self.goalcounter in [0, 1, 2]):
                    self.goalcounter = 3
            #rospy.loginfo("done_cb. status:{} result:{}".format(num2mvstate(status), result))

        self.client.send_goal(goal, done_cb=done_cb, active_cb=active_cb, feedback_cb=feedback_cb)
        return

    def strategy(self):
        r = rospy.Rate(5) # change speed 5fps
        goal_xyyaw = self.goals
        
        map_name=self.robot_namespace+'/map'
        is_enemy_detected, x, y = self.getEnemyPos(map_name)
        print(is_enemy_detected)
        is_patrol_mode_prev = False
        is_patrol_mode = True
        # cnt = 0
        # flg = True

        while(True):
            r.sleep()
            is_patrol_mode = True
            detect_inner_th = 1.1 # 1.2
            detect_outer_th = 1.1 # 0.7
            if not self.is_enemy_detecting:
                is_patrol_mode = True
            elif self.is_enemy_detecting and detect_inner_th > self.enemy_dist:
                is_patrol_mode = False
            elif not is_patrol_mode and detect_outer_th < self.enemy_dist:
                is_patrol_mode = True

            # 移動実施
            if is_patrol_mode and (not is_patrol_mode_prev or (self.goalcounter is not self.goalcounter_prev)):
                print("次のゴールに行くぜ〜")
                # 新たに巡回モードに切り替わった瞬間及びゴール座標が変わった時
                # goalcounterのゴール座標をセット
                self.setGoal(goal_xyyaw[self.goalcounter][0], goal_xyyaw[self.goalcounter][1], goal_xyyaw[self.goalcounter][2])
                rospy.loginfo( num2mvstate(self.client.get_state()))
                self.goalcounter_prev = self.goalcounter
            
            elif is_patrol_mode:
                # print("巡回するぜ〜")
                # 巡回モード最中。CBが来るまで何もしない。
                pass
            else:
                print("Yeah!! Enemy Det!!敵を見るぜ〜")
                # 敵の方向を向くモード
                self.client.cancel_all_goals()
                twist = Twist()
                # twist.angular.z = radians(degrees(self.near_enemy_twist.angular.z))
                # twist.linear.x  = self.speed
                # self.vel_pub.publish(twist)
                
                twist = Twist()
                #twist.linear.x = 0.1
                twist.angular.z = self.enemy_rad
                self.vel_pub.publish(twist)

                
            is_patrol_mode_prev = is_patrol_mode

    def getEnemyPos(self, frame):
        try:
            enemy_closest_name=self.robot_namespace+'/enemy_closest'
            trans_stamped = self.listener.lookupTransform(frame, enemy_closest_name, rospy.Time())
            #trans = trans_stamped.transform
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException) as e:
            return False, 0, 0
        return True, trans_stamped[0], trans_stamped[1]

    def getEnemyDistRad(self):
        try:
            # <class 'geometry_msgs.msg._TransformStamped.TransformStamped'>
            base_footprint_name=self.robot_namespace+'/base_footprint'
            enemy_closest_name=self.robot_namespace+'/enemy_closest'
            trans_stamped = self.listener.lookupTransform(base_footprint_name, enemy_closest_name, rospy.Time())
            #trans = trans_stamped.transform
            # trans = self.tfBuffer.lookup_transform('enemy_closest', "base_footprint", rospy.Time(), rospy.Duration(4))
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException) as e:
            # rate.sleep()
            # rospy.logwarn(e)
            return False, 0, 0
        # rospy.loginfo(trans)
        #print(trans_stamped)
        
        dist = (trans_stamped[0][0]**2 + trans_stamped[0][1]**2)**0.5
        rad = atan2(trans_stamped[0][1], trans_stamped[0][0])
        # print ("trans.translation.x:{}, trans.translation.y:{}".format(trans.translation.x, trans.translation.y))

        # rot = trans.rotation
        # rad = tf.transformations.euler_from_quaternion([rot.x, rot.y, rot.z, rot.w])[2]

        return True, dist, rad
    
if __name__ == '__main__':
    rospy.init_node('navirun')
    bot = NaviBot()
    bot.strategy()