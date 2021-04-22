#!/usr/bin/python
# -*- coding: utf-8 -*-

from numpy.core.numeric import NaN
import rospy
import numpy as np
import sys
import math
from time import sleep
import cv2
# from cv_bridge import CvBridge, CvBridgeError
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan, Image
# import geometry_msgs.msg


class Follow_Wall(object):
    '''
    Class to hold the brick finder
    '''

    def __init__(self):

        self.dist = 1

        self.wall_flag = 0

        self.vel_msg = Twist()

        # self.pub_vel = rospy.Publisher("navigation_velocity_smoother/raw_cmd_vel", Twist, queue_size=10)
        self.pub_vel = rospy.Publisher("/mobile_base/commands/velocity", Twist, queue_size=10)

        # self.bridge = CvBridge()

        self.image_sub = rospy.Subscriber("scan", LaserScan, self.callback)

        # self.depth_sub = rospy.Subscriber("/camera/depth/image_rect_raw", Image , self.callback)

    def callback(self,laserdata):

        # try:
        #     self.cv_depth = self.bridge.imgmsg_to_cv2(laserdata, desired_encoding="passthrough")
        # except CvBridgeError as e:
        #     print(e)    
        # self.depth_array = np.array(self.cv_depth, dtype=np.float32)
        # self.dist = self.depth_array[239,319]/1000

        self.dist = laserdata.ranges[320]
        # print(laserdata.ranges[0],laserdata.ranges[320],laserdata.ranges[639])
        # print(self.dist)

    # def follower_loop(self):

    #     if self.dist > 0.5 and not math.isnan(self.dist):
    #         self.vel_msg.angular.z = 0.0
    #         self.vel_msg.linear.x = 0.1
    #         print('forward')
    #         self.pub_vel.publish(self.vel_msg)
            
    #     else:
    #         if self.wall_flag == 0:
    #             self.vel_msg.angular.z = 1.2
    #             self.vel_msg.linear.x = 0.0
    #             while self.dist < 0.9 or math.isnan(self.dist):
    #                 self.pub_vel.publish(self.vel_msg)
    #                 print('left')
    #                 sleep(.5)
    #                 print(self.dist)
    #             self.wall_flag = 1

    #         else:
    #             self.vel_msg.angular.z = -1.2
    #             self.vel_msg.linear.x = 0.0
    #             while self.dist < 0.9 or math.isnan(self.dist):
    #                 self.pub_vel.publish(self.vel_msg)
    #                 print('right')
    #                 sleep(.5)
    #                 print(self.dist)
    #             self.wall_flag = 0
    #     # self.pub_vel.publish(self.vel_msg)

    def turn_left(self):
        self.i = 0
        self.vel_msg.angular.z = 0.5
        self.vel_msg.linear.x = 0.0
        print('Turning left 90 degrees')
        while self.i<50:
            self.pub_vel.publish(self.vel_msg)
            sleep(.1)
            self.i += 1
        sleep(1)

    def turn_right(self):
        self.i = 0
        self.vel_msg.angular.z = -0.5
        self.vel_msg.linear.x = 0.0
        print('Turning right 90 degrees')
        while self.i<50:
            self.pub_vel.publish(self.vel_msg)
            sleep(.1)
            self.i += 1
        sleep(1)

    def drive_forward(self):
        self.vel_msg.angular.z = 0.0
        self.vel_msg.linear.x = 0.1
        self.pub_vel.publish(self.vel_msg)
        self.forward_dist = self.dist


    def follower_loop(self):

        if self.dist > 0.3 and not math.isnan(self.dist) and self.wall_flag == 0:
            self.drive_forward()

        else:
            if self.wall_flag == 0:
                sleep(1)
                print('forward_dist = ', self.dist)
                print('checking left')
                self.turn_left()
                self.wall_flag = 1
                print('wall_flag = 1')
                sleep(1)

            elif self.wall_flag == 1:
                self.left_dist = self.dist
                if self.dist > 0.5 and not math.isnan(self.dist):
                    self.left_clear = 1
                    print('left_clear = 1')
                    print('left_dist = ', self.left_dist)
                else:
                    self.left_clear = 0
                    print('left_clear = 0')
                self.turn_right()
                self.turn_right()
                self.wall_flag = 2
                print('wall_flag = 2')

            elif self.wall_flag == 2:
                self.right_dist = self.dist
                if self.dist > 0.5 and not math.isnan(self.dist):
                    self.right_clear = 1
                    print('right_clear = 1')
                else:
                    self.right_clear = 0
                    print('right_clear = 0')
                self.wall_flag = 3
                print('wall_flag = 3')

            elif self.wall_flag == 3:
                if self.right_clear == 1:
                    self.wall_flag = 0
                elif self.left_clear == 1:
                    self.turn_left()
                    self.turn_left()
                    self.wall_flag = 0
                elif self.left_clear == 0 and self.right_clear == 0:
                    self.turn_right()
                    self.wall_flag = 0
                    self.left_clear = 0
                    self.right_clear = 0
                print('wall_flag reset to 0')
                





def main(args):
    rospy.init_node('Wall_Follower', anonymous=True)
    FW = Follow_Wall()

    FW.turn_right()

    while not rospy.is_shutdown():
        FW.follower_loop()
        rospy.sleep(0.1)


if __name__ == '__main__':
    main(sys.argv)


#     r = rospy.Rate(10)