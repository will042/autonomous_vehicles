#!/usr/bin/python
# -*- coding: utf-8 -*-

import rospy
import numpy as np
import sys
import cv2
from cv_bridge import CvBridge, CvBridgeError
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan, Image
# import geometry_msgs.msg


class Follow_Wall(object):
    '''
    Class to hold the brick finder
    '''

    def __init__(self):

        self.pub_vel = rospy.Publisher("navigation_velocity_smoother/raw_cmd_vel", Twist, queue_size=0)

        self.bridge = CvBridge()

        # self.image_sub = rospy.Subscriber("scan", LaserScan, self.callback)

        self.depth_sub = rospy.Subscriber("/camera/depth/image_rect_raw", Image , self.callback)

    def callback(self,laserdata):

        try:
            self.cv_depth = self.bridge.imgmsg_to_cv2(laserdata, desired_encoding="passthrough")
        except CvBridgeError as e:
            print(e)    

        self.depth_array = np.array(self.cv_depth, dtype=np.float32)

        self.dist = self.depth_array[239,319]/1000

        # print(laserdata.ranges[2], laserdata.ranges[320], laserdata.ranges[630])






def main(args):
    rospy.init_node('Wall_Follower', anonymous=True)
    FW = Follow_Wall()

    rospy.spin()

if __name__ == '__main__':
    main(sys.argv)


#     r = rospy.Rate(10)