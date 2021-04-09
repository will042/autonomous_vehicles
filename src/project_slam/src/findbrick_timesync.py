#!/usr/bin/python
# -*- coding: utf-8 -*-
from __future__ import print_function

from math import *
import numpy as np
import sys
import cv2
from cv_bridge import CvBridge, CvBridgeError
import rospy
import sensor_msgs.point_cloud2 as pc2
import message_filters
from std_msgs.msg import String
from sensor_msgs.msg import Image, PointCloud2



import tf
from tf.transformations import quaternion_from_euler


class FindBrick(object):
    '''
    Class to hold the brick finder
    '''

    def __init__(self):
        
        # self.coord_pub = rospy.Publisher("image_topic_2",Coordinates)
        # self.xyzrgb_sub = rospy.Subscriber("camera/depth_registered/points", PointCloud2, self.callback)

        self.bridge = CvBridge()

        # self.image_sub = rospy.Subscriber("camera/rgb/image_raw", Image, self.image_callback)

        # self.depth_sub = rospy.Subscriber("camera/depth/image_rect", Image , self.depth_callback)

        self.image_sub = message_filters.Subscriber('camera/rgb/image_rect_color',Image)
        
        self.depth_sub = message_filters.Subscriber('camera/depth/image_rect_raw',Image)

        self.ts = message_filters.TimeSynchronizer([self.image_sub, self.depth_sub], 1)

        self.ts.registerCallback(self.callback)


    def callback(self, image, depth):

        try:
            self.cv_image = self.bridge.imgmsg_to_cv2(image, "bgr8")
        except CvBridgeError as e:
            print(e)    

        self.hsv = cv2.cvtColor(self.cv_image, cv2.COLOR_BGR2HSV)

        cv2.imshow("RGB", self.hsv)
        cv2.waitKey(3)

    # def image_callback(self,data):

    #     try:
    #         self.cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
    #     except CvBridgeError as e:
    #         print(e)    

    #     self.hsv = cv2.cvtColor(self.cv_image, cv2.COLOR_BGR2HSV)

    #     # self.hits = (self.red_channel > self.threshold
    #     cv2.imshow("RGB", self.hsv)
    #     cv2.waitKey(3)

    # def depth_callback(self,data):

    #     try:
    #         self.cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
    #     except CvBridgeError as e:
    #         print(e)    

    #     self.hsv = cv2.cvtColor(self.cv_image, cv2.COLOR_BGR2HSV)

    #     # self.hits = (self.red_channel > self.threshold
    #     cv2.imshow("RGB", self.hsv)
    #     cv2.waitKey(3)


        # try:
        #     self.image_pub.publish(self.bridge.cv2_to_imgmsg(cv_image, "bgr8"))
        # except CvBridgeError as e:
        #     print(e)


def main(args):
    FB = FindBrick()
    rospy.init_node('BrickFinder', anonymous=True)
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main(sys.argv)

