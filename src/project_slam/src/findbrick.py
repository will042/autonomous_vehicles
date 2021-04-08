#!/usr/bin/python
# -*- coding: utf-8 -*-
# from __future__ import print_function

from math import *
import numpy as np
import sys
import cv2
from cv_bridge import CvBridge, CvBridgeError
import rospy
import sensor_msgs.point_cloud2 as pc2
from std_msgs.msg import String
from sensor_msgs.msg import Image, PointCloud2, LaserScan

import tf
from tf.transformations import quaternion_from_euler

class FindBrick(object):
    '''
    Class to hold the brick finder
    '''

    def __init__(self):
        
        # self.coord_pub = rospy.Publisher("image_topic_2",Coordinates)
        print('initializing...')
        self.bridge = CvBridge()

        self.image_sub = rospy.Subscriber("camera/rgb/image_raw", Image, self.image_callback)

        # self.xyzrgb_sub = rospy.Subscriber("camera/depth_registered/points", PointCloud2, self.callback)

        self.depth_sub = rospy.Subscriber("scan", LaserScan , self.depth_callback)

        self.x = 0

    def image_callback(self,image_data):
        self.x += 1
        try:
            self.cv_image = self.bridge.imgmsg_to_cv2(image_data, "bgr8")
        except CvBridgeError as e:
            print(e)    

        self.hsv = cv2.cvtColor(self.cv_image, cv2.COLOR_BGR2HSV)

        # self.hits = (self.red_channel > self.threshold
        cv2.imshow("RGB", self.hsv)
        cv2.waitKey(3)


    def depth_callback(self,depth_data):
        print('hello')
        print(depth_data.header)



def main(args):
    FB = FindBrick()
    rospy.init_node('BrickFinder', anonymous=True)
    # try:
    #     # rospy.spin()
    while not rospy.is_shutdown():
        print(FB.x)
        rospy.sleep(1)
    # except KeyboardInterrupt:
    #     print("Shutting down")
    # cv2.destroyAllWindows()

if __name__ == '__main__':
    main(sys.argv)


# if __name__ == '__main__':

#     # ROS initializzation
#     rospy.init_node('findbrick')
#     node = FindBrick()
    
#     # Filter at 10 Hz
#     r = rospy.Rate(10)

#     while not rospy.is_shutdown():

#         node.iterate()
#         r.sleep()
