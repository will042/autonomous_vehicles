#!/usr/bin/python
# -*- coding: utf-8 -*-

from math import *
import numpy as np
import cv2
from cv_bridge import CvBridge, CvBridgeError
import rospy
import sensor_msgs.point_cloud2 as pc2


import tf
from tf.transformations import quaternion_from_euler


class FindBrick(object):
    '''
    Class to hold the brick finder
    '''

    def __init__(self):
        
        # self.coord_pub = rospy.Publisher("image_topic_2",Coordinates)

        bridge = CvBridge()

        image_message = bridge.cv2_to_imgmsg(cv_image, encoding="passthrough")

        self.image_sub = rospy.Subscriber("image_topic",camera/rgb/image, self.callback)

        self.depth_sub = rospy.Subscriber("image_topic",'camera/depth_registered/image', self.callback)

    def callback(self,data):

        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            print(e)    

        threshold = 100

        red_channel = cv_image[:,:,3]

        hits = (red_channel > threshold)



        # try:
        #     self.image_pub.publish(self.bridge.cv2_to_imgmsg(cv_image, "bgr8"))
        # except CvBridgeError as e:
        #     print(e)



if __name__ == '__main__':

    # ROS initializzation
    rospy.init_node('findbrick')
    node = FindBrick()
    
    # Filter at 10 Hz
    r = rospy.Rate(10)

    while not rospy.is_shutdown():

        node.iterate()
        r.sleep()
