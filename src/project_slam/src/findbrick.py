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
np.set_printoptions(threshold=sys.maxsize)
# import tf
import tf2_ros
import tf2_msgs.msg
import geometry_msgs.msg
from tf.transformations import quaternion_from_euler




class FindBrick(object):
    '''
    Class to hold the brick finder
    '''

    def __init__(self):

        self.pub_tf = rospy.Publisher("/tf", tf2_msgs.msg.TFMessage, queue_size=1)

        self.bridge = CvBridge()

        self.image_sub = rospy.Subscriber("camera/rgb/image_rect_color", Image, self.image_callback)

        # self.xyzrgb_sub = rospy.Subscriber("camera/depth_registered/points", PointCloud2, self.depth_callback)ros 

        self.depth_sub = rospy.Subscriber("/camera/depth/image_rect_raw", Image , self.depth_callback)

    def image_callback(self,image_data):

        try:
            self.cv_image = self.bridge.imgmsg_to_cv2(image_data, "bgr8")
        except CvBridgeError as e:
            print(e)    

        self.hsv = cv2.cvtColor(self.cv_image, cv2.COLOR_BGR2HSV)

        self.red_lower = (169, 125, 125)
        self.red_upper = (179, 255, 255)

        self.mask = cv2.inRange(self.hsv, self.red_lower, self.red_upper)

        self.result = cv2.bitwise_and(self.hsv, self.hsv, mask=self.mask)

        # cv2.imshow("RGB",self.cv_image)
        # cv2.imshow("HSV", self.hsv)
        # cv2.imshow("HSV Masked",self.result)
        cv2.imshow("Mask", self.mask)
        cv2.waitKey(3)

        # print(self.mask)


    def depth_callback(self,depth_data):

        try:
            self.cv_depth = self.bridge.imgmsg_to_cv2(depth_data, desired_encoding="passthrough")
        except CvBridgeError as e:
            print(e)    

        self.depth_array = np.array(self.cv_depth, dtype=np.float32)



        try:
            self.count = (self.mask == 255).sum()

            if self.count > 200:
                self.x_center, self.y_center = np.argwhere(self.mask==255).sum(0)/self.count
            
                self.dist = self.depth_array[self.y_center,self.x_center]/1000

            if self.dist > 0 and 250<self.y_center<450:
                print(self.y_center, self.x_center)
                print(self.dist)
                self.coordinate_transformation(self.dist)
        
        except:
            None

    def coordinate_transformation(self, dist):

        t = geometry_msgs.msg.TransformStamped()
        t.header.frame_id = "camera_rgb_frame"
        t.child_frame_id = "brick"

        t.header.stamp = rospy.Time.now()
        t.transform.translation.x = dist
        t.transform.translation.y = 0.0
        t.transform.translation.z = 0.0
        t.transform.rotation.x = 0.0
        t.transform.rotation.y = 0.0
        t.transform.rotation.z = 0.0
        t.transform.rotation.w = 1
        tfm = tf2_msgs.msg.TFMessage([t])
        self.pub_tf.publish(tfm)

    

def main(args):
    rospy.init_node('BrickFinder', anonymous=True)
    FB = FindBrick()

    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main(sys.argv)


#     r = rospy.Rate(10)