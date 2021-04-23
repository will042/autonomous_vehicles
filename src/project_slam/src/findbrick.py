#!/usr/bin/python
# -*- coding: utf-8 -*-
# from __future__ import print_function

from math import *
import numpy as np
import sys
import cv2
import rospy
from cv_bridge import CvBridge, CvBridgeError

# np.set_printoptions(threshold=sys.maxsize)

# import tf
# import tf2_ros

# import sensor_msgs.point_cloud2 as pc2
# from tf.transformations import quaternion_from_euler

from visualization_msgs.msg import MarkerArray, Marker
from std_msgs.msg import String
from sensor_msgs.msg import Image, PointCloud2, LaserScan
from geometry_msgs.msg import TransformStamped, Point
from tf2_msgs.msg import TFMessage

class FindBrick(object):
    '''
    Class to hold the brick finder
    '''

    def __init__(self):

        self.pub_tf = rospy.Publisher("/tf", TFMessage, queue_size=1)

        self.pub_marker = rospy.Publisher("/brickmarker", Marker, queue_size=10)

        self.bridge = CvBridge()

        self.mark_count = 0

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

            if self.count > 300:
                self.x_center, self.y_center = np.argwhere(self.mask==255).sum(0)/self.count
            
                self.dist = self.depth_array[self.y_center,self.x_center]/1000

            else:
                self.dist = 0

            if self.dist > 0 and 250<self.y_center<450:
                # print(self.y_center, self.x_center)
                # print(self.dist)
                self.show_marker(self.dist)
        
        except:
            None

    # def coordinate_transformation(self, dist):

    #     t = TransformStamped()
    #     t.header.frame_id = "camera_rgb_frame"
    #     t.child_frame_id = "brick"

    #     t.header.stamp = rospy.Time.now()
    #     t.transform.translation.x = dist
    #     t.transform.translation.y = 0.0
    #     t.transform.translation.z = 0.0
    #     t.transform.rotation.x = 0.0
    #     t.transform.rotation.y = 0.0
    #     t.transform.rotation.z = 0.0
    #     t.transform.rotation.w = 1
    #     tfm = TFMessage([t])
    #     self.pub_tf.publish(tfm)

    #     self.show_marker(dist)

    #     # rospy.sleep(.5)

    def show_marker(self, dist):
        '''
        Records all the mappings of the brick every made when to brick is detected.
        Places a marker at all of the mappings.
        '''
        marker = Marker()
        marker = Marker(type=Marker.POINTS,ns='brick_detection', action=Marker.ADD)
        marker.header.frame_id = "camera_rgb_frame"
        marker.header.stamp = rospy.Time.now()
        points = []
        point = Point()
        point.x = dist
        point.y = 0
        point.z = 0 
        points.append(point)

        marker.points = points
        marker.scale.x = 0.1
        marker.scale.y = 0.1
        marker.scale.z = 0.1
        marker.color.a = 2.0
        marker.color.r = 1.0
        marker.action = marker.ADD
        marker.pose.orientation.w = 1
        marker.lifetime = rospy.Duration(0)
        marker.frame_locked = False
        self.pub_marker.publish(marker)      




    # def show_marker(self):
        
    #     marker = Marker()
    #     marker.header.frame_id = "brick"
    #     marker.header.stamp = rospy.Time.now()
    #     marker.type = marker.CUBE
    #     marker.ns = 'brick_marker'
    #     marker.id = self.mark_count

    #     marker.pose.position.x = 0.0
    #     marker.pose.position.y = 0.0
    #     marker.pose.position.z = 0.0
    #     marker.pose.orientation.x = 0.0
    #     marker.pose.orientation.y = 0.0
    #     marker.pose.orientation.z = 0.0
    #     marker.pose.orientation.w = 1.0

    #     marker.lifetime = 9999999999
    #     marker.scale.x = 0.5
    #     marker.scale.y = 0.5
    #     marker.scale.z = 0.5
    #     marker.color.a = 0.9
    #     marker.color.r = 0.9
    #     marker.color.g = 0.1
    #     marker.color.b = 0.1
    #     marker.frame_locked = False

    #     print(marker)

    #     self.pub_marker.publish(marker)

    #     self.mark_count += 1
        
def main(args):
    rospy.init_node('BrickFinder', anonymous=True)
    FB = FindBrick()

    try:
        rospy.Rate(0.5)
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main(sys.argv)


#     r = rospy.Rate(10)