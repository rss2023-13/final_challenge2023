#!/usr/bin/env python

import numpy as np
import rospy

import cv2
from cv_bridge import CvBridge, CvBridgeError

from sensor_msgs.msg import Image
from geometry_msgs.msg import Point #geometry_msgs not in CMake file
from visual_servoing.msg import ConeLocationPixel

from lookahead_detection import *


class ConeDetector():
    """
    A class for applying your cone detection algorithms to the real robot.
    Subscribes to: /zed/zed_node/rgb/image_rect_color (Image) : the live RGB image from the onboard ZED camera.
    Publishes to: /relative_cone_px (ConeLocationPixel) : the coordinates of the cone in the image frame (units are pixels).
    """
    def __init__(self):
        # Subscribe to ZED camera RGB frames
        self.cone_pub = rospy.Publisher("/relative_cone_px", ConeLocationPixel, queue_size=10)
        self.debug_pub = rospy.Publisher("/cone_debug_img", Image, queue_size=10)
        self.image_sub = rospy.Subscriber("/zed/zed_node/rgb/image_rect_color", Image, self.image_callback)
        self.bridge = CvBridge() # Converts between ROS images and OpenCV Images


    def slice_img(self, img, lower, upper): # replace 
        img[:int(lower*img.shape[0])] = (0,0,0)
        img[int(upper*img.shape[0]):] = (0,0,0)

        return img
    
    def image_callback(self, image_msg):
        """Given an image message, return u, v pixel coordinates for the lookahead point between the white lane lines.

        Args:
            image_msg (_type_): _description_
        """

        #################################
        # YOUR CODE HERE
        # detect the cone and publish its
        # pixel location in the image.
        #################################

        img_orig = self.bridge.imgmsg_to_cv2(image_msg, "bgr8")

        x_lookahead, y_lookahead = lookahead_px_from_img(img_orig)

        cone_loc = ConeLocationPixel()
        cone_loc.u = x_lookahead
        cone_loc.v = y_lookahead

        self.cone_pub.publish(cone_loc)

        debug_msg = self.bridge.cv2_to_imgmsg(img_orig, "bgr8")
        self.debug_pub.publish(debug_msg)


if __name__ == '__main__':
    try:
        rospy.init_node('ConeDetector', anonymous=True)
        ConeDetector()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
