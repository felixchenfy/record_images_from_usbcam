#!/usr/bin/env python
# -*- coding: utf-8 -*-

import numpy as np
import sys, os
import cv2
import datetime
from my_lib import *

import rospy
from sensor_msgs.msg import CameraInfo
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

VIDEO_TOPIC = "usb_cam/image_raw"

# A virtual class for subscribing image from ROS topic
class ImageSubscriber(object):
    def __init__(self, topic_name):
        self.bridge = CvBridge()

        self.img_subscriber = rospy.Subscriber(
            topic_name, Image, self._call_back)

        self.cnt = 0  # count images
        self.rosImage = None
        self.t = None
        self.is_image_updated = False
    
    def isReceiveImage(self):
        return self.is_image_updated
    
    def get_image(self):
        raise RuntimeError("Please overload this `def get_image`.")
    
    def _call_back(self, rosImage):
        self.rosImage = rosImage
        self.t = rospy.get_time()
        self.cnt += 1
        self.is_image_updated = True

    def _get_image(self):
        self.is_image_updated = False
        return self.rosImage, self.t


# Subscribe color image from ROS topic
class ColorImageSubscriber(ImageSubscriber):
    def __init__(self, topic_name):
        super(ColorImageSubscriber, self).__init__(topic_name)
   
    def get_image(self):
        rosImage, t = self._get_image()
        return self.bridge.imgmsg_to_cv2(rosImage, "bgr8"), t


if __name__ == "__main__":
    rospy.init_node("record_usb_cam_video")
    
    img_subscriber = ColorImageSubscriber(VIDEO_TOPIC)
    image_recorder = KeyProcessorAndImageRecorder(
        sub_folder_name=rospy.get_param("~sub_folder_name"),
        dst_folder="data/",
        img_suffix="jpg")

    while not rospy.is_shutdown():
        rospy.sleep(0.005)

        if not img_subscriber.isReceiveImage():
            continue
        
        image, t = img_subscriber.get_image()

        cv2.imshow("human_pose_recorder", image)
        key = cv2.waitKey(5)
        
        image_recorder.check_key_and_save_image(key, image)
        if(key >= 0 and chr(key) == 'q'):
            break

    cv2.destroyAllWindows()
    rospy.loginfo("Recorder stops")


