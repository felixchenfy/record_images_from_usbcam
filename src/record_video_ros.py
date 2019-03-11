#!/usr/bin/env python
# -*- coding: utf-8 -*-


# -- Standard
import numpy as np
import sys, os
import cv2
import datetime
from my_lib import *

# -- ROS
import rospy
from sensor_msgs.msg import CameraInfo
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

PYTHON_FILE_PATH=os.path.join(os.path.dirname(__file__))+"/"

# ----------------------------------------------------

class ImageSubscriber(object):
    def __init__(self, topic_name):
        self.bridge = CvBridge()

        self.sub = rospy.Subscriber(topic_name, Image, self._call_back)

        self.cnt = 0  # count images
        self.rosImage=None
        self.t=None
        self.is_image_updated=False
    
    def _call_back(self, rosImage):
        self.rosImage =rosImage
        self.t=rospy.get_time()
        self.cnt+=1
        self.is_image_updated=True

    def _get_image(self):
        self.is_image_updated=False
        return self.rosImage, self.t

    def isReceiveImage(self):
        return self.is_image_updated

# Subscribe color image topic
class ColorImageSubscriber(ImageSubscriber):
    def __init__(self, topic_name):
        super(ColorImageSubscriber, self).__init__(topic_name)
   
    def get_image(self):
        rosImage, t = self._get_image()
        return self.bridge.imgmsg_to_cv2(rosImage, "bgr8"), t


# -- Main
if __name__ == "__main__":
    rospy.init_node("record_usb_cam_video")
    
    # Choose between usb_cam or folder
    source = rospy.get_param("~source")
    if source == "usb_cam":
        sub = ColorImageSubscriber("usb_cam/image_raw")
    else:
        print("Sorry, this mode has been completed. Use usb_cam mode please")
        # sub = ReadImageFromFolder()
        # framerate = 10

    processor = ProcessEvent(folder_name = rospy.get_param("~action_type"))

    # Loop to read images
    while not rospy.is_shutdown():
        if sub.isReceiveImage():
            image, t = sub.get_image()

            # Show image
            cv2.imshow("human_pose_recorder", image)
            
            # Process key event
            key = cv2.waitKey(10) 
            processor.process_event(key, image)
            if(key>=0 and chr(key)=='q'):
                break
            
            # sleep
            if not (source == "usb_cam"):
                rospy.sleep(1.0/framerate)

        rospy.sleep(0.0001)

    # return
    cv2.destroyAllWindows()
    rospy.loginfo("Recorder stops")


