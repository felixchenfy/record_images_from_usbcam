#!/usr/bin/env python
# -*- coding: utf-8 -*-


# -- Standard
import numpy as np
import sys, os
import cv2
import datetime
PYTHON_FILE_PATH=os.path.join(os.path.dirname(__file__))+"/"

# -- ROS
import rospy
from sensor_msgs.msg import CameraInfo
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

from my_lib import *

def int2str(num, blank):
    return ("{:0"+str(blank)+"d}").format(num)

class ProcessEvent(object):
    def __init__(self):
        self.flag_record = False
        self.path = PYTHON_FILE_PATH + "../data/"
        self.folder = None
        self.cnt_video = 0
        self.cnt_image = 0
        self.action_type = rospy.get_param("~action_type")

    def process_event(self, q, image):
        
        if q=='s' and self.flag_record == False:
            self.flag_record = True
            self.cnt_video += 1
            self.cnt_image = 0

            # self.folder = self.action_type + "_"  + int2str(self.cnt_video, blank = 2)
            self.folder = self.action_type + "_"  + get_time()
            
            if not os.path.exists(self.path + self.folder):
                os.makedirs(self.path + self.folder)

            print "\n\n"
            print "==============================================\n"
            print "Start recording video ...\n"

        if q=='d' and self.flag_record == True:
            self.flag_record = False
            print "Stop recording video ...\n"
            print "==============================================\n"
            print "\n\n"

        if self.flag_record:
            self.cnt_image += 1 
            blank = 5
            filename = self.path  + "/" +  self.folder + "/" + int2str(self.cnt_image, blank) + ".png"
            cv2.imwrite(filename, image)
            print "record image: " + filename + "\n"


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

    processor = ProcessEvent()

    # Loop to read images
    while not rospy.is_shutdown():
        if sub.isReceiveImage():
            image, t = sub.get_image()

            # Show image
            cv2.imshow("human_pose_recorder", image)
            
            # Process key event
            key = chr(cv2.waitKey(10))
            processor.process_event(key, image)
            if(key=='q'):
                break
            
            # sleep
            if not (source == "usb_cam"):
                rospy.sleep(1.0/framerate)

        rospy.sleep(0.0001)

    # return
    cv2.destroyAllWindows()
    rospy.loginfo("Recorder stops")


