
# -- Standard
import numpy as np
import sys, os
import cv2
import datetime
PYTHON_FILE_PATH=os.path.join(os.path.dirname(__file__))+"/"

# -- ROS
# import rospy
# from sensor_msgs.msg import CameraInfo
# from sensor_msgs.msg import Image
# from cv_bridge import CvBridge, CvBridgeError

def get_time():
    s=str(datetime.datetime.now())[5:].replace(' ','-').replace(":",'-').replace('.','-')[:-3]
    return s # day, hour, seconds: 02-26-15-51-12-556

def int2str(num, blank):
    return ("{:0"+str(blank)+"d}").format(num)


class ProcessEvent(object):
    def __init__(self, folder_name):
        self.flag_record = False
        self.path = PYTHON_FILE_PATH + "../data/"
        self.folder = None
        self.cnt_video = 0
        self.cnt_image = 0 
        self.folder_name = folder_name

    def process_event(self, q, image):
        
        if q>=0 and chr(q)=='s' and self.flag_record == False:
            self.flag_record = True
            self.cnt_video += 1
            self.cnt_image = 0

            # self.folder = self.folder_name + "_"  + int2str(self.cnt_video, blank = 2)
            self.folder = self.folder_name + "_"  + get_time()
            
            if not os.path.exists(self.path + self.folder):
                os.makedirs(self.path + self.folder)

            print("\n\n")
            print("==============================================\n")
            print("Start recording video ...\n")

        if q>=0 and chr(q)=='d' and self.flag_record == True:
            self.flag_record = False
            print("Stop recording video ...\n")
            print("==============================================\n")
            print("\n\n")

        if self.flag_record:
            self.cnt_image += 1 
            blank = 5
            filename = self.path  + "/" +  self.folder + "/" + int2str(self.cnt_image, blank) + ".png"
            cv2.imwrite(filename, image)
            print("record image: " + filename + "\n")

