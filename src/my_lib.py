
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

def get_time():
    s=str(datetime.datetime.now())[5:].replace(' ','-').replace(":",'-').replace('.','-')[:-3]
    return s # day, hour, seconds: 02-26-15-51-12-556
    

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



# class ReadImageFromFolder(object):
#     def __init__(self, folder):
#         self.folder = folder
#     def isReceiveImage(self):
#         # todo 
#         return True
#     def get_image(self):
#         # todo 
#         return cv2.imread(path)