import numpy as np
import sys, os
import cv2
import datetime
ROOT=os.path.join(os.path.dirname(__file__))+"/../"

def get_time():
    s=str(datetime.datetime.now())[5:].replace(' ','-').replace(":",'-').replace('.','-')[:-3]
    return s # day, hour, seconds: 02-26-15-51-12-556

def int2str(num, blank):
    return ("{:0"+str(blank)+"d}").format(num)


class KeyProcessorAndImageRecorder(object):
    def __init__(self, sub_folder_name, 
                 img_suffix="jpg", dst_folder="data"):
        self.is_recording = False
        self.path = ROOT + dst_folder + "/"
        self.folder = None
        self.cnt_video = 0
        self.cnt_image = 0 
        self.img_suffix = img_suffix
        self.sub_folder_name = sub_folder_name

    def check_key_and_save_image(self, q, image):
        
        if q>=0 and chr(q)=='s' and self.is_recording == False:
            self.is_recording = True
            self.cnt_video += 1
            self.cnt_image = 0
            self.folder = self.sub_folder_name + "_"  + get_time()
            
            if not os.path.exists(self.path + self.folder):
                os.makedirs(self.path + self.folder)

            print("\n\n")
            print("==============================================\n")
            print("Start recording video ...\n")

        if q>=0 and chr(q)=='d' and self.is_recording == True:
            self.is_recording = False
            print("Stop recording video ...\n")
            print("==============================================\n")
            print("\n\n")

        if self.is_recording:
            self.cnt_image += 1 
            blank = 5
            filename = self.path  + "/" +  self.folder + "/" + int2str(self.cnt_image, blank) + "." + self.img_suffix
            cv2.imwrite(filename, image)
            print("record image: " + filename + "\n")

