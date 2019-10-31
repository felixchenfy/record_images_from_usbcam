#!/usr/bin/env python
# -*- coding: utf-8 -*-

import numpy as np
import sys, os
import cv2
import datetime
import multiprocessing, queue, threading, time
import argparse

from my_lib import *


def parse_args():
    parser = argparse.ArgumentParser(
        description="Webcam image recorder: \nPress s to record images; Press d to stop recording.")
    parser.add_argument("-i", "--webcam_index", required=False,
                        default=0)
    parser.add_argument("-r", "--frame_rate", required=False,
                        default=30.0)
    parser.add_argument("-s", "--sub_folder_name", required=False,
                        default="none")
    args = parser.parse_args()

    idx = args.webcam_index
    if isinstance(idx, str) and idx.isdigit():
        args.webcam_index = int(idx)
        
    return args
    
args = parse_args()

WEBCAM_INDEX = int(args.webcam_index)
FRAME_RATE = float(args.frame_rate)
SUB_FOLDER_NAME = args.sub_folder_name

class ReadFromWebcam(object):
    def __init__(self, max_framerate=30.0, webcam_idx=0):
        ''' Read images from web camera.
        Argument:
            max_framerate {float}: the real framerate will be reduced below this value.
            webcam_idx {int}: index of the web camera on your laptop. It should be 0 by default.
        '''
        # Settings
        self._max_framerate = max_framerate
        queue_size = 3

        # Initialize video reader
        self._video = cv2.VideoCapture(webcam_idx)
        self._is_stoped = False

        # Use a thread to keep on reading images from web camera
        self._imgs_queue = queue.Queue(maxsize=queue_size)
        self._is_thread_alive = multiprocessing.Value('i', 1)
        self._thread = threading.Thread(
            target=self._thread_reading_webcam_images)
        self._thread.start()

        # Manually control the framerate of the webcam by sleeping
        self._min_dt = 1.0 / self._max_framerate
        self._prev_t = time.time() - 1.0 / max_framerate

    def read_image(self):
        dt = time.time() - self._prev_t
        if dt <= self._min_dt:
            time.sleep(self._min_dt - dt)
        self._prev_t = time.time()
        image = self._imgs_queue.get(timeout=10.0)
        return image

    def has_image(self):
        return True  # The web camera always has new image

    def stop(self):
        self._is_thread_alive.value = False
        self._video.release()
        self._is_stoped = True

    def __del__(self):
        if not self._is_stoped:
            self.stop()

    def _thread_reading_webcam_images(self):
        while self._is_thread_alive.value:
            ret, image = self._video.read()
            if self._imgs_queue.full():  # if queue is full, pop one
                img_to_discard = self._imgs_queue.get(timeout=0.001)
            self._imgs_queue.put(image, timeout=0.001)  # push to queue
        print("Web camera thread is dead.")

if __name__=="__main__":

    image_recorder = KeyProcessorAndImageRecorder(
        sub_folder_name=SUB_FOLDER_NAME,
        dst_folder="data/",
        img_suffix="jpg")

    webcam_reader = ReadFromWebcam(max_framerate=FRAME_RATE, webcam_idx=WEBCAM_INDEX)

    i=0
    while webcam_reader.has_image():
        
        image = webcam_reader.read_image()

        cv2.imshow("human_pose_recorder", image)
        key = cv2.waitKey(10) 

        image_recorder.check_key_and_save_image(key, image)

        if(key>=0 and chr(key)=='q'):
            break

    cv2.destroyAllWindows()

