# from ctypes import *
# import random
# import os
# import cv2
# import time
# import darknet
# import argparse
# from threading import Thread, enumerate
# from queue import Queue
import chcone
# import math
# import serial
# from constants import (
#     ARDUINO_CONNECTED,
#     BAUD_RATE,
#     TOP_VIEW_IMAGE_DIMESNION,
#     MAX_CONELESS_FRAMES,
#     MS,
#     P,
#     CAM_PATH,
#     log_constants
# )
# import dataLog
# import datetime
# from json import dump

# Capture image and put in queue
def video_capture(frame_queue, darknet_image_queue):
    while cap.isOpened():
        ret, frame = cap.read()
        if not ret:
            break
        frame_rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        #frame_rgb = frame#cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        frame_resized = cv2.resize(frame_rgb, (width, height),
                                   interpolation=cv2.INTER_LINEAR)
        frame_queue.put(frame_resized)
        img_for_detect = darknet.make_image(width, height, 3)
        darknet.copy_image_from_bytes(img_for_detect, frame_resized.tobytes())
        darknet_image_queue.put(img_for_detect)
    cap.release()

# Get frame and put detections in queue
def inference(darknet_image_queue, detections_queue, fps_queue):
    while cap.isOpened():
        darknet_image = darknet_image_queue.get()
        prev_time = time.time()
        detections = darknet.detect_image(network, class_names, darknet_image, thresh=args.thresh)
        detections_queue.put(detections)
        ###############################################

        ###############################################
        fps = int(1/(time.time() - prev_time))
        fps_queue.put(fps)
        #print("FPS: {}".format(fps))
        #darknet.print_detections(detections, args.ext_output)
        darknet.free_image(darknet_image)
    cap.release()


"""

input : frame, model param
output : detections

queus : 
frame_queue = raw image put 
darknet_image_queue = darknet formatted image

detections_queue = detections put
fps_queue = fps put


"""
