# from ctypes import *
# import random
# import os
# import cv2
# import time
# import darknet
# import argparse
# from threading import Thread, enumerate
# from queue import Queue
import legacy.chcone
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

def boundary_separation(darknet_image_queue, detections_queue, fps_queue):
    detections = detections_queue.get()
    ###############################################
    mybox = top_view_coordinates_queue.get()
    left_box = list()
    right_box = list()
    """
    further lines sparates boundary
    """
    return left_box, right_box
    ###############################################

def path_plan(darknet_image_queue, detections_queue, fps_queue):
    while cap.isOpened():
        detections = detections_queue.get()
        ###############################################
        left_box, right_box = boundary_separation()
        lines = list()
        ###############################################
    cap.release()
