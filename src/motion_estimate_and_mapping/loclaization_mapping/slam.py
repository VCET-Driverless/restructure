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

def slam():
    return None

def mapping(darknet_image_queue, detections_queue, fps_queue):
    while cap.isOpened():
        detections = detections_queue.get()
        ###############################################
        # frame_resized = frame_queue.get()
        # top_view_img_for_draw, top_view_matrix = chcone.inv_map(frame_resized)
        # top_view_frame_queue.put(top_view_img_for_draw)
        ###############################################
        person, mybox = chcone.get_inv_coor(detections)
        top_view_coordinates_queue.put(mybox)
        ###############################################
    cap.release()

"""

repair:
queues

"""