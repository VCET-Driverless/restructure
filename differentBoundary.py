from ctypes import *
import random
import os
import cv2
import time
import darknet
import argparse
from threading import Thread, enumerate
from queue import Queue
import chcone
import math
import serial
from constants import ARDUINO_CONNECTED, BAUD_RATE, TOP_VIEW_IMAGE_DIMESNION, MAX_CONELESS_FRAMES, MS, P, CAM_PATH, log_constants
import dataLog
import datetime
from json import dump

prev_time_my = time.time()

BOUNDARY_INVERT = None

if(ARDUINO_CONNECTED):
    try:
        s=serial.Serial('/dev/ttyACM0',BAUD_RATE)
        print("Connecting to : /dev/ttyACM0")
    except:
        try:
            print("failed...")
            s=serial.Serial('/dev/ttyACM1',BAUD_RATE)
            print("Connecting to : /dev/ttyACM1")
        except:
            print("failed... give port premission")

def parser():
    parser = argparse.ArgumentParser(description="YOLO Object Detection")
    parser.add_argument("--input", type=str, default=CAM_PATH,
                        help="video source. If empty, uses webcam 0 stream")
    parser.add_argument("--out_filename", type=str, default="",
                        help="inference video name. Not saved if empty")
    parser.add_argument("--weights", default="yolov4-newtiny3l.weights",
                        help="yolo weights path")
    parser.add_argument("--dont_show", action='store_true',
                        help="windown inference display. For headless systems")
    parser.add_argument("--ext_output", action='store_true',
                        help="display bbox coordinates of detected objects")
    parser.add_argument("--config_file", default="./cfg/yolov4-newtiny3l.cfg",
                        help="path to config file")
    parser.add_argument("--data_file", default="./cfg/coco.data",
                        help="path to data file")
    parser.add_argument("--thresh", type=float, default=.25,
                        help="remove detections with confidence below this value")
    parser.add_argument("--boundary", type=int, default=1,
                        help="0->boundary has different color \n 1->boundary has mix colors")
    return parser.parse_args()


def str2int(video_path):
    """
    argparse returns and string althout webcam uses int (0, 1 ...)
    Cast to int if needed
    """
    try:
        return int(video_path)
    except ValueError:
        return video_path


def check_arguments_errors(args):
    assert 0 < args.thresh < 1, "Threshold should be a float between zero and one (non-inclusive)"
    if not os.path.exists(args.config_file):
        raise(ValueError("Invalid config path {}".format(os.path.abspath(args.config_file))))
    if not os.path.exists(args.weights):
        raise(ValueError("Invalid weight path {}".format(os.path.abspath(args.weights))))
    if not os.path.exists(args.data_file):
        raise(ValueError("Invalid data file path {}".format(os.path.abspath(args.data_file))))
    if str2int(args.input) == str and not os.path.exists(args.input):
        raise(ValueError("Invalid video path {}".format(os.path.abspath(args.input))))


def set_saved_video(input_video, output_video, size):
    fourcc = cv2.VideoWriter_fourcc(*"MJPG")
    fps = int(input_video.get(cv2.CAP_PROP_FPS))
    video = cv2.VideoWriter(output_video, fourcc, fps, size)
    return video


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
        ###############################################
        top_view_img_for_draw, top_view_matrix = chcone.inv_map(frame_resized)
        top_view_frame_queue.put(top_view_img_for_draw)
        ###############################################
    cap.release()


# Get frame and put detections in queue
def inference(darknet_image_queue, detections_queue, fps_queue):
    while cap.isOpened():
        darknet_image = darknet_image_queue.get()
        prev_time = time.time()
        detections = darknet.detect_image(network, class_names, darknet_image, thresh=args.thresh)
        detections_queue.put(detections)
        ###############################################
        blue, orange = chcone.get_inv_coor_different_boundary(detections)
        top_view_blue_coordinates_queue.put(blue)
        top_view_orange_coordinates_queue.put(orange)
        ###############################################
        fps = int(1/(time.time() - prev_time))
        fps_queue.put(fps)
        #print("FPS: {}".format(fps))
        #darknet.print_detections(detections, args.ext_output)
        darknet.free_image(darknet_image)
    cap.release()


# draw bound box in image
def drawing(frame_queue, detections_queue, fps_queue):

    f, log_file_name = dataLog.give_file()
    DATA = [log_constants()]
    DATA[0]["log_constants"]["CAM_PATH"] = args.input
    log_data = []

    random.seed(3)  # deterministic bbox colors
    video = set_saved_video(cap, log_file_name+".mp4", (width, height))
    global prev_time_my

    counter = 0
    steering = '4'
    limit_frames = 5
    angle_limit = [0]*limit_frames
    frame_count = 0
    serial_writen_now = False
    serial_data = None
    if(ARDUINO_CONNECTED):
        s.write(str('a').encode())

    try:
        while cap.isOpened():
            frame_resized = frame_queue.get()
            detections = detections_queue.get()
            fps = fps_queue.get()
            if frame_resized is not None:
                image = darknet.draw_boxes(detections, frame_resized, class_colors)
                ###############################################
                top_image = top_view_frame_queue.get()
                blue = top_view_blue_coordinates_queue.get()
                orange = top_view_orange_coordinates_queue.get()

                if args.boundary == 0:
                    left_box, right_box, lines = chcone.pathplan_different_boundary(blue, 
                                                                                    orange, 
                                                                                    BOUNDARY_INVERT)
                else:
                    mybox = blue + orange
                    left_box, right_box, lines = chcone.pathplan(mybox, steering)
                top_image = chcone.pathbana(left_box, right_box, lines, top_image)

                # stop the car if no cones found for *MAX_CONELESS_FRAMES* frames
                if len(blue)+len(orange) == 0:
                    counter = counter + 1
                    if counter == MAX_CONELESS_FRAMES:
                        if(ARDUINO_CONNECTED):
                            serial_data = 'c'
                            serial_writen_now = True
                            s.write(str(serial_data).encode())
                        counter = 0

                # encode signal for steering control
                angle = chcone.angle(lines[0], lines[1])
                angle = math.floor(angle)

                # Takes average turning/steering angle of *limit_frames* frames
                angle_limit.append(angle)
                angle_limit.pop(0)
                angle_a = chcone.steer( (sum(angle_limit))//limit_frames )
                st_ang = P*(sum(angle_limit))//limit_frames
                print( st_ang )

                # send 'RATE' number of signels per second
                if(time.time() - prev_time_my >= MS):
                    prev_time_my = time.time()
                    if(ARDUINO_CONNECTED):
                        serial_data = st_ang
                        serial_writen_now = True
                        s.write(str(serial_data).encode())

                # Prevents Arduino buffer overlfow,   
                if(steering != angle_a):
                    if(ARDUINO_CONNECTED):
                        '''s.write(str.encode(angle_a))'''
                        pass
                    steering = angle_a
                    print( 'updated' , angle_a)

                # data logger
                frame_data = {
                        "time_stamp":datetime.datetime.now().astimezone().isoformat(),
                        "frame_count":frame_count,
                        "steering": int(st_ang),
                        "serial_writen_now":serial_writen_now,
                        "serial_data":serial_data,
                        "detections": chcone.get_boxes(detections),
                        "left_box":left_box,
                        "right_box":right_box,
                        "lines":lines
                    }
                log_data.append(frame_data)
                frame_count += 1
                serial_writen_now = False
                serial_data = None

                ###############################################
                image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
                top_image = cv2.cvtColor(top_image, cv2.COLOR_BGR2RGB)
                
                ##############################
                #if args.out_filename is not None:
                video.write(image)
                ##############################

                if not args.dont_show:
                    cv2. namedWindow("Inference") 
                    cv2. moveWindow("Inference", 1000,30)
                    image = cv2.resize(image, (2*TOP_VIEW_IMAGE_DIMESNION[0],
                                               2*TOP_VIEW_IMAGE_DIMESNION[0]))
                    cv2.imshow('Inference', image)
                    top_image = cv2.resize(top_image, (2*TOP_VIEW_IMAGE_DIMESNION[0],
                                                       2*TOP_VIEW_IMAGE_DIMESNION[1]))
                    cv2.imshow('top_view', top_image)
                if cv2.waitKey(fps) == 27:
                    if ARDUINO_CONNECTED:
                        s.write(str('c').encode())
                    break
    finally:  
        if ARDUINO_CONNECTED:
            s.write(str('c').encode())
        cap.release()
        video.release()
        cv2.destroyAllWindows()
        DATA.append( {
                "log_data" : log_data
            } )
        dump(DATA, f, indent=4)
        f.close()


if __name__ == '__main__':
    frame_queue = Queue()
    darknet_image_queue = Queue(maxsize=1)
    detections_queue = Queue(maxsize=1)
    fps_queue = Queue(maxsize=1)

    top_view_frame_queue = Queue()
    top_view_blue_coordinates_queue = Queue(maxsize=1)
    top_view_orange_coordinates_queue = Queue(maxsize=1)

    args = parser()
    if args.boundary == 0:
        BOUNDARY_INVERT = input("enter bl or br: ") == "bl"
    check_arguments_errors(args)
    network, class_names, class_colors = darknet.load_network(
            args.config_file,
            args.data_file,
            args.weights,
            batch_size=1
        )
    width = darknet.network_width(network)
    height = darknet.network_height(network)
    input_path = str2int(args.input)
    cap = cv2.VideoCapture(input_path)
    try:
        Thread(target=video_capture, args=(frame_queue, darknet_image_queue)).start()
        Thread(target=inference, args=(darknet_image_queue, detections_queue, fps_queue)).start()
        Thread(target=drawing, args=(frame_queue, detections_queue, fps_queue)).start()
    except:
        cap.release()
        video.release()
        cv2.destroyAllWindows()
