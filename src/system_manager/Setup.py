import argparse
import serial
import cv2
import os
from constants import BAUD_RATE, CAM_PATH,ARDUINO_CONNECTED
import datetime

class Setup:

    def __init__(self):
        self.serial=serial.Serial()
        self.video=cv2.VideoWriter()
        self.parser = argparse.ArgumentParser(description="Setup Parser")
        self.args=argparse.Parser()
        self.cam_path=CAM_PATH
        self.log_file_name=""
        self.file= open()
        self.BOUNDARY_INVERT = None
        self.baud_rate=BAUD_RATE
        self.arduino_connected=ARDUINO_CONNECTED
        

    def Parser(self):
        
        self.parser.add_argument("--input", type=str, default=self.cam_path,
                            help="video source. If empty, uses webcam 0 stream")
        self.parser.add_argument("--out_filename", type=str, default="",
                            help="inference video name. Not saved if empty")
        self.parser.add_argument("--weights", default="yolov4-newtiny3l.weights",
                            help="yolo weights path")
        self.parser.add_argument("--dont_show", action='store_true',
                            help="windown inference display. For headless systems")
        self.parser.add_argument("--ext_output", action='store_true',
                            help="display bbox coordinates of detected objects")
        self.parser.add_argument("--config_file", default="./cfg/yolov4-newtiny3l.cfg",
                            help="path to config file")
        self.parser.add_argument("--data_file", default="./cfg/coco.data",
                            help="path to data file")
        self.parser.add_argument("--thresh", type=float, default=.25,
                            help="remove detections with confidence below this value")
        self.parser.add_argument("--boundary", type=int, default=1,
                            help="0->boundary has different color \n 1->boundary has mix colors")
        self.parser.add_argument("--controller", type=int, default=0,
                            help="0->old controller \n 1->pure pursuit controller")
        
        self.args = self.parser.parse_args()


    def connectAurdino(self):
        
        if(self.arduino_connected):
            try:
                self.serial=serial.Serial('/dev/ttyACM0',self.baud_rate)
                print("Connecting to : /dev/ttyACM0")
            except:
                try:
                    print("failed...")
                    self.serial=serial.Serial('/dev/ttyACM1',self.baud_rate)
                    print("Connecting to : /dev/ttyACM1")
                except:
                    print("failed... give port premission")   


    def set_saved_video(self,input_video, output_video, size):
        fourcc = cv2.VideoWriter_fourcc(*"MJPG")
        fps = int(input_video.get(cv2.CAP_PROP_FPS))
        self.video = cv2.VideoWriter(output_video, fourcc, fps, size)
        return self.video


    def check_arguments_errors(self):
        # self.args=args
        assert 0 < self.args.thresh < 1, "Threshold should be a float between zero and one (non-inclusive)"
        if not os.path.exists(self.args.config_file):
            raise(ValueError("Invalid config path {}".format(os.path.abspath(self.args.config_file))))
        if not os.path.exists(self.args.weights):
            raise(ValueError("Invalid weight path {}".format(os.path.abspath(self.args.weights))))
        if not os.path.exists(self.args.data_file):
            raise(ValueError("Invalid data file path {}".format(os.path.abspath(self.args.data_file))))
        if str2int(self.args.input) == str and not os.path.exists(self.args.input):
            raise(ValueError("Invalid video path {}".format(os.path.abspath(self.args.input))))
            
     def str2int(self, video_path):
        """
        argparse returns and string althout webcam uses int (0, 1 ...)
        Cast to int if needed
        """
        try:
            return int(video_path)
        except ValueError:
            return video_path       


    def  set_cam_input(self, path):
        self.cam_path=path            # "http://192.168.43.156:4747/video"
        
     def give_file(self):
        
        log_folder="logs"
        test_count = 0
        today = datetime.datetime.now()

        if not os.path.isdir(log_folder):
            os.mkdir(log_folder)

        day_folder = log_folder + "/" + str(today.day) + "-" + str(today.month) + "-" + str(today.year)

        if not os.path.isdir(day_folder):
            os.mkdir(day_folder)

        while os.path.isfile(day_folder + "/" +"test" + str(test_count) + ".json"):
            test_count = test_count + 1

        self.log_file_name = day_folder + "/" + "test" + str(test_count)	
        
        self.file = open(self.log_file_name + ".json", "w+")
    
     def setup_driver(self,input_video, output_video, size,args): 
        Setup.Parser(self)
        Setup.connect_aurdino(self)
        Setup.check_arguments_errors(self)
        Setup.set_cam_input(self, self.args.input)                                  # Take input from parser... default set to 6.
        Setup.give_file(self)
        Setup.set_saved_video(self,input_video, self.log_file_name + ".mp4", size)  # Need to figure out how to pass input path.
        