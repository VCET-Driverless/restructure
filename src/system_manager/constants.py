import cv2
import numpy as np

class Constants:
    
    def __init__(self):
        
        self.CAM_PATH = 6              # "http://192.168.43.156:4747/video"
        self.TESTER = "Tester's name"
        self.WHICH_SYSTEM = "Tester's System"
        self.BAUD_RATE = 115200 # sampeling speed 
        self.CAR_BELOW_Y = 25 # y coordinate of car below max y coordinate
        self.LIMIT_CONE = 100 # threshold after which detections won't be considered -> 'y' coordinate
        self.MIDPOINT_ONE_BOUNDARY = 100 # one side is empty of cones, this is used as offset X coordinate
        self.P = 2.115
        self.MAX_CONELESS_FRAMES = 30
        self.ARDUINO_CONNECTED = False
        self.RATE = 1
        self.TOP_VIEW_IMAGE_DIMESNION = (416, 285) # inv map output-image size (w, h) = (x, y)
        self.FRONT_VIEW_IMAGE_DIMESNION = (416, 416) # (w, h) = (x, y)
        self.FRONT_VIEW_POINTS = [(0   , 100),# camera 
                            (-600, 416),
                            (416 , 100), 
                            (1016, 416)]
        
        self.TOP_VIEW_POINTS = []
        self.MIDPOINT_ONE_BOUNDARYM = cv2.getPerspectiveTransform()
        self.TOP_VIEW_CAR_COORDINATE = () # car coordinates on image
        self.MS = 1/self.RATE

        # Pure pursuit constants
        self.k = 0.1  # lookahead distance coefficient
        self.Lfc = self.R = 180.0  # lookahead distance
        self.Kp = 2.15  # Speed P controller coefficient
        self.L = 65  # Vehicle wheelbase, unit:pixel
        
        
    def set_dynamic_constants(self):
        
        self.CAM_PATH = 6              # "http://192.168.43.156:4747/video"
        self.TESTER = "Sharvin"
        self.WHICH_SYSTEM = "Ubuntu"
        
        
    def set_independent_constants(self):
        self.BAUD_RATE = 115200 # sampeling speed 
        self.CAR_BELOW_Y = 25 # y coordinate of car below max y coordinate
        self.LIMIT_CONE = 100 # threshold after which detections won't be considered -> 'y' coordinate
        self.MIDPOINT_ONE_BOUNDARY = 100 # one side is empty of cones, this is used as offset X coordinate
        self.P = 2.115
        self.MAX_CONELESS_FRAMES = 30
        self.ARDUINO_CONNECTED = False
        self.RATE = 1
        self.TOP_VIEW_IMAGE_DIMESNION = (416, 285) # inv map output-image size (w, h) = (x, y)
        self.FRONT_VIEW_IMAGE_DIMESNION = (416, 416) # (w, h) = (x, y)
        self.FRONT_VIEW_POINTS = [(0   , 100),# camera 
                            (-600, 416),
                            (416 , 100), 
                            (1016, 416)]
        
        
    def set_dependent_constants(self):
        
        self.TOP_VIEW_POINTS = [(0         				   , 0         				    ),
                        (0         				   , self.TOP_VIEW_IMAGE_DIMESNION[1]),
                        (self.TOP_VIEW_IMAGE_DIMESNION[0], 0         				    ), 
                        (self.TOP_VIEW_IMAGE_DIMESNION[0], self.TOP_VIEW_IMAGE_DIMESNION[1])]

        self.M = cv2.getPerspectiveTransform( np.float32(self.FRONT_VIEW_POINTS), np.float32(self.TOP_VIEW_POINTS))

        self.TOP_VIEW_CAR_COORDINATE = (self.TOP_VIEW_IMAGE_DIMESNION[0]//2, self.TOP_VIEW_IMAGE_DIMESNION[1] + self.CAR_BELOW_Y) # car coordinates on image

        self.MS = 1/self.RATE
        
        
    def set_constants(self):
        self.set_dynamic_constants()
        self.set_independent_constants()
        self.set_dependent_constants()