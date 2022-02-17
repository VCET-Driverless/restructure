
# Library imports
import cv2
import numpy as np

class Constants:
    
    def __init__(self):
        
        self.CAM_PATH, self.TESTER, self.WHICH_SYSTEM = self.set_dynamic_constants()
        
        (self.BAUD_RATE, self.CAR_BELOW_Y, self.LIMIT_CONE,
         self.MIDPOINT_ONE_BOUNDARY, self.P,
         self.MAX_CONELESS_FRAMES, self.ARDUINO_CONNECTED,
         self.RATE, self.TOP_VIEW_IMAGE_DIMENSION,
         self.FRONT_VIEW_IMAGE_DIMENSION, self.FRONT_VIEW_POINTS) = self.set_independent_constants()
        
        self.TOP_VIEW_POINTS, self.M, self.TOP_VIEW_CAR_COORDINATE, self.MS = self.set_dependent_constants()
        
        self.k, self.Lfc, self.Kp, self.L  = self.set_pure_pursuit_constants()
        
        
    def set_dynamic_constants(self):
        
        CAM_PATH = 6              # "http://192.168.43.156:4747/video"
        TESTER = "Sharvin"
        WHICH_SYSTEM = "Ubuntu"
        
        return CAM_PATH, TESTER, WHICH_SYSTEM
        
    def set_independent_constants(self):
        BAUD_RATE = 115200 # sampeling speed 
        CAR_BELOW_Y = 25 # y coordinate of car below max y coordinate
        LIMIT_CONE = 100 # threshold after which detections won't be considered -> 'y' coordinate
        MIDPOINT_ONE_BOUNDARY = 100 # one side is empty of cones, this is used as offset X coordinate
        P = 2.115
        MAX_CONELESS_FRAMES = 30
        ARDUINO_CONNECTED = False
        RATE = 1
        TOP_VIEW_IMAGE_DIMESNION = (416, 285) # inv map output-image size (w, h) = (x, y)
        FRONT_VIEW_IMAGE_DIMESNION = (416, 416) # (w, h) = (x, y)
        FRONT_VIEW_POINTS = [(0   , 100),# camera 
                            (-600, 416),
                            (416 , 100), 
                            (1016, 416)]
        
        return (
            BAUD_RATE, CAR_BELOW_Y, LIMIT_CONE, MIDPOINT_ONE_BOUNDARY,
            P, MAX_CONELESS_FRAMES, ARDUINO_CONNECTED, RATE, TOP_VIEW_IMAGE_DIMESNION, 
            FRONT_VIEW_IMAGE_DIMESNION, FRONT_VIEW_POINTS
        )
        
    def set_dependent_constants(self):
        
        TOP_VIEW_POINTS = [(0         				   , 0         				    ),
                        (0         				   , self.TOP_VIEW_IMAGE_DIMENSION[1]),
                        (self.TOP_VIEW_IMAGE_DIMENSION[0], 0         				    ), 
                        (self.TOP_VIEW_IMAGE_DIMENSION[0], self.TOP_VIEW_IMAGE_DIMENSION[1])]

        M = cv2.getPerspectiveTransform( np.float32(self.FRONT_VIEW_POINTS), np.float32(TOP_VIEW_POINTS))

        TOP_VIEW_CAR_COORDINATE = (self.TOP_VIEW_IMAGE_DIMENSION[0]//2, self.TOP_VIEW_IMAGE_DIMENSION[1] + self.CAR_BELOW_Y) # car coordinates on image

        MS = 1/self.RATE
        
        return TOP_VIEW_POINTS, M, TOP_VIEW_CAR_COORDINATE, MS
        
    def set_pure_pursuit_constants(self):
        
        # Pure pursuit constants
        k = 0.1  # lookahead distance coefficient
        Lfc = 180.0  # lookahead distance
        Kp = 2.15  # Speed P controller coefficient
        L = 65  # Vehicle wheelbase, unit:pixel
        
        return k, Lfc, Kp, L 