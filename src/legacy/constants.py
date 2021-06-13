import cv2
import numpy as np

# Dynamic constants
CAM_PATH = 6 # "http://192.168.43.156:4747/video"

TESTER = "sanket"

WHICH_SYSTEM = "sanket"

# Independent constants
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

# Dependent constants
TOP_VIEW_POINTS = [(0         				   , 0         				    ),
          		   (0         				   , TOP_VIEW_IMAGE_DIMESNION[1]),
          		   (TOP_VIEW_IMAGE_DIMESNION[0], 0         				    ), 
          		   (TOP_VIEW_IMAGE_DIMESNION[0], TOP_VIEW_IMAGE_DIMESNION[1])]

M = cv2.getPerspectiveTransform( np.float32(FRONT_VIEW_POINTS), np.float32(TOP_VIEW_POINTS) )

TOP_VIEW_CAR_COORDINATE = (TOP_VIEW_IMAGE_DIMESNION[0]//2, TOP_VIEW_IMAGE_DIMESNION[1] + CAR_BELOW_Y) # car coordinates on image

MS = 1/RATE

# Pure pursuit constants
k = 0.1  # lookahead distance coefficient
Lfc = R = 180.0  # lookahead distance
Kp = 2.15  # Speed P controller coefficient
L = 65  # Vehicle wheelbase, unit:pixel

def log_constants():
	return {
		"log_constants" : {
			"CAM_PATH" : CAM_PATH,  
			"BAUD_RATE" : BAUD_RATE, 
			"CAR_BELOW_Y" : CAR_BELOW_Y,
			"LIMIT_CONE" : LIMIT_CONE,
			"MIDPOINT_ONE_BOUNDARY" : MIDPOINT_ONE_BOUNDARY,
			"P" : P,
			"MAX_CONELESS_FRAMES" : MAX_CONELESS_FRAMES,
			"ARDUINO_CONNECTED" : ARDUINO_CONNECTED,
			"RATE" : RATE, 
			"TESTER" : TESTER,
			"WHICH_SYSTEM" : WHICH_SYSTEM,
			"TOP_VIEW_IMAGE_DIMESNION" : TOP_VIEW_IMAGE_DIMESNION,
			"FRONT_VIEW_POINTS" : FRONT_VIEW_POINTS,
			"Lookahead_Distance" : Lfc
		}
	} 


