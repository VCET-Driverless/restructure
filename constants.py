import cv2
import numpy as np

# Independent
path = "http://192.168.43.156:4747/video"

TOP_VIEW_IMAGE_DIMESNION = (416, 285) # inv map output-image size (w, h) = (x, y)

BAUD_RATE = 115200 # sampeling speed

FRONT_VIEW_POINTS = [(0   , 100),# camera 
					 (-600, 416),
         			 (416 , 100), 
         			 (1016, 416)]

CAR_BELOW_Y = 25 # y coordinate of car below max y coordinate

LIMIT_CONE = 100 # threshold after which detections won't be considered -> 'y' coordinate

mid_c = 100 # when one side is empty of cones, this variable is used as offset 

P = 2.115

MAX_CONELESS_FRAMES = 30

ARDUINO_CONNECTED = False

RATE = 1

# Dependent
TOP_VIEW_POINTS = [(0         				  , 0         				   ),
          		   (0         				  , TOP_VIEW_IMAGE_DIMESNION[1]),
          		   (TOP_VIEW_IMAGE_DIMESNION[0], 0         				   ), 
          		   (TOP_VIEW_IMAGE_DIMESNION[0], TOP_VIEW_IMAGE_DIMESNION[1])]

M = cv2.getPerspectiveTransform( np.float32(pt_in), np.float32(pt_out) )

TOP_VIEW_CAR_COORDINATE = (TOP_VIEW_IMAGE_DIMESNION[0]//2, TOP_VIEW_IMAGE_DIMESNION[1] + CAR_BELOW_Y) # car coordinates on image

MS = 1/RATE