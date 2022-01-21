
# Library imports
import cv2
import numpy as np
import sys
sys.path.append('../system_manager')

# System imports
from constants import Constants

class Transform(Constants):

    def __init__(self, setup):
        
        super().__init__()

    def inv_map(self, frame):
        """
        Transforms given image to top-view image (used for visual debug)
        :frame: front-view image
        :returns: transformation matrix and transformed image
        """
        
        image = cv2.warpPerspective(frame, self.M, self.TOP_VIEW_IMAGE_DIMESNION, flags=cv2.INTER_LINEAR)
        
        return image, self.M

    
    def convertBack(self, x, y, w, h):
        """
        Converts detections output into x-y coordinates
        :x, y: position of bounding box
        :w, h: height and width of bounding box
        """
        
        xmin = int(round(x - (w / 2)))
        xmax = int(round(x + (w / 2)))
        ymin = int(round(y - (h / 2)))
        ymax = int(round(y + (h / 2)))
        return xmin, ymin, xmax, ymax

    def get_inv_coor_different_boundary(self,detections):
        """
        Converts front-view coordinates (of cone) to top-view coordinates
        :detections: front-view coordinates
        :M: transformation matrix
        :returns: top-view coordinates of cones and person
        """
        
        blue = []
        orange = []
       
        for detection in detections:
            x, y, w, h = detection[2][0],\
                detection[2][1],\
                detection[2][2],\
                detection[2][3]
            xmin, ymin, xmax, ymax = self.convertBack(
                float(x), float(y), float(w), float(h))
            pt1 = (xmin, ymin)
            pt2 = (xmax, ymax)
            
            a = np.array([[( (xmax+xmin)//2 ), (ymax//1)]], dtype='float32')
            a = np.array([a])
            pointsOut = cv2.perspectiveTransform(a, self.M)
            box = int(pointsOut[0][0][0]), int(pointsOut[0][0][1])
            
            if(detection[0] == 'blue'):
                blue.append(box)
            else:
                orange.append(box)
        
        blue = sorted(blue, key=lambda k:(k[1], k[0])).copy()
        orange = sorted(orange, key=lambda k:(k[1], k[0])).copy()

        return blue[::-1], orange[::-1]


