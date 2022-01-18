import cv2
import numpy as np
#import darknet
#Value for M, TOP_VIEW_IMAGE_DIMESNION


class transform:


    def __init__(self, setup):
        # self.cap = cv2.VideoCapture(setup.CAM_PATH)
        self.network, self.class_names, self.class_colors = darknet.load_network(
            setup.args.config_file,
            setup.args.data_file,
            setup.args.weights,
            batch_size=1
        )


    def inv_map(setup, frame):
        """
        Transforms given image to top-view image (used for visual debug)
        :frame: front-view image
        :returns: transformation matrix and transformed image
        """
        image = cv2.warpPerspective(frame, setup.M, setup.TOP_VIEW_IMAGE_DIMESNION, flags=cv2.INTER_LINEAR)
        #cv2.imshow('itshouldlookfine!', image)
        return image, setup.M

    
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
        #print(len(detections))
        for detection in detections:
            x, y, w, h = detection[2][0],\
                detection[2][1],\
                detection[2][2],\
                detection[2][3]
            xmin, ymin, xmax, ymax = self.convertBack(
                float(x), float(y), float(w), float(h))
            pt1 = (xmin, ymin)
            pt2 = (xmax, ymax)
            #print(type(detection[0]))
            #person.append( ( (xmin+xmax)//2,(ymax) ) )
            a = np.array([[( (xmax+xmin)//2 ), (ymax//1)]], dtype='float32')
            a = np.array([a])
            pointsOut = cv2.perspectiveTransform(a, M)
            box = int(pointsOut[0][0][0]), int(pointsOut[0][0][1])
            #print(detection[0])
            if(detection[0] == 'blue'):
                blue.append(box)
            else:
                orange.append(box)
        
        blue = sorted(blue, key=lambda k:(k[1], k[0])).copy()
        orange = sorted(orange, key=lambda k:(k[1], k[0])).copy()
        #print(orange[::-1],'\n')

        return blue[::-1], orange[::-1]


