
# Library imports
import time
import cv2
import sys
sys.path.append("/home/tejas/Documents/VCET-Driverless/darknet")

# System imports
import darknet
from perception.transform import Transform

class Detect:

    def __init__(self, setup):
        print("Darknet setup starts")
        self.network, self.class_names, self.class_colors = darknet.load_network(
            setup.args.config_file,
            setup.args.data_file,
            setup.args.weights,
            batch_size=1
        )
        print("Darknet setup done")
        self.width = darknet.network_width(self.network)
        self.height = darknet.network_height(self.network)
        print("Width: {}; Height: {}".format(self.width, self.height))

    def detect(self, setup, transform, darknet_image):
        
        # transform = Transform()
        
        # Detecting objects from image
        prev_time = time.time()
        print("Detect: detections starts")
        detections = darknet.detect_image(self.network, self.class_names, darknet_image, thresh=setup.args.thresh)
        print("Detect: detections done")
        # Get top view coordinates of each detected object 
        blue, orange = transform.get_inv_coor_different_boundary(detections)
        
        # Calculating fps - speed of object detection and top view conversion
        fps = int(1/(time.time() - prev_time))
        #print("FPS: {}".format(fps))
        #darknet.print_detections(detections, args.ext_output)
        darknet.free_image(darknet_image)
                    
        print("Detection process has stopped")
        
        return detections, blue, orange