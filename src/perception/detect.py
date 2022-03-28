
# Library imports
import time
import cv2
import sys
sys.path.append('../darknet')

# System imports
import darknet
from perception.transform import Transform

class Detect:

    def init(self, setup):
        
        self.network, self.class_names, self.class_colors = darknet.load_network(
            setup.args.config_file,
            setup.args.data_file,
            setup.args.weights,
            batch_size=1
        )
        
        self.width = darknet.network_width(self.network)
        self.height = darknet.network_height(self.network)


    def detect(self, setup, darknet_image):
        
        transform = Transform()
        
        # Detecting objects from image
        prev_time = time.time()
        detections = darknet.detect_image(self.network, self.class_names, darknet_image, thresh=setup.args.thresh)
        
        # Get top view coordinates of each detected object 
        blue, orange = transform.get_inv_coor_different_boundary(detections)
        
        # Calculating fps - speed of object detection and top view conversion
        fps = int(1/(time.time() - prev_time))
        #print("FPS: {}".format(fps))
        #darknet.print_detections(detections, args.ext_output)
        darknet.free_image(darknet_image)
                    
        print("Detection process has stopped")
        
        return detections, blue, orange