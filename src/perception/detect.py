
# Library imports
import time
import cv2
import sys
sys.path.append('../../darknet')

# System imports
import darknet
from transform import Transform

class Detect:

    def init(self, setup):
        
        self.network, self.class_names, self.class_colors = darknet.load_network(
            setup.args.config_file,
            setup.args.data_file,
            setup.args.weights,
            batch_size=1
        )


    def detect(self, darknet_image_queue, detections_queue, fps_queue, top_view_blue_coordinates_queue, top_view_orange_coordinates_queue, p1_parent, p2_child):
        
        transform = Transform()
        
        while True:
            
            # Detecting objects from image
            darknet_image = darknet_image_queue.get()
            prev_time = time.time()
            detections = darknet.detect_image(self.network, self.class_names, darknet_image, thresh=setup.args.thresh)
            detections_queue.put(detections)
            
            # Get top view coordinates of each detected object 
            blue, orange = transform.get_inv_coor_different_boundary(detections)
            top_view_blue_coordinates_queue.put(blue)
            top_view_orange_coordinates_queue.put(orange)
            
            # Calculating fps - speed of object detection and top view conversion
            fps = int(1/(time.time() - prev_time))
            fps_queue.put(fps)
            #print("FPS: {}".format(fps))
            #darknet.print_detections(detections, args.ext_output)
            darknet.free_image(darknet_image)
            
            if p2_child.recv() == False:
                p1_parent.send(False)
                break
            else:
                p1_parent.send(True)
                
        
        print("Detection process released")