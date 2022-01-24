
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
        
        self.width = darknet.network_width(self.network)
        self.height = darknet.network_height(self.network)


    def detect(self, setup, darknet_image_queue, detections_queue, top_view_blue_coordinates_queue, top_view_orange_coordinates_queue, p1_child, p2_parent):
        
        queue_is_empty = False
        transform = Transform()
        
        while True:
            
            if queue_is_empty == False:
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
                #print("FPS: {}".format(fps))
                #darknet.print_detections(detections, args.ext_output)
                darknet.free_image(darknet_image)
                
                if p1_child.recv() == False:
                    p2_parent.send(False)
                    break
                else:
                    p2_parent.send(True)
                    
                # Check if queue is empty
                if darknet_image_queue.empty():
                    queue_is_empty = True
            
            else:
                
                if not darknet_image_queue.empty():
                    queue_is_empty = False
                    
                print("Debug detections: darknet_image_queue is empty") 
        
        print("Detection process has stopped")