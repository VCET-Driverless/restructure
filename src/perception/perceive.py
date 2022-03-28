
# Library imports
import cv2
import sys
sys.path.append("../darknet")

# System imports
import darknet
from perception.transform import Transform
from perception.detect import Detect

class Perceive(Detect):

    def init(self):
        super().__init__()

    def video_capture(self, setup, frame_queue, top_view_frame_queue, top_view_blue_coordinates_queue, top_view_orange_coordinates_queue, p1_child):
        
        transform = Transform()
        detect = Detect(setup)
        
        while True:
            
            ret, frame = setup.cap.read()
            if not ret:
                break
            
            # Resizing image according to trained model image size
            frame_rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
            frame_resized = cv2.resize(frame_rgb, (self.width, self.height),
                                    interpolation=cv2.INTER_LINEAR)
            
            # Preparing image for detections
            img_for_detect = darknet.make_image(self.width, self.height, 3)
            darknet.copy_image_from_bytes(img_for_detect, frame_resized.tobytes())
            
            # Obtaining top view image
            top_view_img_for_draw, top_view_matrix = transform.inv_map(frame_resized)
            
            # Detections, bounding boxes and top view coordinates
            detections, blue, orange = detect.detect(setup, img_for_detect)
            
            # Adding images to queue
            frame_queue.put(frame_resized)
            top_view_frame_queue.put(top_view_img_for_draw)
            top_view_blue_coordinates_queue.put(blue)
            top_view_orange_coordinates_queue.put(orange)
            
            if not p1_child.empty():
                if p1_child.get() == False:
                    break
            
            
        setup.cap.release()
        print("Perception process has stopped")
