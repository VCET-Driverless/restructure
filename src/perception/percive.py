
# Library imports
import cv2
import sys
sys.path.append("../../darknet")

# System imports
import darknet
from transform import Transform

class Perceive:

    def init(self, setup, detect):
        self.cap = cv2.VideoCapture(setup.cam_path)
        self.width = darknet.network_width(detect.network)
        self.height = darknet.network_height(detect.network)

    def video_capture(self, frame_queue, darknet_image_queue, top_view_frame_queue, p1_child):
        
        transform = Transform()
        
        while True:
            
            ret, frame = self.cap.read()
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
            
            # Adding images to queue
            frame_queue.put(frame_resized)
            darknet_image_queue.put(img_for_detect)
            top_view_frame_queue.put(top_view_img_for_draw)
            
            if p1_child.recv() == False:
                break
            
            
        self.cap.release()
        print("Video capture released")
