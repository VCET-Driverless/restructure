
# Library imports
import cv2

# System imports
import darknet
from transform import inv_map

class Perceive:

    def init(self, setup):
        self.cap = cv2.VideoCapture(setup.CAM_PATH)
        self.width = darknet.network_width(self.network)
        self.height = darknet.network_height(self.network)

    def video_capture(self, frame_queue, darknet_image_queue):
        
        while self.cap.isOpened():
            ret, frame = self.cap.read()
            if not ret:
                break
            frame_rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
            #frame_rgb = frame#cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
            frame_resized = cv2.resize(frame_rgb, (self.width, self.height),
                                    interpolation=cv2.INTER_LINEAR)
            frame_queue.put(frame_resized)
            img_for_detect = darknet.make_image(self.width, self.height, 3)
            darknet.copy_image_from_bytes(img_for_detect, frame_resized.tobytes())
            darknet_image_queue.put(img_for_detect)
            ###############################################
            top_view_img_for_draw, top_view_matrix = inv_map(frame_resized)
            self.top_view_frame_queue.put(top_view_img_for_draw)
            ###############################################
        self.cap.release()
