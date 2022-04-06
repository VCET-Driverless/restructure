
# Library imports
import cv2
# import sys
# sys.path.append("/home/tejas/Desktop/darknet")

# System imports
# import darknet
from perception.transform import Transform
from perception.detect import Detect

class Perceive():

    def __init__(self, detect):
        self.width = detect.width
        self.height = detect.height

    def video_capture(self, setup, detect, frame_queue, top_view_frame_queue, top_view_blue_coordinates_queue, top_view_orange_coordinates_queue, p1_child):
        
        print("perception starts")
        transform = Transform()
        # detect = Detect(setup)
        print("Detection instance set")
        cap = cv2.VideoCapture("/home/tejas/Documents/VCET-Driverless/restructure/data/video.mp4")
        while True:
            
            ret, frame = cap.read()
            print(frame)
            if not ret:
                print("cam not detected")
                break
            print("Perception frame read done")
            # Resizing image according to trained model image size
            frame_rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
            frame_resized = cv2.resize(frame_rgb, (self.width, self.height),
                                    interpolation=cv2.INTER_LINEAR)
            print(frame.shape)
            print(frame_rgb.shape)
            print(frame_resized.shape)
            # Obtaining top view image
            top_view_img_for_draw, top_view_matrix = transform.inv_map(frame_resized)
            
            # cv2.imshow("frame", frame_resized)

            # if cv2.waitKey(3000) == 27:
            #     break

            print("Detection starts")
            # Detections, bounding boxes and top view coordinates

            try:
                detections, blue, orange = detect.detect(setup, transform, frame_resized)
            except:
                blue=[]
                orange=[]

            # Adding images to queue
            frame_queue.put(frame_resized)
            top_view_frame_queue.put(top_view_img_for_draw)
            top_view_blue_coordinates_queue.put(blue)
            top_view_orange_coordinates_queue.put(orange)
            
            print("Detections added to queue")
            if not p1_child.empty():
                if p1_child.get() == False:
                    break
            
            
        setup.cap.release()
        cv2.destroyAllWindows()
        print("Perception process has stopped")
