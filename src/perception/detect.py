import time
import cv2
#import darknet

class detect:

    def init(self, setup):
        self.cap = cv2.VideoCapture(setup.CAM_PATH)


def inference(self, setup, darknet_image_queue, detections_queue, fps_queue, top_view_blue_coordinates_queue, top_view_orange_coordinates_queue):
        while self.cap.isOpened():
            darknet_image = darknet_image_queue.get()
            prev_time = time.time()
            detections = darknet.detect_image(self.network, self.class_names, darknet_image, thresh=setup.args.thresh)
            detections_queue.put(detections)
            ###############################################
            blue, orange = self.get_inv_coor_different_boundary(detections)
            top_view_blue_coordinates_queue.put(blue)
            top_view_orange_coordinates_queue.put(orange)
            ###############################################
            fps = int(1/(time.time() - prev_time))
            fps_queue.put(fps)
            #print("FPS: {}".format(fps))
            #darknet.print_detections(detections, args.ext_output)
            darknet.free_image(darknet_image)
        self.cap.release()