import cv2
# import darknet
import time
import numpy as np
#IMPORT TOP_VIEW_IMAGE_DIMENSION, M

class Perception:

    def __init__(self, setup):
        self.cap = cv2.VideoCapture(setup.CAM_PATH)
        self.network, self.class_names, self.class_colors = darknet.load_network(
            setup.args.config_file,
            setup.args.data_file,
            setup.args.weights,
            batch_size=1
        )
        
        self.width = darknet.network_width(self.network)
        self.height = darknet.network_height(self.network)

        # blue = self.top_view_blue_coordinates_queue.get()
        # orange = self.top_view_orange_coordinates_queue.get()


    def inv_map(self, frame):
        """
        Transforms given image to top-view image (used for visual debug)
        :frame: front-view image
        :returns: transformation matrix and transformed image
        """
        image = cv2.warpPerspective(frame, M, TOP_VIEW_IMAGE_DIMESNION, flags=cv2.INTER_LINEAR)
        #cv2.imshow('itshouldlookfine!', image)
        return image, M

    
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
            xmin, ymin, xmax, ymax = convertBack(
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


    def video_capture(self, frame_queue, darknet_image_queue):
        while self.capcap.isOpened():
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
            top_view_img_for_draw, top_view_matrix = self.inv_map(frame_resized)
            self.top_view_frame_queue.put(top_view_img_for_draw)
            ###############################################
        self.cap.release()

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
