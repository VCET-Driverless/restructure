
# Library imports
import numpy as np
import math
import cv2
import time

# System imports
from pure_pursuit import Pure_Pursuit


#sending top view image to the control module is left  -> Done, sent to control.
#top view image is needed to draw pure pursuit path

class Control:
    
    def __init__(self, constants):
        
        self.P = constants.P
        self.limit_frames = 5
        self.rate = constants.RATE
        self.MS = constants.MS
        self.top_view_img_dim = constants.TOP_VIEW_IMAGE_DIMESNION
        
    def send_steer_angle(angle):
        """
         Maps angle range to integer for sending to Arduino
        :angle:   steering angle
        :returns: mapped integer
        """
        if (angle in range(-75, -26)):  #
            return '0'
        elif (angle in range(-26, -19)):  # 7
            return '1'
        elif (angle in range(-19, -13)):  # 6
            return '2'
        elif (angle in range(-13, -7)):  # 6
            return '3'
        elif (angle in range(-7, 0)):  # 8
            return '4'
        elif (angle in range(0, 7)):
            return '4'
        elif (angle in range(7, 13)):
            return '5'
        elif (angle in range(13, 19)):
            return '6'
        elif (angle in range(19, 26)):
            return '7'
        elif (angle in range(26, 75)):
            return '8'

    def angle(self,lane1,lane2):
        """
        Computes angle w.r.t., car
        :returns: angle w.r.t, car
        """
        x, y = lane1
        p, q = lane2
        try:
            slope = (q - y) / (p - x)
        except:
            slope = 99999
        angle = np.arctan(slope) * 180 / math.pi
        if (angle > 0):
            return -1 * (90 - angle)
        return (90 + angle)

    def control(self, setup, path_queue, constants, top_view_image, p2_child):
        
        pp = Pure_Pursuit(constants)
        angle = 0
        steering = '4'
        angle_limit = [0]*self.limit_frames
        prev_time_my = time.time()
                    
        lines = path_queue.get()

        if (setup.args.controller == 0):
            # encode signal for steering control(old controller)
            angle = Control.angle(self,lines[0],lines[1])
            angle = math.floor(angle)
        elif (setup.args.controller == 1):
            # encode signal for steering control(new controller)
            angle, top_image = pp.pure_pursuit(self,lines,top_view_image)
            angle = math.floor(angle)

        angle_limit.append(angle)
        angle_limit.pop(0)
        angle_send = Control.send_steer_angle((sum(angle_limit))//self.limit_frames)
        st_ang = self.P * (sum(angle_limit)) // self.limit_frames

        if (time.time() - prev_time_my >= self.MS):
            prev_time_my = time.time()
            if (setup.ARDUINO_CONNECTED):
                serial_data = st_ang
                serial_writen_now = True
                setup.s.write(str(serial_data).encode())

            # Prevents Arduino buffer overlfow,
        if (steering != angle_send):
            if (setup.ARDUINO_CONNECTED):
                '''s.write(str.encode(angle_a))'''
                pass
            steering = angle_send
            print('updated', angle_send)

        if not setup.args.dont_show:
            top_image = cv2.resize(top_image, (2 * self.top_view_img_dim[0],
                                               2 * self.top_view_img_dim[1]))
            cv2.imshow('top_view', top_image)

        if cv2.waitKey(2) == 27:
            if setup.ARDUINO_CONNECTED:
                setup.s.write(str('c').encode())

            # This will be added later in wip branch
            # if p2_child.recv()==False:
            #     break
