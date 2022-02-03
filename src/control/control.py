
# Library imports
import numpy as np
import math
import cv2
import time

# System imports
from pure_pursuit import Pure_Pursuit
from system_manager.constants import Constants


class Control(Constants):
    
    def __init__(self):
        
        super().__init__()
        self.limit_frames = 5
        
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
    
    def stop_car(self, setup):
        if self.ARDUINO_CONNECTED:
                setup.s.write(str('c').encode())
                
                
    def send_angle(self, setup, ang):
        if self.ARDUINO_CONNECTED:
                setup.s.write(str(ang).encode())
                    

    def control(self, setup, path, steering, prev_time, top_view_image):
        
        pp = Pure_Pursuit()
        angle = 0
        angle_limit = [0]*self.limit_frames
        path_list = path
        serial_writen_now = False

        if (setup.args.controller == 0):
            # encode signal for steering control(old controller)
            angle = Control.angle(self,path_list[0],path_list[1])
            angle = math.floor(angle)
        elif (setup.args.controller == 1):
            # encode signal for steering control(new controller)
            angle, top_view_image = pp.pure_pursuit(self, path_list, top_view_image)
            angle = math.floor(angle)

        angle_limit.append(angle)
        angle_limit.pop(0)
        angle_send = Control.send_steer_angle((sum(angle_limit))//self.limit_frames)
        st_ang = self.P * (sum(angle_limit)) // self.limit_frames

        if (time.time() - prev_time >= self.MS):
            prev_time = time.time()
            if (self.ARDUINO_CONNECTED):
                serial_data = st_ang
                serial_writen_now = True
                self.send_angle(setup, serial_data)
        else:
            serial_writen_now = False

        # Prevents Arduino buffer overlfow,
        if (steering != angle_send):
            if (self.ARDUINO_CONNECTED):
                '''s.write(str.encode(angle_a))'''
                pass
            steering = angle_send
            print('updated', angle_send)

        return steering, st_ang, serial_writen_now, prev_time, top_view_image
            
