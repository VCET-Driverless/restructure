import numpy as np
import math
import cv2

class Pure_Pursuit:
    def __init__(self):
        self.look_ahead_dist = 180.0
        self.path_lines = []
        self.top_view_image_dimension = (416, 285)
        self.front_view_image_dimension = (416, 416)  # (w, h) = (x, y)
        self.car_below_y = 25  # y coordinate of car below max y coordinate
        self.top_view_car_coordinate = (self.top_view_image_dimension[0]//2, self.top_view_image_dimension[1]+self.car_below_y)
        self.cx, self.cy = self.top_view_car_coordinate
        self.wheelbase = 65
        
        
    def intersect(self, path_lines):
        r = self.look_ahead_dist

        index = 0
        got_a_pt = False
        cx, cy = self.top_view_car_coordinate
        for i in path_lines:
            if ((i[0] - cx) ** 2 + (i[1] - cy) ** 2 - r ** 2 > 0):
                index = path_lines.index(i)
                got_a_pt = True
                break
           #Find a b c
            if got_a_pt:
                try:
                    slope = (path_lines[index][1] - cy - path_lines[index - 1][1] + cy) / (path_lines[index][0] - cx - path_lines[index - 1][0] + cx)
                except:
                    slope = math.inf
                a = slope
                b = -1
                c = -1 * slope * (path_lines[index - 1][0] - cx) + path_lines[index - 1][1] - cy

                x0 = -1 * a * c / (a * a + b * b)
                y0 = -b * c / (a * a + b * b)
                d = r * r - c * c / (a * a + b * b)
                multiplier = math.sqrt(d / (a * a + b * b))
                ax = x0 + b * multiplier
                bx = x0 - b * multiplier
                ay = y0 - a * multiplier
                by = y0 + a * multiplier

                if (ay + cy < cy):
                    return (ax + cx, ay + cy)
                else:
                    return (bx + cx, by + cy)
            else:
                return path_lines[-1]

    def pure_pursuit_control(self, tx,ty):
        self.wheelbase = 65
        alpha = (math.pi / 2) + math.atan2(ty - self.cy - self.wheelbase, tx - self.cx)
        delta = math.atan2(2.0 * self.wheelbase * math.sin(alpha), self.look_ahead_dist)
        return delta,alpha

    def pure_pursuit(self,path_lines,frame):
        alpha = 0
        delta = 0
        w_x , w_y = self.intersect(path_lines)
        alpha,delta = self.pure_pursuit_control(w_x,w_y)
        cv2.line(frame, (self.cx, self.cy), (w_x, w_y), (125, 125, 255), 3)
        cv2.circle(frame, (self.cx, self.cy), int(self.look_ahead_dist), (0, 255, 255), 3)
        cv2.circle(frame, (w_x, w_y), 5, (0, 255, 255), 3)
        delta = delta*180/math.pi
        return delta,frame

    
