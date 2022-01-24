
# Library imports
import math
import cv2

# System imports
from constants import Constants

class Pure_Pursuit(Constants):
    
    def __init__(self, constants):
        
        super().__init__()
        self.look_ahead_dist = self.Lfc
        self.cx, self.cy = self.TOP_VIEW_CAR_COORDINATE
        self.wheelbase = self.L

    def intersect(self, path_lines):
        r = self.look_ahead_dist

        index = 0
        got_a_pt = False
        for i in path_lines:
            if ((i[0] - self.cx) ** 2 + (i[1] - self.cy) ** 2 - r ** 2 > 0):
                index = path_lines.index(i)
                got_a_pt = True
                break
            # Find a b c
            if got_a_pt:
                try:
                    slope = (path_lines[index][1] - self.cy - path_lines[index - 1][1] + self.cy) / (
                                path_lines[index][0] - self.cx - path_lines[index - 1][0] + self.cx)
                except:
                    slope = math.inf
                a = slope
                b = -1
                c = -1 * slope * (path_lines[index - 1][0] - self.cx) + path_lines[index - 1][1] - self.cy

                x0 = -1 * a * c / (a * a + b * b)
                y0 = -b * c / (a * a + b * b)
                d = r * r - c * c / (a * a + b * b)
                multiplier = math.sqrt(d / (a * a + b * b))
                ax = x0 + b * multiplier
                bx = x0 - b * multiplier
                ay = y0 - a * multiplier
                by = y0 + a * multiplier

                if (ay + self.cy < self.cy):
                    return (ax + self.cx, ay + self.cy)
                else:
                    return (bx + self.cx, by + self.cy)
            else:
                return path_lines[-1]

    def pure_pursuit_control(self, tx, ty):
        
        alpha = (math.pi / 2) + math.atan2(ty - self.cy - self.wheelbase, tx - self.cx)
        delta = math.atan2(2.0 * self.wheelbase * math.sin(alpha), self.look_ahead_dist)
        return delta, alpha

    def pure_pursuit(self, path_lines, frame):
        alpha = 0
        delta = 0
        w_x, w_y = self.intersect(path_lines)
        alpha, delta = self.pure_pursuit_control(w_x, w_y)
        cv2.line(frame, (self.cx, self.cy), (w_x, w_y), (125, 125, 255), 3)
        cv2.circle(frame, (self.cx, self.cy), int(self.look_ahead_dist), (0, 255, 255), 3)
        cv2.circle(frame, (w_x, w_y), 5, (0, 255, 255), 3)
        delta = delta * 180 / math.pi
        return delta, frame
