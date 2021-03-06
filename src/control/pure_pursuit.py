
# Library imports
import math
import cv2

# System imports


class Pure_Pursuit:
    
    def __init__(self, constants):
        self.look_ahead_dist = constants.Lfc
        self.path_lines = []
        self.top_view_image_dimension = constants.TOP_VIEW_IMAGE_DIMESNION
        self.front_view_image_dimension = constants.FRONT_VIEW_IMAGE_DIMESNION  # (w, h) = (x, y)
        self.car_below_y = constants.CAR_BELOW_Y  # y coordinate of car below max y coordinate
        self.top_view_car_coordinate = constants.TOP_VIEW_CAR_COORDINATE
        self.cx, self.cy = constants.top_view_car_coordinate
        self.wheelbase = constants.L

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
            # Find a b c
            if got_a_pt:
                try:
                    slope = (path_lines[index][1] - cy - path_lines[index - 1][1] + cy) / (
                                path_lines[index][0] - cx - path_lines[index - 1][0] + cx)
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

    def pure_pursuit_control(self, tx, ty):
        self.wheelbase = 65
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
