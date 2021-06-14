import cv2
import numpy as np
import math
from constants import (
    M,
    LIMIT_CONE,
    MIDPOINT_ONE_BOUNDARY,
    TOP_VIEW_CAR_COORDINATE,
    TOP_VIEW_IMAGE_DIMESNION
)
from constants import Lfc,L
 
CX,CY  = TOP_VIEW_CAR_COORDINATE
a = range(-75,-26)#
b = range(-26,-19)#7
c = range(-19,-13)#6
d = range(-13,-7)#6
e = range(-7,0)#8
f = range(0,7)
g = range(7,13)
h = range(13,19)
i = range(19,26)
j = range(26,75)

m = range(-90,-26)
n = range(-26,-12)
o = range(-12,0)
p = range(0,12)
q = range(12,26)
r = range(26,90) 

def steer(angle):
    """
    Maps angle range to integer for sending to Arduino
    :angle:   steering angle
    :returns: mapped integer
    """
    if( angle in a ):
        return '0'
    elif( angle in b ):
        return '1'
    elif( angle in c ):
        return '2'
    elif( angle in d ):
        return '3'
    elif( angle in e ):
        return '4'
    elif( angle in f ):
        return '4'
    elif( angle in g ):
        return '5'
    elif( angle in h ):
        return '6'
    elif( angle in i ):
        return '7'
    elif( angle in j ):
        return '8' 

def angle(p1, p2):
    """
    Computes angle w.r.t., car
    :returns: angle w.r.t, car
    """
    x, y = p1
    p, q = p2
    try:
        slope = (q - y)/(p - x)
    except:
        slope = 99999
    angle = np.arctan(slope)*180/math.pi
    if(angle > 0):
        return -1*(90 - angle)
    return (90 + angle)

def inv_map(frame):
    """
    Transforms given image to top-view image (used for visual debug)
    :frame: front-view image
    :returns: transformation matrix and transformed image
    """
    image = cv2.warpPerspective(frame, M, TOP_VIEW_IMAGE_DIMESNION, flags=cv2.INTER_LINEAR)
    #cv2.imshow('itshouldlookfine!', image)
    return image, M

def line_x(direction_coor, cone_coor):
    """
    Finds the position of cone w.r.t., line formed by given point and car coordinates
    :direction_coor: coordinate to form the line
    :cone_coor:      coordinate of cone
    :returns:        position of cone w.r.t., virtual mid-line
    """
    car_x, car_y = TOP_VIEW_CAR_COORDINATE
    cone_x, cone_y = cone_coor
    direction_x, direction_y = direction_coor

    #slope = (direction_y - cone_y) / (direction_x - cone_x)
    slope = (direction_y - car_y) / (direction_x - car_x)

    x_on_line = (cone_y - car_y)/slope + car_x

    error = cone_x - x_on_line

    if(error >= 0):
        # True indicates right side
        return True
    else:
        # False indicates left side
        return False

def pathplan(mybox, str_ang):
    """
    Separates top view coordinates as left and right boundary
    Also uses prior steering angle 
    :mybox:   list having all detections as top view coordinates
    :str_ang: steering angle of previous time-step/frame
    :returns: 3 lists having top view coordinates of left, right and midpoint 
    """
    left_box = []
    right_box = []
    left_count = 5
    right_count = 5
    ratio = 0.25

    for i in range(len(mybox)):
        x, y = mybox[i]
        if( str_ang == '3' or str_ang == '4' or  str_ang == '5' ):
            if(x < 208):
                if(left_count > 0):
                    left_box.append(mybox[i])
                    left_count = left_count - 1

            else:
                if(right_count > 0):
                    right_box.append(mybox[i])
                    right_count = right_count - 1

        elif( str_ang == '0' or str_ang == '1' or str_ang == '2'):
            if( not line_x( (TOP_VIEW_IMAGE_DIMESNION[0]*ratio, 0), (x, y))):
                if(left_count > 0):
                    left_box.append(mybox[i])
                    left_count = left_count - 1
            else:
                if(right_count > 0):
                    right_box.append(mybox[i])
                    right_count = right_count - 1

        elif( str_ang == '6' or str_ang == '7' or str_ang == '8' ):
            if( line_x( ( TOP_VIEW_IMAGE_DIMESNION[0] - 
                          TOP_VIEW_IMAGE_DIMESNION[0] * ratio, 0 ), (x, y) ) ):
                if(right_count > 0):
                    right_box.append(mybox[i])
                    right_count = right_count - 1

            else:
                if(left_count > 0):
                    left_box.append(mybox[i])
                    left_count = left_count - 1


    
    #############################################################################
    left_box.sort(reverse = True)
    right_box.sort(reverse = True)

    left_box =  sorted(left_box, key=lambda k:(k[1], k[0])).copy()
    right_box = sorted(right_box, key=lambda l:(l[1], l[0])).copy()
    '''left_box.sort()
    right_box.sort()'''
    #############################################################################
    ############################### path planning ###############################
    #############################################################################
    try:
        if(left_box[-1][1] < LIMIT_CONE):
            left_box.clear()
    except:
        #print('Left Exception in pathplan function.............')
        pass
            
    try:
        if(right_box[-1][1] < LIMIT_CONE):
            right_box.clear()
    except:
        pass
        #print('Right Exception in pathplan function.............')
    #############################################################################
    
    lines = []
    lines.append(TOP_VIEW_CAR_COORDINATE)


    if( len(left_box) == 0 and len(right_box) == 0 ):
        lines.append((208,350))
         
    elif( len(left_box) == 0 and len(right_box) != 0 ):
        for i in range(len(right_box)):
            #print( 'test1' )
            x, y = right_box[i]
            x = x - MIDPOINT_ONE_BOUNDARY
            lines.append( (int(x), int(y)) )
        
    elif( len(left_box) != 0 and len(right_box) == 0 ):
        for i in range(len(left_box)):
            #print( 'test2' )
            x, y = left_box[i]
            x = x + MIDPOINT_ONE_BOUNDARY
            lines.append( (int(x), int(y)) )
        
    elif( len(left_box) != 0 and len(right_box) != 0 ):

        small_len  = 0
        left_box = left_box[::-1].copy()
        right_box = right_box[::-1].copy()
        if(len(left_box) > len(right_box)):
            small_len = len(right_box)
        else:
            small_len = len(left_box)
        
        for i in reversed(range(small_len)):
                #print( 'test3' )
                x, y = tuple(np.add((right_box[i]), (left_box[i])))
                x = x//2
                y = y//2
                #cv2.circle(transf,(int(x), int(y)), 5, (255,0,255), -1)    # Filled
                lines.append( (int(x), int(y)) )

        left_box = left_box[::-1].copy()
        right_box = right_box[::-1].copy()

    lines = sorted(lines, key=lambda m:(m[1], m[0])).copy()
    #print(len(left_box), len(right_box))
    
    return left_box[::-1], right_box[::-1], lines[::-1]

def pathbana(left_box, right_box, lines, inv_image):
    """
    JUST DRAWING function > draws left, right and midpoint top-view coordinates
    :mybox:     list with all coordinates (top-view)
    :left_box:  list with left side coordinates (top-view)
    :right_box: list with right side coordinates (top-view)
    :lines:     list with midpoint coordinates (top-view)
    :inv_image: top-view image
    :returns:   image with lines drawn
    """
    for i in range(len(lines) - 1):
        cv2.circle(inv_image,lines[i], 5, (0,0,0), -1)
        cv2.line(inv_image,lines[i],lines[i+1],(255,255,0),4)
    '''if(angle(lines[0], lines[1]) > 75 or angle(lines[0], lines[1]) < -75):
        lines.remove(1)'''
    mybox = left_box + right_box
    for i in range(len(mybox)):
        cv2.circle(inv_image, mybox[i], 5, (0,255,255), -1)

    for i in range(len(left_box)-1):
        cv2.line(inv_image, left_box[i], left_box[i+1], (125, 125, 255), 3)

    for i in range(len(right_box)-1):
        cv2.line(inv_image, right_box[i], right_box[i+1], (0,0,0), 3)
    
    #print( lines[0], lines[1] , angle(lines[0], lines[1]) )

    return inv_image

def convertBack(x, y, w, h):
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


def get_inv_coor(detections):
    """
    Converts front-view coordinates (of cone) to top-view coordinates
    :detections: front-view coordinates
    :M: transformation matrix
    :returns: top-view coordinates of cones and person
    """
    mybox = []
    person = []
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
        #if(detection[0].decode() == 'person'):
        #    person.append(box)
        #elif(box[1]>0):
        mybox.append(box)
    
    mybox = sorted(mybox, key=lambda k:(k[1], k[0])).copy()
    #print(mybox[::-1],'\n')

    return person, mybox[::-1]

def get_inv_coor_different_boundary(detections):
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

def pathplan_different_boundary(blue, orange, invert):
    """
    Separates top view coordinates as left and right boundary
    Also uses prior steering angle 
    :mybox:   list having all detections as top view coordinates
    :str_ang: steering angle of previous time-step/frame
    :returns: 3 lists having top view coordinates of left, right and midpoint 
    """
    left_box = []
    right_box = []
    left_count = 5
    right_count = 5

    if invert:
        left_box, right_box = blue.copy(), orange.copy()
        #print(left_box==blue, right_box==orange, "if")
    else:
        left_box, right_box = orange.copy(), blue.copy()
        #print(left_box==blue, right_box==orange, "else")
    # print(len(left_box), len(right_box))
    #############################################################################
    left_box.sort(reverse = True)
    right_box.sort(reverse = True)

    left_box =  sorted(left_box, key=lambda k:(k[1], k[0])).copy()
    right_box = sorted(right_box, key=lambda l:(l[1], l[0])).copy()
    '''left_box.sort()
    right_box.sort()'''
    #############################################################################
    ############################### path planning ###############################
    #############################################################################
    try:
        if(left_box[-1][1] < LIMIT_CONE):
            left_box.clear()
    except:
        #print('Left Exception in pathplan function.............')
        pass
            
    try:
        if(right_box[-1][1] < LIMIT_CONE):
            right_box.clear()
    except:
        pass
        #print('Right Exception in pathplan function.............')
    #############################################################################
    
    lines = []
    lines.append(TOP_VIEW_CAR_COORDINATE)


    if( len(left_box) == 0 and len(right_box) == 0 ):
        lines.append((208,350))
         
    elif( len(left_box) == 0 and len(right_box) != 0 ):
        for i in range(len(right_box)):
            #print( 'test1' )
            x, y = right_box[i]
            x = x - MIDPOINT_ONE_BOUNDARY
            lines.append( (int(x), int(y)) )
        
    elif( len(left_box) != 0 and len(right_box) == 0 ):
        for i in range(len(left_box)):
            #print( 'test2' )
            x, y = left_box[i]
            x = x + MIDPOINT_ONE_BOUNDARY
            lines.append( (int(x), int(y)) )
        
    elif( len(left_box) != 0 and len(right_box) != 0 ):

        small_len  = 0
        left_box = left_box[::-1].copy()
        right_box = right_box[::-1].copy()
        if(len(left_box) > len(right_box)):
            small_len = len(right_box)
        else:
            small_len = len(left_box)
        
        for i in reversed(range(small_len)):
                #print( 'test3' )
                x, y = tuple(np.add((right_box[i]), (left_box[i])))
                x = x//2
                y = y//2
                #cv2.circle(transf,(int(x), int(y)), 5, (255,0,255), -1)    # Filled
                lines.append( (int(x), int(y)) )

        left_box = left_box[::-1].copy()
        right_box = right_box[::-1].copy()

    lines = sorted(lines, key=lambda m:(m[1], m[0])).copy()
    #print(len(left_box), len(right_box))
    
    return left_box[::-1], right_box[::-1], lines[::-1]

def bbox2points(bbox):
    """
    From bounding box yolo format
    to corner points cv2 rectangle
    """
    x, y, w, h = bbox
    xmin = int(round(x - (w / 2)))
    xmax = int(round(x + (w / 2)))
    ymin = int(round(y - (h / 2)))
    ymax = int(round(y + (h / 2)))
    return (xmin, ymin, xmax, ymax)

def get_boxes(detections):
    bounding_box = []
    for label, confidence, bbox in detections:
        temp = bbox2points(bbox)
        bounding_box.append( (label, 
                              confidence, 
                              temp) )
    return bounding_box



##################### Pure Pursuit ########################
#midpoint_ld = 3
#dt = 0.1  # Time interval, unit:s

def intersect(lines, Lfc):
    r = Lfc
    ind = 0
    got_a_pt = False
    for i in lines:
        if((i[0] - CX)**2 + (i[1] - CY)**2 - r**2 > 0):
            ind = lines.index(i)
            got_a_pt = True
            break
    # find a b c
    if got_a_pt:
        try:
            m = (lines[ind][1]-CY - lines[ind-1][1]+CY) / (lines[ind][0]-CX - lines[ind-1][0]+CX)
        except:
            m = math.inf
        a = m
        b = -1
        c = -1 * m*(lines[ind-1][0]-CX) + lines[ind-1][1]-CY

        x0 = -1 * a*c/(a*a+b*b)
        y0 = -b*c/(a*a+b*b)
        d = r*r - c*c/(a*a+b*b)
        mult = math.sqrt(d / (a*a+b*b))
        ax = x0 + b * mult
        bx = x0 - b * mult
        ay = y0 - a * mult
        by = y0 + a * mult


        if(ay+CY < CY):
            return (ax+CX, ay+CY)
        else:
            return (bx+CX, by+CY)
    else:
        return lines[-1]
    
def pure_pursuit_control(tx, ty):
    alpha = (math.pi/2) + math.atan2(ty - CY - L, tx - CX)
    delta = math.atan2(2.0 * L * math.sin(alpha), Lfc)
    return delta, alpha

def PP(lines, frame):
    alpha = 0
    delta = 0
    w_x, w_y = intersect(lines, Lfc)
    delta, alpha = pure_pursuit_control( w_x, w_y)
    cv2.line(frame, (CX, CY), (w_x, w_y), (125, 125, 255), 3)
    cv2.circle(frame, (CX, CY), int(Lfc), (0,255,255), 3)
    cv2.circle(frame, (w_x, w_y), 5, (0,255,255), 3)
    delta = delta*180/math.pi
    return delta, frame

##################################################################################
