import cv2
import numpy as np
import math

path = "http://192.168.43.156:4747/video"

# inv map output-image size
img_dim = (416, 285) # (w, h) = (x, y)

# sampeling speed
BAUD_RATE = 115200

# intel camera 
#pt = [(0,225), (-1500,500), (600,225), (2100,500)]

# Laptop camera 
pt_in = [(0   , 100),
         (-600, 416),
         (416 , 100), 
         (1016, 416)]

pt_out = [(0         , 0         ),
          (0         , img_dim[1]),
          (img_dim[0], 0         ), 
          (img_dim[0], img_dim[1])]

# threshold after which detections won't be considered
# below variable represents threshold 'y' coordinate
LIMIT_CONE = 150 

# when one side is empty of cones, this variable is used as offset
mid_c = 80-5

# car coordinates on image
car_coor = (img_dim[0]//2, img_dim[1]+25)

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
    pts1 = np.float32(pt_in)
    pts2 = np.float32(pt_out)
    M = cv2.getPerspectiveTransform(pts1,pts2)
    image = cv2.warpPerspective(frame, M, img_dim, flags=cv2.INTER_LINEAR)
    #cv2.imshow('itshouldlookfine!', image)
    return image, M

def line_x(direction_coor, cone_coor):
    """
    Finds the position of cone w.r.t., line formed by given point and car coordinates

    :direction_coor: coordinate to form the line
    :cone_coor:      coordinate of cone
    :returns:        position of cone w.r.t., virtual mid-line
    """
    car_x, car_y = car_coor
    cone_x, cone_y = cone_coor
    direction_x, direction_y = direction_coor

    slope = (direction_y - car_y) / (direction_x - car_x)

    x_on_line = (cone_y - car_y)/slope - car_x

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
            if( not line_x( (img_dim[0]*ratio, 0), (x, y) ) ):
                if(left_count > 0):
                    left_box.append(mybox[i])
                    left_count = left_count - 1
            else:
                if(right_count > 0):
                    right_box.append(mybox[i])
                    right_count = right_count - 1

        elif( str_ang == '6' or str_ang == '7' or str_ang == '8' ):
            if( line_x( ( img_dim[0] - img_dim[0]*ratio, 0), (x, y) ) ):
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
        print('Left Exception in pathplan function.............')
            
    try:
        if(right_box[-1][1] < LIMIT_CONE):
            right_box.clear()
    except:
        print('Right Exception in pathplan function.............')
    #############################################################################
    
    lines = []
    lines.append(car_coor)


    if( len(left_box) == 0 and len(right_box) == 0 ):
        lines.append((208,350))
         
    elif( len(left_box) == 0 and len(right_box) != 0 ):
        for i in range(len(right_box)):
            #print( 'test1' )
            x, y = right_box[i]
            x = x - mid_c
            lines.append( (int(x), int(y)) )
        
    elif( len(left_box) != 0 and len(right_box) == 0 ):
        for i in range(len(left_box)):
            #print( 'test2' )
            x, y = left_box[i]
            x = x + mid_c
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
                #cv2.circle(transf,(int(x), int(y)), 5, (255,0,255), -1) 	# Filled
                lines.append( (int(x), int(y)) )

        left_box = left_box[::-1].copy()
        right_box = right_box[::-1].copy()

    lines = sorted(lines, key=lambda m:(m[1], m[0])).copy()
    #print(len(left_box), len(right_box))
    
    return left_box[::-1], right_box[::-1], lines[::-1]

def pathbana(mybox, left_box, right_box, lines, inv_image):
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

    for i in range(len(mybox)):
        cv2.circle(inv_image, mybox[i], 5, (0,255,255), -1)

    for i in range(len(left_box)-1):
        cv2.line(inv_image, left_box[i], left_box[i+1], (125, 125, 255), 3)

    for i in range(len(right_box)-1):
        cv2.line(inv_image, right_box[i], right_box[i+1], (0,0,0), 3)
	
    #print( lines[0], lines[1] , angle(lines[0], lines[1]) )

    return inv_image
