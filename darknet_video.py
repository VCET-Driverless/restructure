from ctypes import *
import math
import random
import os
import cv2
import numpy as np
import time
import darknet
import chcone
import serial

DANGER = 150
MAX_CONELESS_FRAMES = 30
ARDUINO_CONNECTED = False
#cam_path = 3
cam_path = 'test.mp4'
#cam_path = 'http://192.168.43.156:4747/video'

a = range(-75,-26)
b = range(-26,-19)
c = range(-19,-13)
d = range(-13,-7)
e = range(-7,0)
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

# initializing which serial port to connect
if(ARDUINO_CONNECTED):
    try:
        s=serial.Serial('/dev/ttyACM0',chcone.BAUD_RATE)
        print("Connecting to : /dev/ttyACM0")
    except:
        try:
            print("failed...")
            s=serial.Serial('/dev/ttyACM1',chcone.BAUD_RATE)
            print("Connecting to : /dev/ttyACM1")
        except:
            print("failed... give port premission")
	
# prevents : "car starts to move before program starts to execute"
time.sleep(1.5)


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
    
    
    '''if( angle in m ):
        return '0'
    elif( angle in n ):
        return '1'
    elif( angle in o or angle in p):
        return '2'
    elif( angle in q):
        return '3'
    elif( angle in r):
        return '4'
    return '2'
    else:
        print("OUT OF ANGLE!!!")'''

def personDistance(person_coor):
	p_x, p_y = person_coor
	c_x, c_y = chcone.car_coor
	distance = math.sqrt( (p_x - c_x)**2 + (p_y - c_y)**2 )
	return distance

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

def cvDrawBoxes(detections, img):
    """
    *Currently disabled* 
    Draws bounding box on image (front-view)
    """
    for detection in detections:
        x, y, w, h = detection[2][0],\
            detection[2][1],\
            detection[2][2],\
            detection[2][3]
        xmin, ymin, xmax, ymax = convertBack(
            float(x), float(y), float(w), float(h))
        pt1 = (xmin, ymin)
        pt2 = (xmax, ymax)
        cv2.circle(img,pt1, 5, (255,255,255), -1)
        '''cv2.rectangle(img, pt1, pt2, (0, 255, 0), 1)
        cv2.putText(img,
                    detection[0].decode() +
                    " [" + str(round(detection[1] * 100, 2)) + "]",
                    (pt1[0], pt1[1] - 5), cv2.FONT_HERSHEY_SIMPLEX, 0.5,
                    [0, 255, 0], 2)'''
    return img
     
def get_inv_coor(detections, M):
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
        cv2.circle(img,pt1, 5, (255,255,255), -1)
        a = np.array([[( (xmax+xmin)//2 ), (ymax//1)]], dtype='float32')
        a = np.array([a])
        pointsOut = cv2.perspectiveTransform(a, M)
        box = pointsOut[0][0][0], pointsOut[0][0][1]
        if(detection[0].decode('utf-8') == 'person'):
        	person.append(box)
        elif(box[1]>0):
        	mybox.append(box)
    
    mybox = sorted(mybox, key=lambda k:(k[1], k[0])).copy()
    #print(mybox[::-1],'\n')

    return person, mybox[::-1]



netMain = None
metaMain = None
altNames = None


def YOLO():

    global metaMain, netMain, altNames
    configPath = "./tiny9000.cfg"
    weightPath = "./tiny_10000.weights"
    metaPath = "./data/obj.data"
    if not os.path.exists(configPath):
        raise ValueError("Invalid config path `" +
                         os.path.abspath(configPath)+"`")
    if not os.path.exists(weightPath):
        raise ValueError("Invalid weight path `" +
                         os.path.abspath(weightPath)+"`")
    if not os.path.exists(metaPath):
        raise ValueError("Invalid data file path `" +
                         os.path.abspath(metaPath)+"`")
    if netMain is None:
        netMain = darknet.load_net_custom(configPath.encode(
            "ascii"), weightPath.encode("ascii"), 0, 1)  # batch size = 1
    if metaMain is None:
        metaMain = darknet.load_meta(metaPath.encode("ascii"))
    if altNames is None:
        try:
            with open(metaPath) as metaFH:
                metaContents = metaFH.read()
                import re
                match = re.search("names *= *(.*)$", metaContents,
                                  re.IGNORECASE | re.MULTILINE)
                if match:
                    result = match.group(1)
                else:
                    result = None
                try:
                    if os.path.exists(result):
                        with open(result) as namesFH:
                            namesList = namesFH.read().strip().split("\n")
                            altNames = [x.strip() for x in namesList]
                except TypeError:
                    pass
        except Exception:
            pass
    
    cap = cv2.VideoCapture(cam_path)
    cap.set(3, 1280)
    cap.set(4, 720)
    #out = cv2.VideoWriter(
    #    "output.avi", cv2.VideoWriter_fourcc(*"MJPG"), 10.0,
    #    (darknet.network_width(netMain), darknet.network_height(netMain)))
    print("Starting the YOLO loop...")

    # Create an image we reuse for each detect
    darknet_image = darknet.make_image(darknet.network_width(netMain),
                                    darknet.network_height(netMain),3)
        
    #s.write(str.encode('a'))
    counter = 0
    steering = '2'
    limit_frames = 5
    angle_limit = [0]*limit_frames

    while cap.isOpened():
        try:
            prev_time = time.time()
            ret, frame_read = cap.read()
            frame_rgb = cv2.cvtColor(frame_read, cv2.COLOR_BGR2RGB)
            frame_resized = cv2.resize(frame_rgb,
                                       (darknet.network_width(netMain),
                                        darknet.network_height(netMain)),
                                        interpolation=cv2.INTER_LINEAR)
            darknet.copy_image_from_bytes(darknet_image,frame_resized.tobytes())

            detections = darknet.detect_image(netMain, metaMain, 
                                              darknet_image, thresh=0.25)
            image = cvDrawBoxes(detections, frame_resized)
            image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)

            ######################################################
            #############   AAPNA CODE  ##########################
            ######################################################

            # Walkthrough:
            # 1. get frame from camera
            # 2. run object detector and get bounding box
            # 3. transform front-view to top-view
            #    (aka., inverse perspective transform)
            # 4. mark cones as left or right
            # 5. generate midpoint coordinates
            # 6. find angle of a line pointing to first midpoint
            #    (w.r.t., y-axis)
            # 7. send this angle to Arduino control
            # 8. bus aur kuch nahi, simple sa code hai
            #    abhi tho bhoot kuch naya implement karna hai


            # simple inv transform
            inv_image, M = chcone.inv_map(image)

            # getting inv coordinates on person and cone
            person, mybox = get_inv_coor(detections, M)

            # alert in case of threat *Trial*
            '''
            if(len(person) != 0):
            	for human in person:
            		cv2.circle(inv_image, human, 5, (0,0,255), -1)
            		distance = personDistance(human)
            		if(distance <= DANGER):
            			print("U wanna Die?")
            		else:
            			print("hatt be")
            '''
            
            # separates cones and makes a mid-point path
            left_box, right_box, lines = chcone.pathplan(mybox, steering)
            
            # stop the car if no cones found for *MAX_CONELESS_FRAMES* frames
            if len(mybox) == 0:
                counter = counter + 1
                if counter == MAX_CONELESS_FRAMES:
                    if(ARDUINO_CONNECTED):
                        s.write(str.encode('c'))
                    counter = 0

            # encode signal for steering control
            angle = chcone.angle(lines[0], lines[1])
            angle = math.floor(angle)

            # Takes average turning/steering angle of *limit_frames* frames
            angle_limit.append(angle)
            angle_limit.pop(0)
            angle_a = steer( (sum(angle_limit))//limit_frames )
            print( (sum(angle_limit))//limit_frames )
            
            # Prevents Arduino buffer overlfow,   
            if(steering != angle_a):
                if(ARDUINO_CONNECTED):
                    s.write(str.encode(angle_a))
                steering = angle_a
                print( 'updated' )

            # *JUST DRAWING* for front-view
            inv_image = chcone.pathbana(mybox, left_box, right_box, lines, inv_image)
            cv2.circle(image,chcone.pt[0], 5, (255,255,255), -1)
            cv2.circle(image,chcone.pt[1], 5, (255,255,255), -1)
            cv2.circle(image,chcone.pt[2], 5, (255,255,255), -1)
            cv2.circle(image,chcone.pt[3], 5, (255,255,255), -1)
            
            # *JUST DRAWING* for top-view
            cv2.circle(inv_image, chcone.car_coor, DANGER, (0,0,225), 2)
            cv2.line(inv_image, (0, chcone.LIMIT_CONE), (416, chcone.LIMIT_CONE), (0,0,255), 1)
            if( steering == '4' ):
                cv2.line(inv_image, (chcone.img_dim[0]//2, 0), chcone.car_coor, (0,225,255), 1)
            elif( steering == '0' or steering == '1' or steering == '2' or steering == '3' ):
                cv2.line(inv_image, (chcone.img_dim[0]*chcone.ratio, 0), 
                                     chcone.car_coor, (0,225,255), 1)
            else:
                cv2.line(inv_image, (chcone.img_dim[0] - chcone.img_dim[0]*chcone.ratio, 0), 
                                     chcone.car_coor, (0,225,255), 1)                

            # shows image on screen
            image = cv2.resize(image, (800, 800))
            cv2.imshow('image', image)
            inv_image = cv2.resize(inv_image, (2*chcone.img_dim[0], 2*chcone.img_dim[0]))    
            cv2.imshow('transform', inv_image)
        
            # clear lists
            mybox.clear()
            left_box.clear()
            right_box.clear()
            lines.clear()
            
    	    # dont know why? but necessary
            cv2.waitKey(3)
        except Exception as e:
            print(e)
            print('Exception aaya hai!!!!')

            # any kind of exception must stop the car
            if(ARDUINO_CONNECTED):
                s.write(str.encode('c'))
            break
    cap.release()
    #out.release()
if __name__ == "__main__":
    YOLO()
