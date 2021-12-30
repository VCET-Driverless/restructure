import cv2
import numpy as np
import math
from constants import M, LIMIT_CONE, MIDPOINT_ONE_BOUNDARY, TOP_VIEW_CAR_COORDINATE, TOP_VIEW_IMAGE_DIMESNION
from constants import Lfc,L

class path_plan:
    def __init__(self):
        
        self.left_box=[]
        self.right_box=[]
        self.lines=[]
        #top_image=[]
        CX,CY  = TOP_VIEW_CAR_COORDINATE
        
    def pathbana(self,left_box, right_box, lines, inv_image):
        """
        JUST DRAWING function > draws left, right and midpoint top-view coordinates
        :mybox:     list with all coordinates (top-view)
        :self.left_box:  list with left side coordinates (top-view)
        :self.right_box: list with right side coordinates (top-view)
        :self.lines:     list with midpoint coordinates (top-view)
        :inv_image: top-view image
        :returns:   image with self.lines drawn
        """
        self.left_box = left_box
        self.right_box = right_box
        self.lines = lines
        

        for i in range(len(self.lines) - 1):
            cv2.circle(inv_image,self.lines[i], 5, (0,0,0), -1)
            cv2.line(inv_image,self.lines[i],self.lines[i+1],(255,255,0),4)
        '''if(angle(self.lines[0], self.lines[1]) > 75 or angle(self.lines[0], self.lines[1]) < -75):
            self.lines.remove(1)'''
        mybox = self.left_box + self.right_box
        for i in range(len(mybox)):
            cv2.circle(inv_image, mybox[i], 5, (0,255,255), -1)

        for i in range(len(self.left_box)-1):
            cv2.line(inv_image, self.left_box[i], self.left_box[i+1], (125, 125, 255), 3)

        for i in range(len(self.right_box)-1):
            cv2.line(inv_image, self.right_box[i], self.right_box[i+1], (0,0,0), 3)
        
        #print( self.lines[0], self.lines[1] , angle(self.lines[0], self.lines[1]) )

        return inv_image


    def pathplan(self,mybox, str_ang):
        """
        Separates top view coordinates as left and right boundary
        Also uses prior steering angle 
        :mybox:   list having all detections as top view coordinates
        :str_ang: steering angle of previous time-step/frame
        :returns: 3 lists having top view coordinates of left, right and midpoint 
        """
        
        
        self.left_box = []
        self.right_box = []
        left_count = 5
        right_count = 5
        ratio = 0.25

        for i in range(len(mybox)):
            x, y = mybox[i]
            if( str_ang == '3' or str_ang == '4' or  str_ang == '5' ):
                if(x < 208):
                    if(left_count > 0):
                        self.left_box.append(mybox[i])
                        left_count = left_count - 1

                else:
                    if(right_count > 0):
                        self.right_box.append(mybox[i])
                        right_count = right_count - 1

            elif( str_ang == '0' or str_ang == '1' or str_ang == '2'):
                if( not self.line_x((TOP_VIEW_IMAGE_DIMESNION[0]*ratio, 0), (x, y))):
                    if(left_count > 0):
                        self.left_box.append(mybox[i])
                        left_count = left_count - 1
                else:
                    if(right_count > 0):
                        self.right_box.append(mybox[i])
                        right_count = right_count - 1

            elif( str_ang == '6' or str_ang == '7' or str_ang == '8' ):
                if( self.line_x(( TOP_VIEW_IMAGE_DIMESNION[0] - 
                            TOP_VIEW_IMAGE_DIMESNION[0] * ratio, 0 ), (x, y))):
                    if(right_count > 0):
                        self.right_box.append(mybox[i])
                        right_count = right_count - 1

                else:
                    if(left_count > 0):
                        self.left_box.append(mybox[i])
                        left_count = left_count - 1


        
        #############################################################################
        self.left_box.sort(reverse = True)
        self.right_box.sort(reverse = True)

        self.left_box =  sorted(self.left_box, key=lambda k:(k[1], k[0])).copy()
        self.right_box = sorted(self.right_box, key=lambda l:(l[1], l[0])).copy()
        '''self.left_box.sort()
        self.right_box.sort()'''
        #############################################################################
        ############################### path planning ###############################
        #############################################################################
        try:
            if(self.left_box[-1][1] < LIMIT_CONE):
                self.left_box.clear()
        except:
            #print('Left Exception in pathplan function.............')
            pass
                
        try:
            if(self.right_box[-1][1] < LIMIT_CONE):
                self.right_box.clear()
        except:
            pass
            #print('Right Exception in pathplan function.............')
        #############################################################################
        
        self.lines = []
        self.lines.append(TOP_VIEW_CAR_COORDINATE)


        if( len(self.left_box) == 0 and len(self.right_box) == 0 ):
            self.lines.append((208,350))
            
        elif( len(self.left_box) == 0 and len(self.right_box) != 0 ):
            for i in range(len(self.right_box)):
                #print( 'test1' )
                x, y = self.right_box[i]
                x = x - MIDPOINT_ONE_BOUNDARY
                self.lines.append( (int(x), int(y)) )
            
        elif( len(self.left_box) != 0 and len(self.right_box) == 0 ):
            for i in range(len(self.left_box)):
                #print( 'test2' )
                x, y = self.left_box[i]
                x = x + MIDPOINT_ONE_BOUNDARY
                self.lines.append( (int(x), int(y)) )
            
        elif( len(self.left_box) != 0 and len(self.right_box) != 0 ):

            small_len  = 0
            self.left_box = self.left_box[::-1].copy()
            self.right_box = self.right_box[::-1].copy()
            if(len(self.left_box) > len(self.right_box)):
                small_len = len(self.right_box)
            else:
                small_len = len(self.left_box)
            
            for i in reversed(range(small_len)):
                    #print( 'test3' )
                    x, y = tuple(np.add((self.right_box[i]), (self.left_box[i])))
                    x = x//2
                    y = y//2
                    #cv2.circle(transf,(int(x), int(y)), 5, (255,0,255), -1)    # Filled
                    self.lines.append( (int(x), int(y)) )

            self.left_box = self.left_box[::-1].copy()
            self.right_box = self.right_box[::-1].copy()

        self.lines = sorted(self.lines, key=lambda m:(m[1], m[0])).copy()
        #print(len(self.left_box), len(self.right_box))
        
        return self.left_box[::-1], self.right_box[::-1], self.lines[::-1]

    def angle(self,p1, p2):
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


    def steer(self,angle):
    
        """Maps angle range to integer for sending to Arduino
        :angle:   steering angle
        :returns: mapped integer"""

    
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
        # m = range(-90,-26)
        # n = range(-26,-12)
        # o = range(-12,0)
        # p = range(0,12)
        # q = range(12,26)
        # r = range(26,90)
        
        if(angle in a ):
            return '0'
        elif(angle in b ):
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
        
        

    def line_x(self,direction_coor, cone_coor):
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