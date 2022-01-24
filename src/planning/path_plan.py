
# Library imports
import cv2
import numpy as np
import time
import datetime
import sys
sys.path.append('../control')
sys.path.append('../system_manager')

# System imports
from control import Control
from constants import Constants
 
class Planning(Constants):
    
    def __init__(self):
        
        super().__init__()
        self.left_box=[]
        self.right_box=[]
        self.lines=[]
        
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
                if( not self.line_x((self.topViewImageDim[0]*ratio, 0), (x, y))):
                    if(left_count > 0):
                        self.left_box.append(mybox[i])
                        left_count = left_count - 1
                else:
                    if(right_count > 0):
                        self.right_box.append(mybox[i])
                        right_count = right_count - 1

            elif( str_ang == '6' or str_ang == '7' or str_ang == '8' ):
                if( self.line_x(( self.topViewImageDim[0] - 
                            self.topViewImageDim[0] * ratio, 0 ), (x, y))):
                    if(right_count > 0):
                        self.right_box.append(mybox[i])
                        right_count = right_count - 1

                else:
                    if(left_count > 0):
                        self.left_box.append(mybox[i])
                        left_count = left_count - 1


        
        
        self.left_box.sort(reverse = True)
        self.right_box.sort(reverse = True)

        self.left_box =  sorted(self.left_box, key=lambda k:(k[1], k[0])).copy()
        self.right_box = sorted(self.right_box, key=lambda l:(l[1], l[0])).copy()
        '''self.left_box.sort()
        self.right_box.sort()'''
        
        try:
            if(self.left_box[-1][1] < self.LIMIT_CONE):
                self.left_box.clear()
        except:
            #print('Left Exception in pathplan function.............')
            pass
                
        try:
            if(self.right_box[-1][1] < self.LIMIT_CONE):
                self.right_box.clear()
        except:
            pass
            #print('Right Exception in pathplan function.............')
    
        
        self.lines = []
        self.lines.append(self.TOP_VIEW_CAR_COORDINATE)


        if( len(self.left_box) == 0 and len(self.right_box) == 0 ):
            self.lines.append((208,350))
            
        elif( len(self.left_box) == 0 and len(self.right_box) != 0 ):
            for i in range(len(self.right_box)):
                #print( 'test1' )
                x, y = self.right_box[i]
                x = x - self.MIDPOINT_ONE_BOUNDARY
                self.lines.append( (int(x), int(y)) )
            
        elif( len(self.left_box) != 0 and len(self.right_box) == 0 ):
            for i in range(len(self.left_box)):
                #print( 'test2' )
                x, y = self.left_box[i]
                x = x + self.MIDPOINT_ONE_BOUNDARY
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

    def line_x(self,direction_coor, cone_coor):
        """
        Finds the position of cone w.r.t., line formed by given point and car coordinates
        :direction_coor: coordinate to form the line
        :cone_coor:      coordinate of cone
        :returns:        position of cone w.r.t., virtual mid-line
        """
       

        car_x, car_y = self.TOP_VIEW_CAR_COORDINATE
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

    def pathplan_different_boundary(self, blue, orange, invert):
        """
        Separates top view coordinates as left and right boundary
        Also uses prior steering angle 
        :mybox:   list having all detections as top view coordinates
        :str_ang: steering angle of previous time-step/frame
        :returns: 3 lists having top view coordinates of left, right and midpoint 
        """
        left_box = []
        right_box = []
        
        if invert:
            left_box, right_box = blue.copy(), orange.copy()
            #print(left_box==blue, right_box==orange, "if")
        else:
            left_box, right_box = orange.copy(), blue.copy()
            #print(left_box==blue, right_box==orange, "else")
        # print(len(left_box), len(right_box))
       
        left_box.sort(reverse = True)
        right_box.sort(reverse = True)

        left_box =  sorted(left_box, key=lambda k:(k[1], k[0])).copy()
        right_box = sorted(right_box, key=lambda l:(l[1], l[0])).copy()
        '''left_box.sort()
        right_box.sort()'''
     
        try:
            if(left_box[-1][1] < self.LIMIT_CONE):
                left_box.clear()
        except:
            #print('Left Exception in pathplan function.............')
            pass
                
        try:
            if(right_box[-1][1] < self.LIMIT_CONE):
                right_box.clear()
        except:
            pass
            #print('Right Exception in pathplan function.............')
     
        
        lines = []
        lines.append(self.TOP_VIEW_CAR_COORDINATE)


        if( len(left_box) == 0 and len(right_box) == 0 ):
            lines.append((208,350))
            
        elif( len(left_box) == 0 and len(right_box) != 0 ):
            for i in range(len(right_box)):
                #print( 'test1' )
                x, y = right_box[i]
                x = x - self.MIDPOINT_ONE_BOUNDARY
                lines.append( (int(x), int(y)) )
            
        elif( len(left_box) != 0 and len(right_box) == 0 ):
            for i in range(len(left_box)):
                #print( 'test2' )
                x, y = left_box[i]
                x = x + self.MIDPOINT_ONE_BOUNDARY
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

    def path_plan_driver(self, setup, detected_images_queue, top_view_frame_queue, top_view_blue_coordinates_queue, top_view_orange_coordinates_queue, log_queue, p1_parent):
        
        queue_is_empty = False
        steering = '4'
        control = Control()
        prev_time = time.time()
        frame_count=0
        
        while True:
           
            if queue_is_empty is False:
                
                top_image = top_view_frame_queue.get()
                blue = top_view_blue_coordinates_queue.get()
                orange = top_view_orange_coordinates_queue.get()

                # Appropriate path planning according to the kind of boundary.
                if setup.args.boundary == 0:
                    left_box, right_box, lines = Planning.pathplan_different_boundary(blue, orange, setup.BOUNDARY_INVERT)
                else:
                    mybox = blue + orange
                    left_box, right_box, lines = Planning.pathplan(mybox, steering)
                
                # Drawing the top view visualization and showing on cv2 window
                top_image = Planning.pathbana(left_box, right_box, lines, top_image)
                
                # Call control module
                steering, st_ang, serial_writen_now, prev_time, top_image = control.control(setup, lines, steering, prev_time, top_image)
                
                # Display the output
                if not setup.args.dont_show:
                    image = detected_images_queue.get()
                    
                    # Visualization of front view
                    cv2.namedWindow("Inference") 
                    cv2.moveWindow("Inference", 1000,30)
                    image = cv2.resize(image, (2*self.TOP_VIEW_IMAGE_DIMESNION[0],
                                               2*self.TOP_VIEW_IMAGE_DIMESNION[0]))
                    cv2.imshow('Inference', image)
                    
                    # Visualization of Top view
                    top_image = cv2.resize(top_image, (2*self.TOP_VIEW_IMAGE_DIMESNION[0],
                                                       2*self.TOP_VIEW_IMAGE_DIMESNION[1]))
                    cv2.imshow('top_view', top_image)                
                
                # updating log queue
                frame_data = {
                        "time_stamp":datetime.datetime.now().astimezone().isoformat(),
                        "frame_count":frame_count,
                        "steering": int(st_ang),
                        "serial_writen_now":serial_writen_now,
                        # "detections": chcone.get_boxes(detections),
                        "left_box":left_box,
                        "right_box":right_box,
                        "lines":lines
                    }
                log_queue.put(frame_data)
                
                # Used to break out of the loop if cv2 window is escaped
                if cv2.waitKey(2) == 27:
                    control.stop_car(setup)     # Send signal to arduino to stop hardware. 
                    p1_parent.send(False)  # Act as a parent and send signal to child i.e detect
                    break
                else:
                    p1_parent.send(True)
                    
                
                # Check if at any point shared memory queue is empty.
                if top_view_frame_queue.empty() or top_view_blue_coordinates_queue.empty() or top_view_orange_coordinates_queue.empty():
                    queue_is_empty = True
                    
                frame_count+=1
            
            else:
                if not top_view_frame_queue.empty() and not top_view_blue_coordinates_queue.empty() and not top_view_orange_coordinates_queue.empty():
                    queue_is_empty = True
                    
                print("Debug planning: one of the queue is empty")
                
        cv2.destroyAllWindows()
        print("Path plan has stopped")
