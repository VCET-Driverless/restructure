
import math
import numpy as np
import matplotlib.pyplot as plt
import time
import datetime
import cv2
from sklearn.linear_model import LinearRegression
from sklearn.preprocessing import PolynomialFeatures
from a_star import AStarPlanner


# System imports
from control.control import Control
from system_manager.constants import Constants

class Planning(Constants):
    
    def __init__(self):
        
        super().__init__()
        self.left_box=[]
        self.right_box=[]
        self.lines=[]
        self.s_arr = []
        self.ox = []
        self.oy = []
        self.sx = []
        self.sy = []


            
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

    def sorted_arr(self, lines,ox,oy,sx,sy):
        
        self.sx = lines[0][0]
        self.sy = lines[0][1] 
         
        self.ox.append(sx - 50)
        self.oy.append(sy + 10)
        self.ox.append(sx + 50)
        self.oy.append(sy + 10)
        
    
        obstacles=list(zip(ox,oy))
        s_arr=sorted(obstacles,key= lambda x:x[1])
        print("after sorting : ", s_arr)
        s_arr = np.array(s_arr)
        
        return s_arr,sx,sy

    def set_goal_pt(self,s_arr,sx,sy):

            s_ob = self.s_arr
            x =[]
            y =[]
            for i in range(len(self.s_ob)):
                x.append(self.s_ob[i][0])
                y.append(self.s_ob[i][1])
                    
                # x.append(lines [0][0])
                # y.append(lines[0][1])
              

            # print("sorted_arr: ",sorted_arr)
            # print(x)
            x = np.array(x)
            y = np.array(y)
            x = x.reshape((-1, 1))
            y = y.reshape((-1, 1))
            model = LinearRegression() 
            y_poly = PolynomialFeatures(degree=1).fit_transform(y)
            model.fit(y_poly,x)
            # print("y_poly :",y_poly)
            x_pred = model.predict(y_poly)

            x_angle1= x_pred[0]
            x_angle2= x_pred[2]

            y_angle1 = y[0]
            y_angle2 = y[2]

            reg_inc = (y_angle2 - y_angle1)/ (x_angle2 - x_angle1)
            reg_angle = round(math.degrees(math.atan(reg_inc)))
            print("reg_inc :",reg_inc)
            print("reg_angle",reg_angle)

        

            distance = 70              
            # Mathematics
            x_far_cone = s_ob[0][0]
            y_far_cone = s_ob[0][1]
            x_near_cone = self.sx
            y_near_cone = self.sy 
            slope_from_car = (y_far_cone - y_near_cone)/(x_far_cone - x_near_cone)
            print("slope of car :",slope_from_car)

            angle_from_car = round(math.degrees(math.atan(slope_from_car)))

     

            gx = 0
            gy = 0


            if reg_angle in range(80 , 90) or reg_angle in range(-89,-80):

                    if angle_from_car < 0 :
                         gx = s_ob[0][0] - distance
                         gy = s_ob[0][1]
                         print("case 1")
                    elif angle_from_car > 0:
                         gx = s_ob[0][0] + distance
                         gy = s_ob[0][1]
                         print("case 2")


            else:


                if reg_angle < 0:
                    if reg_angle == angle_from_car:
                        gx = s_ob[0][0] + distance
                        gy = s_ob[0][1]
                        print("case eq1")

                    elif reg_angle > angle_from_car:
                        gx = s_ob[0][0] + distance
                        gy = s_ob[0][1]
                        print("case3")
                    elif reg_angle < angle_from_car:    
                         gx = s_ob[0][0] - distance
                         gy = s_ob[0][1]
                         print("case4")

                elif reg_angle > 0:
                    if reg_angle == angle_from_car:
                        gx = s_ob[0][0] - distance
                        gy = s_ob[0][1]
                        print("case eq2")

                    elif reg_angle < angle_from_car:
                         gx = s_ob[0][0] + distance
                         gy = s_ob[0][1]
                         print("case5")
                    elif reg_angle > angle_from_car:
                         gx = s_ob[0][0] - distance
                         gy = s_ob[0][1]
                         print("case6")
                    
            print("angle_from_car : ", angle_from_car)
            print("gx : ", gx)
            print("gy : ", gy)
            return gx ,gy

    def obstacle_list_update(self,ox,oy,gx,gy):
        
            # Appending to bring goal point in frame
            self.ox.append(self.gx - 50)
            self.oy.append(self.gy - 30)
            self.ox.append(self.gx + 50)
            self.oy.append(self.gy - 30)

            return ox, oy

     def path_plan_driver(self, ox,oy,sx,sy, detected_images_queue, top_view_frame_queue, top_view_blue_coordinates_queue, top_view_orange_coordinates_queue, log_queue, p1_parent):
        
        queue_is_empty = False
        steering = '4'
        control = Control()
        prev_time = time.time()
        frame_count=0

        grid_size = 5.0  # [m]
        robot_radius= 1.0  # [m]  robot_radius>grid_size always!!!

        while True:
           
            if queue_is_empty is False:
                
                top_image = top_view_frame_queue.get()
                blue = top_view_blue_coordinates_queue.get()
                orange = top_view_orange_coordinates_queue.get()
                mybox= blue + orange

                #recieve the midpt 
                left_box, right_box, lines = Planning.pathplan(mybox, steering)

                #form obstacle list
                left_boxes = np.array(left_boxes)
                right_boxes = np.array(right_boxes)

                # Reading and appending coords with y > 310 - 60 in obstacles
                if len(left_boxes) <= 0 and len(right_boxes) <= 0:
                    continue
                else:
                    for i in range(left_boxes.shape[0]): 
                        #  and (left_boxes[i][0] < sx - 80 or left_boxes[i][0] > sx + 80)
                        if left_boxes[i][1] < 20: 
                            continue
                        else:   
                            ox.append(left_boxes[i][0])
                            oy.append(left_boxes[i][1])
                        
                    for i in range(right_boxes.shape[0]):
                        if right_boxes[i][1] < 20 : 
                            continue
                        else:   
                            ox.append(right_boxes[i][0])
                            oy.append(right_boxes[i][1])

                    #for all cones
                    # for i in range(left_boxes.shape[0]):     #????
                    #     ox.append(left_boxes[i][0])
                    #     oy.append(left_boxes[i][1])
                        
                    # for i in range(right_boxes.shape[0]):
                    #     ox.append(right_boxes[i][0])
                    #     oy.append(right_boxes[i][1])   
                print("ox :",ox)
                print("oy :",oy)       

                #sorts the array
                s_arr,sx,sy = Planning.sorted_arr(self, lines,ox,oy,sx,sy)
                #set the goal point
                gx,gy = Planning.set_goal_pt(self,s_arr,sx,sy)
                #updates the obstacle list
                ox,oy= Planning.obstacle_list_update(self,ox,oy,gx,gy)
                #Calls A Star algorithm
                a_star = AStarPlanner(ox, oy, grid_size, robot_radius)
                rx, ry = a_star.planning(sx, sy, gx, gy)



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
                    p1_parent.put(False)  # Act as a parent and send signal to child i.e detect
                    break
                else:
                    p1_parent.put(True)
                    
                
                # Check if at any point shared memory queue is empty.
                if top_view_frame_queue.empty() or top_view_blue_coordinates_queue.empty() or top_view_orange_coordinates_queue.empty():
                    queue_is_empty = True
                    
                frame_count+=1
            
            else:
                if not top_view_frame_queue.empty() and not top_view_blue_coordinates_queue.empty() and not top_view_orange_coordinates_queue.empty():
                    queue_is_empty = False
                    
                print("Debug planning: one of the queue is empty")
                
        cv2.destroyAllWindows()
        print("Path plan has stopped")        


    
