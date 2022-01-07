import Setup            
#from system_manager.setup import Setup
import Perception
#from perception.perception import Perception
import serial
import math
import cv2

class check_fault:
    def intersect_error():                                  #pure_pursuit.py
        slope= math.inf

    def str2int_error(video_path):                          #Setup.py
        return video_path

    def pathplan_different_boundary_error():                #path_plan.py
        pass

    def angle_error():                                      #path_plan.py
        slope = 99999

    def pathplan_error():                                   #path_plan.py
        pass      #left box and right box

    def connect_arduino_error():                            #Setup.py
        try:
            print("failed...")
            Setup.serial=serial.Serial('/dev/ttyACM1',Setup.baud_rate)
            print("Connecting to : /dev/ttyACM1")
        except:
            print("failed... give port premission")   

    def different_boundary_main_error():
        Perception.cap.release()
        Setup.video.release()
        cv2.destroyAllWindows()

