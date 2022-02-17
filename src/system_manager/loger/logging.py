
# Library imports
from json import dump
import sys
sys.path.append('../')

# System imports
from system_manager.constants import Constants

class Log(Constants):
    
    def __init__(self, setup):
        
        super().__init__()
        self.log_data=[]
        self.log_file_name = setup.log_file_name
        self.DATA = [self.log_constants()]
        
    def log_constants(self):
        
        '''
        :returns: All the necessary static constants required to log
        '''
        
        return {
            "log_constants" : {
                "CAM_PATH" : self.CAM_PATH,  
                "BAUD_RATE" : self.BAUD_RATE, 
                "CAR_BELOW_Y" : self.CAR_BELOW_Y,
                "LIMIT_CONE" : self.LIMIT_CONE,
                "MIDPOINT_ONE_BOUNDARY" : self.MIDPOINT_ONE_BOUNDARY,
                "P" : self.P,
                "MAX_CONELESS_FRAMES" : self.MAX_CONELESS_FRAMES,
                "ARDUINO_CONNECTED" : self.ARDUINO_CONNECTED,
                "RATE" : self.RATE, 
                "TESTER" : self.TESTER,
                "WHICH_SYSTEM" : self.WHICH_SYSTEM,
                "TOP_VIEW_IMAGE_DIMESNION" : self.TOP_VIEW_IMAGE_DIMESNION,
                "FRONT_VIEW_POINTS" : self.FRONT_VIEW_POINTS,
                "Lookahead_Distance" : self.Lfc
            }
        }
        
    def save_file(self):
        
        """
        Saves the JSON file with all the data
        :returns: Saves the data in a JSON file
        """
        
        file = open(self.log_file_name + ".json", "w+")
        self.DATA.append( {
                "log_data" : self.log_data
            } )
        dump(self.DATA, file, indent=4)
        file.close()
        
    def log(self, log_queue):
        
        """
        Saves the JSON file with all the data
        " log_queue: frame by frame map of all the data to be logged
        :returns: Saves the data in a JSON file
        """
        
        while not log_queue.empty():
            
            self.log_data.append(log_queue.get())
        
        self.save_file()
        
        print("Data logged succesfully")
        
    # def log(self, percieve_map, detection_map, path_map):
        
    #     """
    #     Logs the entire data obtained from all the processes
    #     :perception_map: data from perception process
    #     :detection_map: data from detection process
    #     :path_map: data from path plan and control process
    #     :returns: Saves the data in a JSON file
    #     """
        
    #     frame=0
    #     while frame<len(percieve_map) and frame<len(detection_map) and frame<len(path_map):
    
    #         logs = {}
            
    #         percep_map = percieve_map.get(frame)
    #         detect_map = detection_map.get(frame)
    #         path_plan_map = path_map.get(frame)
            
    #         logs.update(percep_map)
    #         logs.update(detect_map)
    #         logs.update(path_plan_map)
            
    #         self.log_data.append(logs)
            
    #         frame+=1
            
        
    #     # All maps must have equal number of frames
    #     if(len(percieve_map) != len(path_map) or len(percieve_map) != len(detection_map) or len(path_map) != len(detection_map)):
    #         print("Log data is faulty.")
            
            
    #     # Open file, write, save it
    #     self.save_file()
        
    #     print("Data logged succesfully")
        
        
            
        
