import os
import serial

class Fault:
    
    def check_fault(self, setup, perception, path_plan, control):
        
        Fault.check_setup(setup)
        Fault.perception_check(perception)
        Fault.hardware_test(setup)
        
    def check_setup(self, setup):
        
        if os.isFile(setup.file_name):
            raise Exception("Log file already exists")

        try:
            setup.serial=serial.Serial('/dev/ttyACM0',setup.baud_rate)
            print("Connecting to : /dev/ttyACM0")
        except:
            try:
                setup.serial=serial.Serial('/dev/ttyACM1',setup.baud_rate)
                print("Connecting to : /dev/ttyACM1")
            except:
                raise Exception("Acess is denied to both serial port")

        #check for parser() error for weight_file ,config_file and data_file
        if not os.path.exists(setup.args.config_file):
            raise(ValueError("Invalid config path {}".format(os.path.abspath(setup.args.config_file))))
        if not os.path.exists(setup.args.weights):
            raise(ValueError("Invalid weight path {}".format(os.path.abspath(setup.args.weights))))
        if not os.path.exists(setup.args.data_file):
            raise(ValueError("Invalid data file path {}".format(os.path.abspath(setup.args.data_file))))
        if Setup.str2int(setup.args.input) == str and not os.path.exists(setup.args.input):
            raise(ValueError("Invalid video path {}".format(os.path.abspath(setup.args.input))))

            
    def perception_check(self, perception):
        
        if not perception.cap.isOpened():
           raise Exception("Cannot access the camera") 
       

       
    def hardware_test(self, setup):
        
        setup.serial.write(str.encode(3))
    
        setup.serial.write(str.encode(4))
        
        setup.serial.write(str.encode(5))
        
        setup.serial.write(str.encode(4))
