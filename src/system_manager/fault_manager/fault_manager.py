import os
import serial
import time

class Fault:
    
    def check_fault(self, setup, perception, path_plan, control):
        Fault.check_logfile(self,setup)
        Fault.check_serial(setup)
        Fault.check_args(setup)
        Fault.perception_check(perception)
        Fault.hardware_test(setup)
        
    def check_logfile(self, setup):
        #It will check for existing logfile name and if so,raises an exception
        if os.path.isFile(setup.file_name):
            raise Exception("Log file already exists")

    def check_serial(setup):
        #checks if the serial port is open for use
        if not setup.serial.isOpen():
            try:
                setup.serial=serial.Serial('/dev/ttyACM1',setup.baud_rate)
                print("Connecting to : /dev/ttyACM1")
            except:
                raise Exception("Acess is denied to both serial port")

    def check_args(setup):
        #check for parser() error for weight_file ,config_file and data_file
        if not os.path.exists(setup.args.config_file):
            raise(ValueError("Invalid config path {}".format(os.path.abspath(setup.args.config_file))))
        if not os.path.exists(setup.args.weights):
            raise(ValueError("Invalid weight path {}".format(os.path.abspath(setup.args.weights))))
        if not os.path.exists(setup.args.data_file):
            raise(ValueError("Invalid data file path {}".format(os.path.abspath(setup.args.data_file))))
        if setup.str2int(setup.args.input) == str and not os.path.exists(setup.args.input):
            raise(ValueError("Invalid video path {}".format(os.path.abspath(setup.args.input))))

            
    def perception_check(perception):
        #checks if the cam is open or not
        if not perception.cap.isOpened():
           raise Exception("Cannot access the camera") 
       

       
    def hardware_test(setup):
        #Hardware testing: checks the steering control 
        setup.serial.write(str.encode(3))
        time.sleep(10)

        setup.serial.write(str.encode(4))
        time.sleep(10)
        
        setup.serial.write(str.encode(5))
        time.sleep(10)

        setup.serial.write(str.encode(4))
        time.sleep(10)

        