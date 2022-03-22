import os
import serial
import time

class Fault:
    
    def check_fault(self, setup, perception, path_plan, control):
        self.check_logfile(setup)
        self.check_serial(setup)
        self.check_args(setup)
        self.perception_check(setup)
        self.hardware_test(setup)
        
    def check_logfile(self, setup):
        #It will check for existing logfile name and if so,raises an exception
        if os.path.isfile(setup.log_file_name):
            raise Exception("Log file already exists")

    def check_serial(self, setup):
        #checks if the serial port is open for use
        if setup.ARDUINO_CONNECTED is True:
            if not setup.serial.isOpen():
                try:
                    setup.serial=serial.Serial('/dev/ttyACM1',setup.BAUD_RATE)
                    print("Connecting to : /dev/ttyACM1")
                except:
                    raise Exception("Acess is denied to both serial port")

    def check_args(self, setup):
        #check for parser() error for weight_file ,config_file and data_file
        if not os.path.exists(setup.args.config_file):
            raise(ValueError("Invalid config path {}".format(os.path.abspath(setup.args.config_file))))
        if not os.path.exists(setup.args.weights):
            raise(ValueError("Invalid weight path {}".format(os.path.abspath(setup.args.weights))))
        if not os.path.exists(setup.args.data_file):
            raise(ValueError("Invalid data file path {}".format(os.path.abspath(setup.args.data_file))))
        if setup.str2int(setup.args.input) == str and not os.path.exists(setup.args.input):
            raise(ValueError("Invalid video path {}".format(os.path.abspath(setup.args.input))))

            
    def perception_check(self, perception):
        #checks if the cam is open or not
        if not perception.cap.isOpened():
           raise Exception("Cannot access the camera") 
       

       
    def hardware_test(self, setup):
        #Hardware testing: checks the steering control 
        if setup.ARDUINO_CONNECTED is True:
            setup.serial.write(str.encode(3))
            time.sleep(10)

            setup.serial.write(str.encode(4))
            time.sleep(10)
            
            setup.serial.write(str.encode(5))
            time.sleep(10)

            setup.serial.write(str.encode(4))
            time.sleep(10)

        