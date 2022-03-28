
# Library imports
from multiprocessing import Process,Queue,SimpleQueue

# System imports
from perception.perceive import Perceive
from perception.detect import Detect
from planning.path_plan import Planning
from control.control import Control
from system_manager.setup import Setup
from system_manager.fault_manager.fault_manager import Fault
from system_manager.loger.logging import Log


def main():

	# Declaring objects(instances variables) of each core class
	setup = Setup()
	setup.setup_driver()
	detect = Detect()
	perception = Perceive()
	path_plan = Planning()
	control = Control()
	log = Log(setup)
	fault = Fault()
	
	# Raise error w.r.t hardware or software(eg: cv2 port not found, serial port not found, log file name already exists, etc)
	fault.check_fault(setup, perception, path_plan, control)
	
	# Declaring Shared memory variables for data transfer
	frame_queue = Queue()                        # used by perception and path_plan 
	top_view_queue = Queue()                     # use by perception and path_plan
	top_view_blue_coordinates_queue = Queue()    # used by detect and pathplan
	top_view_orange_coordinates_queue = Queue()  # used by detect and pathplan
	log_queue = Queue()                          # used by path plan
	
	# Declaring Shared memory variables for terminating each process gracefully
	p1_queue = SimpleQueue()  # shared by perception and plan
	
	# Declare all the processes
	process1 = Process(target=perception.video_capture,
							args=(
								setup, 
								frame_queue,
								top_view_queue,
        						top_view_blue_coordinates_queue, 
                   				top_view_orange_coordinates_queue,
								p1_queue,
							))
 
	process2 = Process(target=path_plan.path_plan_driver,
							args=(	
								setup, 
								frame_queue, 
        						top_view_queue, 
              					top_view_blue_coordinates_queue, 
                   				top_view_orange_coordinates_queue,
								log_queue,
        						p1_queue
							))
	
	try:						
		# Start the processes
		process1.start()
		process2.start()

		# Wait for terminatation of each processes
		process2.join()
		process1.join()
		
		# Log and save data
		log.log(log_queue)
  
		print("Execution Succesful")
		
	except Exception as err:
		print("Process execution failed: ", err)
	
	
main()
