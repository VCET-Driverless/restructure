
# Library imports
from asyncio.proactor_events import constants
from multiprocessing import Process,Queue,Pipe

# System imports
from perception.perceive import Perceive
from perception.detect import Detect
from planning.path_plan import Planning
from control.control import Control
from system_manager.setup import Setup
from system_manager.fault_manager import Fault
from system_manager.loger.logging import Log


def main():

	# Declaring objects(instances variables) of each core class
	setup = Setup()
	setup.setup_driver()
	perception = Perceive()
	detect = Detect()
	path_plan = Planning()
	control = Control()
	log = Log()
	fault = Fault()
	
	# Raise error w.r.t hardware or software(eg: cv2 port not found, serial port not found, log file name already exists, etc)
	fault.check_fault(setup, perception, path_plan, control)
	
	# Declaring Shared memory variables for data transfer
	frame_queue = Queue()                        # used by perception and path_plan 
	darknet_image_queue = Queue()                # used by perception and detect
	detections_queue = Queue()                   # use by percetion and path_plan
	top_view_queue = Queue()                     # use by perception and path_plan
	top_view_blue_coordinates_queue = Queue()    # used by detect and pathplan
	top_view_orange_coordinates_queue = Queue()  # used by detect and pathplan
	log_queue = Queue()                          # used by path plan
	
	# Declaring Shared memory variables for terminating each process gracefully
	p1_parent, p1_child = Pipe()  # shared by perception and plan
	p2_parent, p2_child = Pipe()  # shared by plan and control
	
	# Declare all the processes
	process1 = Process(target=perception.video_capture,
							args=(
								setup, 
								frame_queue,
								darknet_image_queue,
								top_view_queue,
								p2_child,
							))
	
	process2 = Process(target=detect.detect,
							args=(
								setup, 
        						darknet_image_queue, 
								detections_queue,
        						top_view_blue_coordinates_queue,
								top_view_orange_coordinates_queue, 
        						p1_child,
              					p2_parent
							))
 
	process3 = Process(target=path_plan.path_plan_driver,
							args=(	
								setup, 
								frame_queue, 
        						top_view_queue, 
              					top_view_blue_coordinates_queue, 
                   				top_view_orange_coordinates_queue,
								log_queue,
        						p1_parent
							))
	
	try:						
		# Start the processes
		process1.start()
		process2.start()
		process3.start()

		# Wait for terminatation of each processes
		process1.join()
		process2.join()
		process3.join()
		
		# Log and save data
		log.log(log_queue)
  
		print("Execution Succesful")
		
	except:
		print("Process execution failed")
	
	
main()
