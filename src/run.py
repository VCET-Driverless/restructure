
# Library imports
from multiprocessing import Process,Queue,Pipe

# System imports
from perception.perception import Perception
from planning.path_plan import Planning
from control.control import Control
from system_manager.setup import Setup

# fault and log will be called in each class individually at required places.
# from system_manager.fault_manager.detect import check_fault
# from system_manager.loger.logging import Log


def main():

	# Declaring objects(instances variables) of each core class
	setup = Setup()
	perception = Perception()
	path_plan = Planning()
	control = Control()
	
	# Declaring Shared memory variables for data transfer
	frame_queue = Queue()           # used by perception
	detections_queue = Queue()      # use by percetion and path_plan
	top_view_queue = Queue()        # use by perception and path_plan
	path_queue = Queue()            # used by path_plan and control
	
	# Declaring Shared memory variables for terminating each process gracefully
	p1 = Pipe()  # shared by perception and plan
	p2 = Pipe()  # shared by plan and control
	
	# Declare all the processes
	process1 = Process(target=perception.perception_driver,
							args=(
								setup, 
								frame_queue,
								detections_queue
								top_view_queue
								p1
							))
							
	process2 = Process(target=path_plan.path_plan_driver,
							args=(
								setup,
								top_view_queue,
								path_queue
								p1,
								p2	
							))
							
	process3 = Process(target=control.control_driver,
							args=(
								setup,
								path_queue,
								p2
							))
							
	# Start the processes
	process1.start()
	process2.start()
	process3.start()
	
	# Wait for terminatation of each processes
	process1.join()
	process2.join()
	process3.join()
	
	print("Execution Succesful")
	
main()
