import os
import datetime
from constants import log_constants
from json import dump, load

test_cone_coordinates = [(1,2),(3,5)]
log_folder = "logs"

def give_file():
	test_count = 0
	today = datetime.datetime.now()

	if not os.path.isdir(log_folder):
		os.mkdir(log_folder)

	day_folder = log_folder + "/" + str(today.day) + "-" + str(today.month) + "-" + str(today.year)

	if not os.path.isdir(day_folder):
		os.mkdir(day_folder)

	while os.path.isfile(day_folder + "/" +"test" + str(test_count) + ".json"):
		test_count = test_count + 1

	f = open(day_folder + "/" + "test" + str(test_count) + ".json", "w+")

	return f 

test_script = True

if test_script:
	f = give_file()

	DATA = [log_constants()]
	log_data = []
	
	for _ in range(100):
		frame_data = {
				"time_stamp":datetime.datetime.now().astimezone().isoformat(),
				"frame_count":_,
				"steering":12,
                "left_box":test_cone_coordinates,
                "right_box":test_cone_coordinates,
                "lines":test_cone_coordinates
			}
		log_data.append(frame_data)
			
	DATA.append( {
			"log_data" : log_data
		} )

	dump(DATA, f, indent=4)

	f.close() 