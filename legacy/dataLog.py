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

	log_file_name = day_folder + "/" + "test" + str(test_count)	
	
	f = open(log_file_name + ".json", "w+")

	return f, log_file_name

if __name__ == '__main__':
	f, log_file_name = give_file()

	DATA = [log_constants()]
	DATA[0]["log_constants"]["CAM_PATH"] = "video.mp4"
	log_data = []

	for i in range(10):
		frame_data = {
				"time_stamp":datetime.datetime.now().astimezone().isoformat(),
				"frame_count":i,
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