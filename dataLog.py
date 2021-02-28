import os
import datetime
import constants as ch

def give_file():
	test_count = 0
	today = datetime.datetime.now()
	folder = str(today.day) + "-" + str(today.month) + "-" + str(today.year)
	L = [(1,2),(3,5)]

	if not os.path.isdir(folder):
		os.mkdir(folder)

	while os.path.isfile(folder+"/" +"test"+str(test_count)+ ".txt"):
		test_count = test_count+1

	f = open(folder+"/" +"test"+str(test_count)+ ".txt", "w+")

	#img_dim
	#BAUD_RATE
	#pt_in
	#LIMIT_CONE
	#mid_c
	#car_coor
	DATA = {}
	'''
	DATA["TOP_VIEW_IMAGE_DIMESNION"] = ch.TOP_VIEW_IMAGE_DIMESNION
	DATA["BAUD_RATE"] = ch.BAUD_RATE 
	DATA["CAR_BELOW_Y"] = ch.CAR_BELOW_Y 
	DATA["pt_in"] = ch.pt_in 
	DATA["LIMIT_CONE"] = ch.LIMIT_CONE 
	DATA["mid_c"] = ch.mid_c 
	DATA["car_coor"] = ch.car_coor
	'''
	#DATA = [ch.img_dim, ch.BAUD_RATE, ch.pt_in, ch.LIMIT_CONE, ch.mid_c, ch.car_coor]
	f.write(str(DATA)+"\n")
	#for _ in range(100):
	#	f.write(str(datetime.datetime.now()) + " " + str(L) + "\n")
	f.close()
	f = open(folder+"/" +"test"+str(test_count)+ ".txt", "a+")
	return f