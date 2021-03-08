import cv2
import os,glob
from os import listdir,makedirs
import shutil, random
from PIL import Image
import sys
from os.path import isfile,join

a = input("Enter the case 1.bgr2rgb  2.rename  3.move files  4.resize  5.multipleclass          ");

def bgr2rgb():
	path = input("enter the source folder") # Source Folder
	for f in os.listdir(os.path.expanduser(path)):
		print(f)

	dstpath = input("enter the destination folder") # Destination Folder
	for f in os.listdir(os.path.expanduser(path)):
		print(f)

	try:
	    makedirs(dstpath)
	except:
		print ("Directory already exist, images will be written in same folder")
	# Folder won't used
	files = list(filter(lambda f: isfile(join(path,f)), listdir(path)))
	for image in files:
		try:
			img = cv2.imread(os.path.join(path,image))
			gray = cv2.cvtColor(img,cv2.COLOR_BGR2RGB)
			dstPath = join(dstpath,image)
			cv2.imwrite(dstPath,gray)
		except:
			print ("{} is not converted".format(image))
	for fil in glob.glob("*.jpg"):
		try:
			image = cv2.imread(fil) 
			gray_image = cv2.cvtColor(os.path.join(path,image), cv2.COLOR_BGR2RGB) # convert to rgb
			cv2.imwrite(os.path.join(dstpath,fil),gray_image)
		except:
			print('{} is not converted')
	
def rename():
	path = raw_input("enter the path of the folder")
	for filename in os.listdir(path):
    
		filename2 = path + "/"+filename
		print(filename2)
		os.rename(filename2,path + "/"+"rgb_"+filename)
# filename = "0001.txt"
# os.rename(filename,"rgb_"+filename)

def movefiles():
	dirpath = 'FINAL_IMAGES'
	dirpath1 = 'FINAL_LABELS'
	destDirectory = 'VALIDATION_DATASET'

	filenames = random.sample(os.listdir(dirpath), 720)
	for fname in filenames:
    		srcpath = os.path.join(dirpath, fname)
    		fname1 = fname[:-3]+"txt"
    		srcpath1 = os.path.join(dirpath1, fname1)
    		# print(srcpath,srcpath1)
    		shutil.move(srcpath, destDirectory)
    		shutil.move(srcpath1, destDirectory)
    		print("success")
    		
def resize():
	# print(os.listdir())
	path = input("enter the Path")
	for f in os.listdir(os.path.expanduser(path)):
		print(f)

	dirs = os.listdir( path )
	# print(dirs)
	def resize():
		for item in dirs:
        	# print(path+item)
        	# print(os.path.isfile(path+item))
			if os.path.isfile(path+item):
					im = Image.open(path+item)
					f, e = os.path.splitext(path+item)
			imResize = im.resize((800,608), Image.ANTIALIAS)
            	# imResize.save(f + ' resized.jpg', 'JPEG', quality=90)
			imResize.save(f +'.jpg', 'JPEG', quality=90)
			print("success")

	resize()

def multipleclass():
	path = input("enter the path")
	for f in os.listdir(os.path.expanduser(path)):
		print(f)

	# Read every file in directory
	for filename in os.listdir(path):
		filename = path + "/"+filename
		print(filename)
	f = open(filename)
	lines = f.readlines()
	f.close()
	f = open(filename, 'w')
	for line in lines:
        	if line[0] == "0": 
            		f.write("1"+ line[1:])
        	else:
            		f.write("0"+ line[1:])
	f.close()


if a == 1:
	bgr2rgb()
elif a == 2:
	rename()
elif a == 3:
	movefiles()
elif a == 4:
	resize()
else:
	multipleclass()

my_switch = {1: bgr2rgb, 2: rename, 3: movefiles, 4: resize, 5: multipleclass}	
