# hardwareTesting.py
'''
This file communicates directly to actuating hardware for testing purpose
'''

import serial

# sampeling speed
from chcone import BAUD_RATE

# initializing which serial port to connect
try:
	s=serial.Serial('/dev/ttyACM0',BAUD_RATE)
	print("Connected to : /dev/ttyACM0")
except:
	s=serial.Serial('/dev/ttyACM1',BAUD_RATE)
	print("Connected to : /dev/ttyACM1")

# reads char from user and sends to hardware control (Arduino)
# each character is mapped to an action (See Arduino's code)
while True:
	ip = input('Enter char 0 to 9: ')
	s.write(str.encode(ip))
