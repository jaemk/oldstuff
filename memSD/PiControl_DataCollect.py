# Continuously pull and print data from main control (mega - 'DriveControl_PulseControl.ino')
import thread, os, time, serial

def input_thread(L):
    stow = raw_input()
    L.append(stow)
    

ard_loc = open('Arduino_USB_Location','r').read()
ser = serial.Serial(ard_loc,115200)
ser.write('1;')  # commence
raw_input('Press enter to receive and print')
L = []
thread.start_new_thread(input_thread, (L,))

while True:
	val = ser.readline().strip()
	print val + ', Press "Enter" to reset'
	if L:
		print L[0]
		del L[0]
		break



