# code accepts rpm values below 18,000rpm. Sends value as set point to arduino motorcontroller (running 'Drive_ControlledFromPi.ino').

import thread, os, time, serial

def input_thread(L):
    stow = raw_input()
    L.append(stow)
    
def do_print(sending):
    L = []
    thread.start_new_thread(input_thread, (L,))
    while 1:
		start = time.time()
		ser.write(sending)
		val = ser.readline().strip()
		time.sleep(.01)
		stop = time.time()
		elap = stop - start
		#~ print 'Send: ' + sending + '| Confirm: ' + val + '| PiTime: ' + str(round(elap,6)) + ', "Enter" to reset'
		print val + elap + ', Press "Enter" to reset'
		if L:
			print L[0]
			del L[0]
			#count = 2
			break

ard_loc = open('Arduino_USB_Location','r').read()
ser = serial.Serial(ard_loc,115200)
ser.write('1;')  # commence
derp = '0'
#~ count = 0

while True:
	#sending = raw_input('Enter new RPM value to send: ').strip()
	sending = '1'
	raw_input('Press enter')
	#~ if count < 1:
	try:
		if int(sending) > 18000:
			print 'PWM value out of range...'
		else:
			sending = sending + ';'
			do_print(sending)
	except:
		print 'Please enter an integer'
	
