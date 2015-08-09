#!/usr/bin/env python

# communicates via i2c with arduino running 'laser.py' sketch - programmed from raspberry pi using ino-tool
# captures user input with curses module to remotely control dual servo gimballed laser pointer to facilitate catastic interactions

import curses
import time

import smbus
bus = smbus.SMBus(1)
address = 0x69
count = 0

speed = 2

def request():
	result = bus.read_byte(add)
	leftSideByte = bus.read_byte(add)
	result |= leftSideByte << 8
	return result

def send(servo,val):
    bus.write_byte(address,servo)
    bus.write_byte(address,val)
	# valRight = val & 0xFF
	# valLeft = val >> 8
	# bus.write_byte(add,valRight)
	# bus.write_byte(add,valLeft)


screen = curses.initscr()
curses.noecho()
curses.curs_set(0)
screen.keypad(1)
screen.nodelay(1)

screen.addstr("Control Laser Point\n\n")
time.sleep(.5)

up = False
down = False
left = False
right = False

x_speed = 0
y_speed = 0

width = 180
height = 180
# Current position
x_coord = width/2
y_coord = height/2

check = False
check_light = False
while True:
	event = screen.getch()
    	if event == ord("q"):
		break 
	elif event == ord("f"):
		screen.clear()
		screen.addstr("Key press detected: F,    ")
		check_light = True
    	elif event == curses.KEY_UP:
		screen.clear()
		screen.addstr("Key press detected: Up,    ")
		y_speed = -speed
		check = True
	elif event == curses.KEY_DOWN:
		screen.clear()
		screen.addstr("Key press detected: Down,  ")
		y_speed = speed
		check = True
	elif event == curses.KEY_LEFT:
		screen.clear()
		screen.addstr("Key press detected: Left,  ")
		x_speed = speed
		check = True
	elif event == curses.KEY_RIGHT:
		screen.clear()
		screen.addstr("Key press detected: Right, ")
		x_speed = -speed
		check = True

	if check == True:
		x_coord = x_coord + x_speed
		y_coord = y_coord + y_speed
		x_speed = 0
		y_speed = 0
		if x_coord > width:
		    x_coord = width
		elif x_coord < 0:
		    x_coord = 0

		if y_coord > height:
		    y_coord = height
		elif y_coord < 0:
		    y_coord = 0

		screen.addstr("X coord: " + str(x_coord))
		screen.addstr(", Y coord: " + str(y_coord))
		check = False

		try:
			send(1, int(x_coord))
			send(2, int(y_coord))
		except:
				print 'I2C Communication Error'

	if check_light == True:
		check_light = False
		try:
			send(3,int(100))
		except:
			print 'I2C Communication Error'

curses.endwin()

