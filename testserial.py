#!/usr/bin/python
from time import sleep
import serial

# Establish the connection on a specific port
ser = serial.Serial('/dev/ttyACM0', 115200)

x = 1
while True:
    print ser.readline() # Read the newest output
    x += 1
