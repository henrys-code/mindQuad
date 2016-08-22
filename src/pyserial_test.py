#! /usr/bin/env python

import rospy
import serial

ser = serial.Serial('/dev/ttyUSB0')
while (True):
    data = ser.readline().strip()
    print (data)

