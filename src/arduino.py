#! /usr/bin/env python

import rospy
import serial
from struct import unpack, pack
from geometry_msgs.msg import Twist

class Arduino():
    #Constructor
    def __init__(self):
        #Initialize node (Comperable to ros.init())
        rospy.init_node('arduino')
        #Initialize Imu topic Publisher(topic, message type, queue before discard)
        self.imu_publisher = rospy.Publisher(
            "/imu_data",
            Twist,
            queue_size = 30
        )
        #Initialize Message
        self.twist = Twist()        
        #Initialize array of 6 0's
        temp = [0]*6
        sums = [0]*6
        offsets = [0]*6;
        #Connect to the Arduino at the port with the given baud rate
        ser = serial.Serial('/dev/ttyUSB0', 9600)
        counter = 0
        #Loop until the shutdown signal is sent (Ctrl + c)
        while not rospy.is_shutdown():
            handshake = unpack('c', ser.read(1))[0]
            #Read the data from the wire, if the first byte is the start byte
            if handshake == 'S':
                counter += 1
                #Unpack each byte into the temp array
                temp[0] = unpack('<f', ser.read(4))[0]
                temp[1] = unpack('<f', ser.read(4))[0]
                temp[2] = unpack('<f', ser.read(4))[0]
                temp[3] = unpack('<f', ser.read(4))[0]
                temp[4] = unpack('<f', ser.read(4))[0]
                temp[5] = unpack('<f', ser.read(4))[0]
                #sums[0] += temp[0]
                #sums[1] += temp[1]
                #sums[2] += temp[2]
                #sums[3] += temp[3]
                #sums[4] += temp[4]
                #sums[5] += temp[5]
                #Set the Message properties from the temp array
                self.twist.linear.x = temp[0]
                self.twist.linear.y = temp[1]
                self.twist.linear.z = temp[2]
                self.twist.angular.x = temp[3]
                self.twist.angular.y = temp[4]
                self.twist.angular.z = temp[5]
                #Publish the Message
                self.imu_publisher.publish(self.twist)
                #If we finished too quickly, sleep
                #rospy.sleep(1)
                #Debug print of our data
                #print temp

if __name__ == '__main__':
    Arduino()
