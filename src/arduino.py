#! /usr/bin/env python

import rospy
import serial
from struct import unpack, pack
from geometry_msgs.msg import Twist

class Arduino():
    def __init__(self):
        rospy.init_node('Arduino')
        self.imu_publisher = rospy.Publisher(
            "/imu_data",
            Twist,
            queue_size = 30
        )
        self.twist = Twist()
        temp = [0]*6
        ser = serial.Serial('/dev/ttyUSB0', 9600)
        while not rospy.is_shutdown():

            temp[0] = unpack('<B', ser.read())[0]
            temp[1] = unpack('<B', ser.read())[0]
            temp[2] = unpack('<B', ser.read())[0]
            temp[3] = unpack('<B', ser.read())[0]
            temp[4] = unpack('<B', ser.read())[0]
            temp[5] = unpack('<B', ser.read())[0]

            self.twist.linear.x = temp[0]
            self.twist.linear.y = temp[1]
            self.twist.linear.z = temp[2]
            self.twist.angular.x = temp[3]
            self.twist.angular.y = temp[4]
            self.twist.angular.z = temp[5]
            
            self.imu_publisher.publish(self.twist)
            rospy.sleep(1)

if __name__ == '__main__':
    Arduino()
