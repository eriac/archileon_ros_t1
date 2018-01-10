#!/usr/bin/env python
import rospy
from std_msgs.msg import String
import serial
import time

ser = serial.Serial()
ser.port = '/dev/ttyUSB0'
ser.baudrate = 115200
ser.open()
time.sleep(1.5)

def callback(data):
	rospy.loginfo("ArduinoOut %s",data.data)
	ser.write(data.data)
    
def listener():
	rospy.init_node('arduino_serial', anonymous=True)
	rospy.Subscriber("arduino_out", String, callback, queue_size = 100)
	
	r = rospy.Rate(20) # 20hz
	while not rospy.is_shutdown():
		r.sleep()
		#rospy.spin()
        ser.close()

if __name__ == '__main__':
    listener()
