#!/usr/bin/env python3

import rospy
from std_msgs.msg import String
from std_msgs.msg import Float32
import serial

rosRate=1

ser = serial.Serial(
    port='/dev/ttyUSB0',\
    baudrate=38400,\
    parity=serial.PARITY_NONE,\
    stopbits=serial.STOPBITS_ONE,\
    bytesize=serial.EIGHTBITS,\
        timeout=0)


class optrx:
    def __init__(self):
        self.ndvi_pub = rospy.Publisher('/robot/ndvi', String, queue_size=1)
        self.ndre_pub = rospy.Publisher('/robot/ndre', String, queue_size=1)
        self.optrx_pub = rospy.Publisher('/optrx_msg', String, queue_size=1)

        self.ndvi_msg=String()
        self.ndre_msg=String()
        self.optrx_msg=String()

        self.optrx_read=""
        self.ndvi_read=""
        self.ndre_read=""
        self.optrx_split=[]
        rospy.loginfo("Adquiriendo datos de OptRx")

    def read_data(self,event=None):
        
        self.optrx_read=str(ser.readline())[4:][:-5]
        self.optrx_split=self.optrx_read.split(",")

    def publish_msgs(self,event=None):
        if (len(self.optrx_split)>4):
            self.ndvi_msg.data=self.optrx_split[3].replace(" ","")
            self.ndre_msg.data=self.optrx_split[4].replace(" ","")
            self.optrx_msg.data=self.optrx_read
            self.ndvi_pub.publish(self.ndvi_msg)
            self.ndre_pub.publish(self.ndre_msg)
            self.optrx_pub.publish(self.optrx_msg)


    def start(self):
        rospy.Timer(rospy.Duration(1.0/rosRate),self.publish_msgs)
        rospy.Timer(rospy.Duration(1.0/rosRate),self.read_data)
        rospy.spin()
        


if __name__ == '__main__':
    rospy.init_node("optrx", anonymous=True) #create node
    optrx1=optrx()
    try:
        optrx1.start()
    except rospy.ROSInterruptException:
        pass
