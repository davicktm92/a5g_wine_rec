#!/usr/bin/env python3

from turtle import width
import rospy
from std_msgs.msg import String
from std_msgs.msg import Float32
import serial
import cv2
from sensor_msgs.msg import CompressedImage
import numpy as np
from cv_bridge import CvBridge

rosRate=10

vid = cv2.VideoCapture(0)

class front_camera:
    def __init__(self):
        self.image_pub = rospy.Publisher('/robot/front_camera/compressed', CompressedImage, queue_size=1)
        rospy.loginfo("Iniciando camara frontal")
        

    def read_data(self,event=None):
        
        ret, frame = vid.read()
        width=320
        height=240
        dim=(width,height)
        frame=cv2.resize(frame,dim, interpolation=cv2.INTER_AREA)
        #Para visualizar la imagen en caso de querer anadir algo
        #cv2.imshow('frame', frame)
        #cv2.waitKey(1)
        
        msg = CompressedImage()
        msg.header.stamp = rospy.Time.now()
        msg.format = "jpeg"
        msg.data = np.array(cv2.imencode('.jpg', frame)[1]).tobytes()
        # Publish new image
        self.image_pub.publish(msg)       

    def start(self):
        rospy.Timer(rospy.Duration(1.0/rosRate),self.read_data)
        rospy.spin()
        


if __name__ == '__main__':
    rospy.init_node("front_camera", anonymous=True) #create node
    front_camera1=front_camera()
    try:
        front_camera1.start()
    except rospy.ROSInterruptException:
        pass
