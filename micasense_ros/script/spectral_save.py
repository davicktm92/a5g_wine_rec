#!/usr/bin/env python3

import sys
import cv2
import os
import io
import numpy as np
import rospy
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import Int8, String


class spectral_save:
  def __init__(self):    
    self.phase=0 #0=nothing, 1=v_scan, 2=h_scan, 3=spectral_scan
    self.exec=0 #0=not executing, 1=executing
    self.c1=0
    self.c2=0
    self.c3=0
    self.c4=0
    self.c5=0
    self.c6=0
    #self.zone=input("Numero de zona a explorar: \n")
    #path = directory+"/"+str(self.zone)
    #isExist = os.path.exists(path)
    #if not isExist:
    #  print("Directorio no existe")
    #  os.makedirs(path)
    #  print("Directorio creado")
    #else:
    #  print("Directorio existe")
    
    #os.chdir(path)

  def channel1Cb(self,msg):
    if self.exec==1:
      self.c1+=1 
      np_arr = np.fromstring(msg.data, np.uint8)
      im_np = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
      filename="channel1_"+str(self.c1)+".jpg"
      cv2.imwrite(filename, im_np)
      rospy.loginfo("Image_saved")
    
  def channel2Cb(self,msg):
    if self.exec==1:
      self.c2+=1 
      np_arr = np.fromstring(msg.data, np.uint8)
      im_np = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
      filename="channel2_"+str(self.c2)+".jpg"
      cv2.imwrite(filename, im_np)

  def channel3Cb(self,msg):
    if self.exec==1:
      self.c3+=1 
      np_arr = np.fromstring(msg.data, np.uint8)
      im_np = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
      filename="channel3_"+str(self.c3)+".jpg"
      cv2.imwrite(filename, im_np)

  def channel4Cb(self,msg):
    if self.exec==1:
      self.c4+=1 
      np_arr = np.fromstring(msg.data, np.uint8)
      im_np = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
      filename="channel4_"+str(self.c4)+".jpg"
      cv2.imwrite(filename, im_np)

  def channel5Cb(self,msg):
    if self.exec==1:
      self.c5+=1 
      np_arr = np.fromstring(msg.data, np.uint8)
      im_np = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
      filename="channel5_"+str(self.c5)+".jpg"
      cv2.imwrite(filename, im_np)

  def channel6Cb(self,msg):
    if self.exec==1:
      self.c6+=1 
      np_arr = np.fromstring(msg.data, np.uint8)
      im_np = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
      filename="channel6_"+str(self.c5)+".jpg"
      cv2.imwrite(filename, im_np)
      rospy.loginfo("Imagenes espectrales guardadas")

  def spectralCb(self,msg):
    if msg.data!=0:
      self.exec=1
      rospy.loginfo("Start spectral acquisition")
    else:
      self.exec=0
      rospy.loginfo("Stop spectral acquisition")

  def directoryCb(self,msg):
    directory = r"/home/robcib/spectral"
    path = directory+"/"+str(msg.data)
    isExist = os.path.exists(path)
    if not isExist:
      print("Directorio no existe")
      os.makedirs(path)
      print("Directorio creado")
    else:
      print("Directorio existe")

    os.chdir(path)

    #encerado de los contadores
    self.c1=0
    self.c2=0
    self.c3=0
    self.c4=0
    self.c5=0
    self.c6=0

  def start(self):
    rospy.Subscriber('/robot/save_spectral',Int8,self.spectralCb)
    rospy.Subscriber('/robot/directory',String, self.directoryCb)
    rospy.Subscriber('/micasense/channel1/compressed',CompressedImage,self.channel1Cb)
    rospy.Subscriber('/micasense/channel2/compressed',CompressedImage,self.channel2Cb)
    rospy.Subscriber('/micasense/channel3/compressed',CompressedImage,self.channel3Cb)
    rospy.Subscriber('/micasense/channel4/compressed',CompressedImage,self.channel4Cb)
    rospy.Subscriber('/micasense/channel5/compressed',CompressedImage,self.channel5Cb)
    rospy.Subscriber('/micasense/channel6/compressed',CompressedImage,self.channel6Cb)

if __name__ == '__main__':
  rospy.init_node('spectral_save', anonymous=True) #make node
  spectral1=spectral_save()
  try:
    spectral1.start()
    rospy.spin()
  except rospy.ROSInterruptException:
    pass


