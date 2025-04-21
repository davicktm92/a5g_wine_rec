#!/usr/bin/env python3

import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
from geometry_msgs.msg import Pose
from std_msgs.msg import Int8, Bool, String
import numpy as np
import ftplib
import os
from datetime import datetime

# Configuración de conexión FTP
ftp_host = "172.18.240.48"
ftp_user = "innovacion"
ftp_pass = "Innotec1"

ftp_remote_dir = "/ftp_test/"  # Directorio base en el servidor FTP

# Ruta local de los archivos a subir
#local_dir= r"/home/robcib/test_spectral/11_front"
dir_base=r"/home/robcib/"

class ftp_upload:

    def __init__(self):
        self.ftp=ftplib.FTP(ftp_host)
        self.ftp.login(ftp_user,ftp_pass)

        print(f"Conectado a {ftp_host}")

        unique_dir = datetime.now().strftime("%Y%m%d_%H%M%S")
        self.remote_main_dir = os.path.join(ftp_remote_dir, unique_dir).replace("\\", "/")

        try:
            self.ftp.mkd(self.remote_main_dir)
            print(f"Directorio principal creado: {self.remote_main_dir}")
        except ftplib.error_perm as e:
            print(f"Error al crear el directorio principal {self.remote_main_dir}: {e}")
            return
        
        #creacion de directorios remotos locales pcd y spectral
        remote_pcd_dir = os.path.join(self.remote_main_dir, "pcd").replace("\\", "/")
        remote_spectral_dir = os.path.join(self.remote_main_dir, "spectral").replace("\\", "/")
        self.ftp.mkd(remote_pcd_dir)
        self.ftp.mkd(remote_spectral_dir)
        #Variables ROS
        rospy.Subscriber('/robot/directory',String,self.directoryCb)
        rospy.Subscriber('/robot/3dscan_clear',Int8,self.uploadCb)

        self.local_dir_msg=None

    def directoryCb(self,msg):
        rospy.loginfo("Directorio recibido")
        self.local_dir_msg=msg.data

        try:
            remote_local_dir=self.remote_main_dir+"/spectral/"+self.local_dir_msg
            self.ftp.mkd(remote_local_dir)

            remote_local_dir=self.remote_main_dir+"/pcd/"+self.local_dir_msg
            self.ftp.mkd(remote_local_dir)

            print(f"Directorio local creado: {remote_local_dir}")
        except ftplib.error_perm as e:
            print(f"Error al crear el directorio local {remote_local_dir}: {e}")
            return

    def uploadCb(self,msg):
        rospy.loginfo("Iniciando subida")
        if msg.data==1 and self.local_dir_msg is not None:          
            #enviar archivos de pcd
            local_dir=dir_base+"pcd/"+"vid_"+self.local_dir_msg+".pcd"
            remote_dir=self.remote_main_dir+"pcd/"+self.local_dir_msg
            self.upload_pcd(self.ftp, local_dir, remote_dir)

            #enviar archivos de spectral
            local_dir=dir_base+"spectral/"+self.local_dir_msg
            remote_dir=self.remote_main_dir+"/spectral/"+self.local_dir_msg
            self.upload_files(self.ftp, local_dir, remote_dir)

            rospy.loginfo("Archivos subidos")
            msg.data=0

    def upload_files(self,ftp, path, remote_base_dir):
        for root, dirs, files in os.walk(path):
            for dirname in dirs:
                remote_path = os.path.join(remote_base_dir, os.path.relpath(os.path.join(root, dirname), path)).replace("\\", "/")
                try:
                    ftp.mkd(remote_path)
                    print(f"Directorio creado: {remote_path}")
                except ftplib.error_perm as e:
                    if not e.args[0].startswith('550'):
                        print(f"Error al crear el directorio {remote_path}: {e}")

            for filename in files:
                remote_path = os.path.join(remote_base_dir, os.path.relpath(os.path.join(root, filename), path)).replace("\\", "/")
                try:
                    with open(os.path.join(root, filename), 'rb') as file:
                        ftp.storbinary(f'STOR {remote_path}', file)
                        print(f"Archivo subido: {filename}")
                except Exception as e:
                    print(f"Error al subir {filename}: {e}")

    def upload_pcd(self,ftp, filename, remote_base_dir):
        try:
            file=open(filename, 'rb')
            #remote_path = os.path.join(remote_base_dir, filename).replace("\\", "/")
            remote_path=os.path.join(self.remote_main_dir, os.path.join("pcd/",self.local_dir_msg)+"/vid_"+self.local_dir_msg+".pcd").replace("\\", "/")
            ftp.storbinary(f'STOR {remote_path}', file)
            print(f"Archivo subido: {filename}")
            
        except Exception as e:
            print(f"Error al subir {filename}: {e}")

    def start(self):
        rospy.spin()

if __name__ == "__main__":
    rospy.init_node('ftp_upload', anonymous=True) #make node
    ftp_upload=ftp_upload()
    try:
        ftp_upload.start()
    except rospy.ROSInterruptException:
        pass

