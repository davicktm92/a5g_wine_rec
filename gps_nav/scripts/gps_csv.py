#!/usr/bin/env python3

## Simple talker demo that listens to std_msgs/Strings published 
## to the 'chatter' topic

#Declarar librerías
import rospy # import the python library for ROS
import csv # import the csv library
import os
import time # import the time library para delays

#Declarar mensajes
from std_msgs.msg import String, Bool # import the String message type from the std_msgs package
from sensor_msgs.msg import NavSatFix

#Declarar variables globales
directory='gps_data' # Carpeta donde se guardan los archivos CSV

#Declarar clases
class gps_csv(object):
    def __init__(self):
        #Variables que voy a usar en el resto del programa
        self.name = ""
        self.lat = 0.0
        self.lon = 0.0
        self.cov = 0.0

        #Código que va a ejecutarse una sola vez
        self.name_file = input("Set the name of the file: ")
        self.crear_archivo_csv(self.name_file)

        #Declarar suscriptores y publicadores (cada nodo es un script)
        rospy.Subscriber('ublox/fix', NavSatFix, self.gps_callback)
        #rospy.Subscriber('robot/data_acquisition', Bool, self.acquisition_callback)
        rospy.Subscriber('robot/directory', String, self.name_callback)
        #rospy.Subscriber('robot/save_gps_data', Bool, self.save_gps_data_callback)

    #Definir las funciones de los suscriptores (en el caso de existir timers sería distinto)
    def gps_callback(self, data):
        self.lat = data.latitude
        self.lon = data.longitude
        self.cov = data.position_covariance[0]
    
    # def acquisition_callback(self, data):
    #     if data == True:
    #         if self.lat != 0.0 and self.lon != 0.0:
    #             self.guardar_datos()

    # def save_gps_data_callback(self, data):
    #     #aquí hay que hacer un input
    #     if data == True:
    #         if self.lat != 0.0 and self.lon != 0.0:
    #             self.guardar_datos()

    def name_callback(self, data):
        self.name = data.data
        if self.name != "":
            if self.lat != 0.0 and self.lon != 0.0:
                self.guardar_datos()
    

    def crear_archivo_csv(self, nombre_archivo):
        try:
            file_path = os.path.join(os.environ['HOME'], directory, nombre_archivo)
            with open(file_path, 'a', newline='') as file: # Crea un archivo CSV en modo escritura en el directorio especificado
                writer = csv.writer(file) # Crea un objeto writer
                writer.writerow(["ID", "Latitude", "Longitude", "Covariance"])  # Escribe la primera fila con encabezados

            print(f"File '{nombre_archivo}' successfully created")
        except IOError:
            print("Error creating the file")
    
    def guardar_datos(self):
        try:
            file_path = os.path.join(os.environ['HOME'], directory, self.name_file)
            with open(file_path, 'a', newline='') as file:
                writer = csv.writer(file)
                writer.writerow([self.name, self.lat, self.lon, self.cov])
            print("GPS data saved")
        except IOError:
            print("Error saving GPS data")

    def start(self):
        rospy.spin()


if __name__ == '__main__':
    rospy.init_node('gps_csv', anonymous=True)
    gps_csv_object = gps_csv()
    try:
        gps_csv_object.start()
    except rospy.ROSInterruptException:
            pass

if __name__ == '__main__':
    rospy.init_node('gps_csv', anonymous=True)
    gps_csv_object = gps_csv()
    try:
        gps_csv_object.start()
    except rospy.ROSInterruptException:
            pass
