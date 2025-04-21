#!/usr/bin/env python3

#Declarar librerías
import rospy # import the python library for ROS
import csv # import the csv library
import os
import tf

#Declarar mensajes
from std_msgs.msg import String, Bool # import the String message type from the std_msgs package
from sensor_msgs.msg import NavSatFix

#Declarar variables globales
directory='gps_data' # Carpeta donde se guardan los archivos CSV

#Declarar clases
class map_csv(object):
    def __init__(self):
        #Variables que voy a usar en el resto del programa
        self.name = ""
        self.trans_leader = []
        self.rot_leader = []

        self.listener= tf.TransformListener()

        #Código que va a ejecutarse una sola vez
        self.name_file = input("Set the name of the file: ")
        self.crear_archivo_csv(self.name_file)

        #Declarar suscriptores y publicadores (cada nodo es un script)
        rospy.Subscriber('directory', String, self.name_callback)

    #Definir las funciones de los suscriptores (en el caso de existir timers sería distinto)


    def name_callback(self, data):
        try:
            (self.trans_leader, self.rot_leader) = self.listener.lookupTransform('robot_map', 'robot_base_footprint', rospy.Time(0))
            self.name = data.data
            self.guardar_datos()

        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            pass
              
    
    #Definir funciones propias
    def crear_archivo_csv(self, nombre_archivo):
        try:
            file_path = os.path.join(os.environ['HOME'], directory, nombre_archivo)
            with open(file_path, 'a', newline='') as file: # Crea un archivo CSV en modo escritura en el directorio especificado
                writer = csv.writer(file) # Crea un objeto writer
                writer.writerow(["ID", "Pos_x", "Pos_y", "Pos_z","Rot_x", "Rot_y", "Rot_z", "Rot_w"])
        except IOError:
            print("Error creating the file")
    
    def guardar_datos(self):
        try:
            file_path = os.path.join(os.environ['HOME'], directory, self.name_file)
            with open(file_path, 'a', newline='') as file:
                writer = csv.writer(file)
                writer.writerow([self.name, self.trans_leader[0], self.trans_leader[1], self.trans_leader[2], self.rot_leader[0], self.rot_leader[1], self.rot_leader[2], self.rot_leader[3]])
            print("Map data saved")
        except IOError:
            print("Error saving map data")

    def start(self):
        rospy.spin()

#Inicializar el nodo
if __name__ == '__main__':
    rospy.init_node('map_csv', anonymous=True)
    map_csv_object = map_csv()
    try:
        map_csv_object.start()
    except rospy.ROSInterruptException:
        pass
