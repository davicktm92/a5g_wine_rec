#!/usr/bin/env python3

#Declarar librerías
import rospy 
import csv
import os
import tf
import utm

#Declarar mensajes
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped

#Declarar variables globales
directory='gps_data' # Carpeta donde se guardan los archivos CSV

#Declarar clases
class gps_nav(object):
    def __init__(self):
        #Variables que voy a usar en el resto del programa
        self.coordinates = []
        self.name_file = ""
        self.file_path = ""
        
        self.goal=PoseStamped()

        self.broadcaster= tf.TransformBroadcaster()
        self.listener= tf.TransformListener()
           

        #Código que va a ejecutarse una sola vez
        self.name_file = input("Set the name of the file: ")
        self.file_path = os.path.join(os.environ['HOME'], directory, self.name_file)
        self.read_csv(self.file_path)

        #Declarar suscriptores y publicadores (cada nodo es un script)
        self.move_base_pub=rospy.Publisher('move_base_simple/goal', PoseStamped, queue_size=1)
        rospy.Subscriber('destination', String, self.dest_callback)

    #Definir las funciones de los suscriptores (en el caso de existir timers sería distinto)
    def dest_callback(self, data):
        try:
            (trans_leader, rot_leader) = self.listener.lookupTransform('robot_map', data.data, rospy.Time(0))
            self.goal.header.frame_id= 'robot_map'

            self.goal.pose.position.x=trans_leader[0]
            self.goal.pose.position.y=trans_leader[1]
            self.goal.pose.position.z=trans_leader[2]

            self.goal.pose.orientation.x=rot_leader[0]
            self.goal.pose.orientation.y=rot_leader[1]
            self.goal.pose.orientation.z=rot_leader[2]
            self.goal.pose.orientation.w=rot_leader[3]

            self.move_base_pub.publish(self.goal)
        
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            pass


    #Definir funciones propias
    def read_csv(self, nombre_archivo):
        with open(nombre_archivo, 'r') as file:
            reader = csv.reader(file)
            for i, row in enumerate(reader):
                if i == 0:  # Se salta la cabecera
                    continue
                name= row[0]
                lat = float(row[1])
                lon = float(row[2])
                utm_coord = utm.from_latlon(lat, lon)
                self.coordinates.append((name, utm_coord[0], utm_coord[1]))

    def tf_pub(self, event=None):
        for i, coord in enumerate(self.coordinates):
            x_init = self.coordinates[0][1]
            y_init = self.coordinates[0][2]
            x= coord[1]-x_init
            y= coord[2]-y_init
            self.broadcaster.sendTransform((-x,-y,0),(0,0,0,1),rospy.Time.now(),coord[0],"robot_map")
    
    def start(self):
        rospy.Timer(rospy.Duration(0.1), self.tf_pub)
        rospy.spin()

#Iniciar el nodo
if __name__ == '__main__':
    rospy.init_node('gps_nav', anonymous=True)
    gps_nav_object = gps_nav()
    try:
        gps_nav_object.start()
    except rospy.ROSInterruptException:
        pass
