#!/usr/bin/env python3

#Declarar librerías
import rospy 
import csv
import os
import tf
import time

import utm
import math
import numpy as np
from scipy.spatial.transform import Rotation as R

#Declarar mensajes
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal, MoveBaseActionResult
from std_msgs.msg import String, Bool
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

        self.next_goal=""
        self.last_goal="init"
        self.get_data=False
        
        self.goal=PoseStamped()

        self.broadcaster= tf.TransformBroadcaster()
        self.listener= tf.TransformListener()
           

        #Código que va a ejecutarse una sola vez
        #self.name_file = input("Set the name of the file: ")
        self.name_file = "yepes1.csv"
        self.file_path = os.path.join(os.environ['HOME'], directory, self.name_file)

        #Declarar suscriptores y publicadores (cada nodo es un script)
        self.move_base_pub=rospy.Publisher('move_base_simple/goal', PoseStamped, queue_size=1)
        self.data_acq_pub=rospy.Publisher('data_acquisition', Bool, queue_size=1)
        rospy.Subscriber('destination', String, self.dest_callback)
        rospy.Subscriber('move_base/result', MoveBaseActionResult, self.result_callback)

    #Definir las funciones de los suscriptores
    def dest_callback(self, data):
        self.next_goal=data.data
        #print("Next goal: "+self.next_goal)
        #print("Last goal: "+self.last_goal)
        if ("front" in self.next_goal and "front" in self.last_goal) or ("back" in self.next_goal and "back" in self.last_goal):
            self.pub_move_base(self.next_goal)
        elif "init" in self.last_goal: # Si sale desde el punto de inicio
            if "front" in self.next_goal:
                self.pub_move_base("ref_front")
                self.pub_move_base(self.next_goal)
            elif "back" in self.next_goal:
                self.pub_move_base("ref_back")
                self.pub_move_base(self.next_goal)
            elif "init" in self.next_goal:
                self.pub_move_base(self.next_goal)
        elif "init" in self.next_goal: # Si se dirige al punto de inicio
            if "front" in self.last_goal:
                self.pub_move_base("ref_front")
                self.pub_move_base(self.next_goal)
            elif "back" in self.last_goal:
                self.pub_move_base("ref_back")
                self.pub_move_base(self.next_goal)
        else:
            print("Cambio de hilera")
            self.cambio_hilera(self.next_goal)
        self.last_goal=self.next_goal

    def result_callback(self, data):
        if data.status.text == "Goal reached.":
            print("Goal reached")
            self.get_data=True
        

    #Definir funciones propias
    def read_csv_gps(self, nombre_archivo):
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
    
    def read_csv_map(self, nombre_archivo):
        with open(nombre_archivo, 'r') as file:
            reader = csv.reader(file)
            for i, row in enumerate(reader):
                if i == 0:  # Se salta la cabecera
                    continue
                self.coordinates.append((row[0], float(row[1]), float(row[2]), float(row[3]), float(row[4]), float(row[5]), float(row[6]), float(row[7])))
    
    def cambio_hilera(self, data):
        if "front" in data: # Estamos en la hilera trasera y el próximo destino es la hilera frontal
            self.pub_move_base("ref_back")
            self.pub_move_base("ref_front")
            self.pub_move_base(self.next_goal)
        elif "back" in data: # Estamos en la hilera frontal y el próximo destino es la hilera trasera
            self.pub_move_base("ref_front1")
            self.pub_move_base("ref_back1")
            self.pub_move_base(self.next_goal)
        else:
            print("Invalid destination")

    def pub_move_base(self, data):
        try:
            print("Publishing goal "+data)        
            (trans_leader, rot_leader) = self.listener.lookupTransform('robot_map', data, rospy.Time(0))
            self.goal.header.frame_id= 'robot_map'

            self.goal.pose.position.x=trans_leader[0]
            self.goal.pose.position.y=trans_leader[1]
            self.goal.pose.position.z=trans_leader[2]

            self.goal.pose.orientation.x=rot_leader[0]
            self.goal.pose.orientation.y=rot_leader[1]
            self.goal.pose.orientation.z=rot_leader[2]
            self.goal.pose.orientation.w=rot_leader[3]

            self.move_base_pub.publish(self.goal)

            # Comprobar si el robot ha lleado al destino
            #while True:
                #(trans_1, rot_1) = self.listener.lookupTransform('robot_map', 'robot_base_footprint', rospy.Time(0))
                #(trans_2, rot_2) = self.listener.lookupTransform('robot_map', data, rospy.Time(0))
                #rot_1 = R.from_quat(rot_1)
                #rot_2 = R.from_quat(rot_2)
                #relative_rotation = rot_1.inv() * rot_2
                #self.distance = math.dist(trans_1, trans_2)
                #self.angle = relative_rotation.magnitude() * 180 / math.pi
                #if self.distance <= 0.2 and self.angle <= 20:
                #   break

            while not self.get_data:
                pass
            
            if "ref" not in data and "init" not in data:
                time.sleep(2) # Esperar 2 segundos para que el robot se estabilice
                print("Getting data")
                self.data_acq_pub.publish(True)
    
            self.get_data=False
                
        
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            pass

    def tf_pub_gps(self, event=None):
        for i, coord in enumerate(self.coordinates):
            x_init = self.coordinates[0][1]
            y_init = self.coordinates[0][2]
            x= coord[1]-x_init
            y= coord[2]-y_init
            self.broadcaster.sendTransform((-x,-y,0),(0,0,0,1),rospy.Time.now(),coord[0],"robot_map")
    
    def tf_pub_map(self, event=None):
        for i, coord in enumerate(self.coordinates):
            pos_x=coord[1]
            pos_y=coord[2]
            pos_z=coord[3]
            rot_x=coord[4]
            rot_y=coord[5]
            rot_z=coord[6]
            rot_w=coord[7]
            self.broadcaster.sendTransform((pos_x,pos_y,pos_z),(rot_x,rot_y,rot_z,rot_w),rospy.Time.now(),coord[0],"robot_map")
    
    def start(self):
        #self.read_csv_gps(self.file_path)
        self.read_csv_map(self.file_path)
        #rospy.Timer(rospy.Duration(0.1), self.tf_pub_gps)
        rospy.Timer(rospy.Duration(0.1), self.tf_pub_map)
        rospy.spin()

#Iniciar el nodo
if __name__ == '__main__':
    rospy.init_node('gps_nav', anonymous=True)
    gps_nav_object = gps_nav()
    try:
        gps_nav_object.start()
    except rospy.ROSInterruptException:
        pass
