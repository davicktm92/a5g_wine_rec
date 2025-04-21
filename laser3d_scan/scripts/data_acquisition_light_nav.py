#!/usr/bin/env python3

import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
from geometry_msgs.msg import Pose
from std_msgs.msg import Int8, Bool, String
import numpy as np
from tf.transformations import quaternion_from_euler

rospy.sleep(10)
moveit_commander.roscpp_initialize(sys.argv)
rob_description="z1_arm/robot_description"
robot = moveit_commander.RobotCommander(robot_description=rob_description,ns="z1_arm")
group = moveit_commander.MoveGroupCommander("manipulator", robot_description=rob_description, ns="z1_arm")
#group.set_goal_orientation_tolerance(np.deg2rad(5))
sceneRate=2

v_scan_speed=0.07
h_scan_speed=0.07
spectral_scan_speed=0.9
arm_speed=0.9

sleep_time=0.2
spectral_time=4

accel=0.5

class data_acquisition:
  def __init__(self):
    self.scan_publisher = rospy.Publisher("/robot/state_scan", Int8, queue_size=1)
    self.spectral_publisher = rospy.Publisher("/robot/save_spectral", Int8, queue_size=1)
    self.dir_publisher = rospy.Publisher("/robot/directory", String, queue_size=1)
    self.save3d_publisher = rospy.Publisher("/robot/3dscan_save", Int8, queue_size=1)
    self.clear3d_publisher = rospy.Publisher("/robot/3dscan_clear", Int8, queue_size=1)
    
    self.phase=0 #0=nothing, 1=v_scan, 2=h_scan, 3=spectral_scan
    self.exec=0 #0=not executing, 1=executing
    
  def moveit_go(self,speed,x,y,z,roll,pitch,yaw):
    group.clear_pose_targets()
    
    pose_target=Pose()
    pose_target.position.x = x
    pose_target.position.y = y
    pose_target.position.z = z

    quat = quaternion_from_euler(roll, pitch, yaw)

    pose_target.orientation.x = quat[0]
    pose_target.orientation.y = quat[1]
    pose_target.orientation.z = quat[2]
    pose_target.orientation.w = quat[3]

    group.set_pose_target(pose_target)

    group.set_max_velocity_scaling_factor(speed)
    group.set_max_acceleration_scaling_factor(accel)
    group.go(wait=True)

  def v_scan(self):
    self.exec=1
    group.get_current_pose().pose
    self.moveit_go(arm_speed,0.0,0.0,0.65,0.0,0.0,0.0)
    rospy.sleep(sleep_time)
    #Command to start scanning
    self.scan_publisher.publish(1)
    #self.moveit_go(v_scan_speed,0,0,0.61,0,0,0)
    self.moveit_go(v_scan_speed,0,0,0.5,0,0,0)
    self.moveit_go(v_scan_speed,0,0,0.4,0,0,0)
    self.moveit_go(v_scan_speed,0,0,0.3,0,0,0)
    self.moveit_go(v_scan_speed,0,0,0.2,0,0,0)
    self.scan_publisher.publish(0)
    rospy.sleep(sleep_time)
    self.home()

    self.phase+=1
    self.exec=0

    print(self.phase)
  
  #Puntos para planificacion vertical
  #1) x=0, y=0, z=0.61, w=1 speed scaling 0.5
  #2) publish scan
  #3) x=0, y=0, z=0.2, w=1 speed_scaling 0.05
  #4) stop scan
  #5) home

  def h_scan(self):
    self.exec=1
    group.get_current_pose().pose
    #self.moveit_go(arm_speed,0.0,0.0,0.35,0.0,0.0,0.0)
    self.moveit_go(arm_speed,0.25,0.45,0.35,0,0.0,0.0)
    self.moveit_go(arm_speed,0.25,0.45,0.35,1.57,0.0,0.0)
    rospy.sleep(sleep_time)
    self.scan_publisher.publish(2)
    self.moveit_go(h_scan_speed,0.25,0.30,0.35,1.57,0.0,0.0)
    self.moveit_go(h_scan_speed,0.25,0.15,0.35,1.57,0.0,0.0)
    self.moveit_go(h_scan_speed,0.25,0.0,0.35,1.57,0.0,0.0)
    self.moveit_go(h_scan_speed,0.25,-0.15,0.35,1.57,0.0,0.0)
    self.moveit_go(h_scan_speed,0.25,-0.30,0.35,1.57,0.0,0.0)
    self.moveit_go(h_scan_speed,0.25,-0.45,0.35,1.57,0.0,0.0)
    self.scan_publisher.publish(0)
    rospy.sleep(sleep_time)
    #self.moveit_go(arm_speed,0.0,0.0,0.3,0.0,0.0,0.0)
    self.home()

    self.phase+=1
    self.exec=0

  #Puntos para planificacion horizontal
  #1) x=0.0, y=0.0, z=0.35, w=1 speed scaling 0.5
  #2) x=0.25, y=0.45, z=0.35, ori= x=-0.7, w=0.707, speed scaling 0.5
  #3) publish scan
  #4) x=0.25, y=0.0, z=0.35, ori= x=-0.7, w=0.707, speed scaling 0.05
  #5) x=0.25, y=-0.45, z=0.35, ori= x=-0.7, w=0.707, speed scaling 0.05
  #6) stop scan
  #7) x=0.0, y=0.0, z=0.35, w=1 speed scaling 0.5
  #8) home
    
  def spectral_scan(self):
    self.exec=1
    group.get_current_pose().pose

    self.moveit_go(arm_speed,-0.1,0.21,0.53,0,0.0,0.0)
    rospy.sleep(sleep_time)
    self.spectral_publisher.publish(1)
    rospy.sleep(spectral_time)
    self.spectral_publisher.publish(0)

    #self.moveit_go(spectral_scan_speed,0.2,0.15,0.55,0,0.0,0.0)

    self.moveit_go(spectral_scan_speed,-0.1,0.0,0.53,0,0.0,0.0)
    self.spectral_publisher.publish(1)
    rospy.sleep(spectral_time)
    self.spectral_publisher.publish(0)

    #self.moveit_go(spectral_scan_speed,0.2,-0.15,0.55,0,0.0,0.0)
    self.moveit_go(spectral_scan_speed,-0.1,-0.21,0.53,0,0.0,0.0)
    self.spectral_publisher.publish(1)
    rospy.sleep(spectral_time)
    self.spectral_publisher.publish(0)

    #self.moveit_go(spectral_scan_speed,-0.08,0.0,0.35,0,0.0,0.0)

    self.moveit_go(spectral_scan_speed,-0.08,-0.25,0.35,0,0.0,0.0)
    self.spectral_publisher.publish(1)
    rospy.sleep(spectral_time)
    self.spectral_publisher.publish(0)

    #self.moveit_go(spectral_scan_speed,0.2,-0.25,0.25,0,0.0,0.0)
    self.moveit_go(spectral_scan_speed,-0.08,-0.0,0.35,0,0.0,0.0)
    self.spectral_publisher.publish(1)
    rospy.sleep(spectral_time)
    self.spectral_publisher.publish(0)

    #self.moveit_go(spectral_scan_speed,0.2,0.25,0.25,0,0.0,0.0)
    self.moveit_go(spectral_scan_speed,-0.08,0.25,0.35,0,0.0,0.0)
    self.spectral_publisher.publish(1)
    rospy.sleep(spectral_time)
    self.spectral_publisher.publish(0)

    
    rospy.sleep(sleep_time)
    self.spectral_publisher.publish(0)
    #self.moveit_go(arm_speed,0.0,0.0,0.3,0.0,0.0,0.0)
    self.home()
    self.home_rotated()

    #save pointcloud
    self.save3d_publisher.publish(1)
    rospy.sleep(spectral_time)
    self.clear3d_publisher.publish(1)

    self.phase=0
    self.exec=0
    self.start=False
    

  #Puntos para fotos de la multiespectral
  #1) x=0.0, y=0.0, z=0.35, w=1 speed scaling 0.5
  #2) x=0.2, y=0.35, z=0.55, w=1 speed scaling 0.5
  #3) start saving photos
  #3) x=0.2, y=0.15, z=0.55, w=1 speed scaling 0.1
  #4) x=0.2, y=0.0, z=0.55, w=1 speed scaling 0.1
  #5) x=0.2, y=-0.15, z=0.55, w=1 speed scaling 0.1
  #6) x=0.2, y=-0.35, z=0.55, w=1 speed scaling 0.1
  #7) x=0.2, y=-0.5, z=0.40, w=1 speed scaling 0.1
  #8) x=0.2, y=-0.25, z=0.40, w=1 speed scaling 0.1
  #9) x=0.2, y=-0.0, z=0.40, w=1 speed scaling 0.1
  #10) x=0.2, y=0.25, z=0.40, w=1 speed scaling 0.1
  #11) x=0.2, y=0.5, z=0.40, w=1 speed scaling 0.1
  #12) x=0.2, y=0.5, z=0.25, w=1 speed scaling 0.1
  #13) x=0.2, y=0.25, z=0.25, w=1 speed scaling 0.1
  #14) x=0.2, y=0.0, z=0.25, w=1 speed scaling 0.1
  #15) x=0.2, y=-0.25, z=0.25, w=1 speed scaling 0.1
  #16) x=0.2, y=-0.5, z=0.25, w=1 speed scaling 0.1
  #17) stop saving photos
  #18) x=0.0, y=0.0, z=0.35, w=1 speed scaling 0.5
  #19) home
  def home(self):
    group.clear_pose_targets()
    print ("Home Position")
    joint_goal = group.get_current_joint_values()
    joint_goal[0] = 0
    joint_goal[1] = 0
    joint_goal[2] = 0
    joint_goal[3] = -0.07435
    joint_goal[4] = 0
    joint_goal[5] = 0

    group.set_max_velocity_scaling_factor(0.4)
    group.go(joint_goal, wait=True)
    group.stop()
    #rospy.sleep(1)
  
  def home_rotated(self):
    group.clear_pose_targets()
    print ("Home Rotated Position")
    joint_goal = group.get_current_joint_values()
    joint_goal[0] = 1.5708
    joint_goal[1] = 0
    joint_goal[2] = 0
    joint_goal[3] = -0.07435
    joint_goal[4] = 0
    joint_goal[5] = 0

    group.set_max_velocity_scaling_factor(0.4)
    group.go(joint_goal, wait=True)
    group.stop()
    rospy.sleep(sleep_time)


  def acquisitionCb(self,msg):
    self.start=msg.data
    self.phase=1

  def rothomeCb(self,msg):
    if msg.data:
      self.home_rotated()
    

  def scene(self,event=None):
    if self.start==True:
           
      if self.phase==1 and self.exec==0:        
        rospy.loginfo("Start data acquisition")
        #self.dir=input("Nombre de zona a explorar (1_front): ")
        dir_name=String()
        dir_name.data=self.dir
        self.dir_publisher.publish(dir_name)
        rospy.loginfo("Start Vertical Scanning")
        rospy.loginfo(self.dir)
        self.v_scan()
      elif self.phase==2 and self.exec==0:
        rospy.loginfo("Start Horizontal Scanning")
        rospy.loginfo(self.dir)
        self.h_scan()
      elif self.phase==3:
        self.spectral_scan()
    else:
      #rospy.loginfo("Stop data acquisition")
      self.phase=0

  def directoryCb(self,msg):
    self.dir=msg.data

  def start(self):
    rospy.Subscriber('/robot/data_acquisition',Bool,self.acquisitionCb)
    rospy.Subscriber('/robot/home_rotated',Bool,self.rothomeCb)
    rospy.Subscriber('/robot/destination',String,self.directoryCb)
    rospy.Timer(rospy.Duration(1/sceneRate), self.scene)

if __name__ == '__main__':
  rospy.init_node('data_acquisition', anonymous=True) #make node
  data1=data_acquisition()
  try:
    data1.start()
    rospy.spin()
  except rospy.ROSInterruptException:
    group.stop()
    moveit_commander.roscpp_shutdown()
    #pass


