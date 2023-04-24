#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
Projet PFE : Voiture autonome
@Author : Eliot CHRISTON FRAGA, Julien JOYET
"""

import rospy

from std_msgs.msg import Float32MultiArray, Float32, Bool, Int8

class Navigation() : 

    def __init__(self) :

        # variables
        self.speed = 0.0
        self.angle = 0.0
        self.marche_arr = {"speed" : 0.0, "angle" : 0.0}
        self.nav_lidar = {"speed" : 0.0, "angle" : 0.0}
        self.EP=0
        self.start=False

        # Init ROS node
        rospy.init_node('navigation_haut_niveau', anonymous=True)

        # Init ROS publishers
        self.pub_speed = rospy.Publisher("/SpeedCommand", Float32, queue_size = 1)
        self.pub_angle = rospy.Publisher("/AngleCommand", Float32, queue_size = 1)

        # Init ROS subscribers
        self.sub_marche_arr = rospy.Subscriber("/MarcheArriereSpeedAngleCommand", Float32MultiArray, self.callback_march_arr)
        self.sub_nav_lidar = rospy.Subscriber("/LidarSpeedAngleCommand", Float32MultiArray, self.callback_lidar)
        self.sub_state = rospy.Subscriber("/State", Int8, self.callback_state)
        
        # other subscribers should be added here
        self.sub_start = rospy.Subscriber("/Start_flag",Bool,self.start_stop_callback) #start/stop flag control from teleop_robot.py

    def start_stop_callback(self,msg):
        self.start=msg.data

    def callback_march_arr(self,msg):

        self.marche_arr["speed"] = msg.data[0]
        self.marche_arr["angle"] = msg.data[1]

    def callback_lidar(self, msg) :
        """ Callback for the lidar commands"""
        self.nav_lidar["speed"] = msg.data[0]
        self.nav_lidar["angle"] = msg.data[1]

    def callback_state(self, msg) :
         """ Callback for the robot state"""
         self.EP=msg.data

    def set_speed_angle(self, speed, angle) : 
        """ Set the speed and the angle of the car"""
        self.speed = speed
        self.angle = angle
        self.pub_speed.publish(self.speed)
        self.pub_angle.publish(self.angle)


    def run(self) :
        """ Main loop of the navigation"""

        # constants


        # frequency of the loop
        HZ = 20 # Hz
        rate = rospy.Rate(HZ)
        nav=""
        # main loop
        while not rospy.is_shutdown() :
            #En fonction de l'état on choisi quel commande de vitesse utiliser

            if self.start==True:
                if self.EP == 0:
                    if nav!="lidar":
                        rospy.loginfo("NAV LIDAR")
                        nav="lidar"
                    self.set_speed_angle(self.nav_lidar["speed"], self.nav_lidar["angle"])
                elif self.EP == 1:
                    if nav!="marche_arriere":
                        rospy.loginfo("MARCHE_ARRIERE")
                        nav="marche_arriere"
                    self.set_speed_angle(self.marche_arr["speed"], self.marche_arr["angle"])
                    
            else:
                self.set_speed_angle(0, 0)

                
            
            rate.sleep()
        

if __name__ == "__main__" :
    
        nav = Navigation()
        nav.run()

        rospy.spin()