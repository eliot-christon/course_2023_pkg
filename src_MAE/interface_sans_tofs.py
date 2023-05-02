#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
Projet PFE : Voiture autonome
@Author : Eliot CHRISTON FRAGA, Julien JOYET
"""

import rospy

from std_msgs.msg import Float32MultiArray, Float32, Bool, Int8

class Navigation() : 
    """Ce noeud a pour but de faire la transition entre les différents noeuds de navigation et les commandes données au robot,
        les commandes données au robot vont dépendre de l'état donné par la MAE:
        -Etat 0: Navigation lidar
        -Etat 1: Marche_arrière
        
    Ce noeud d'interface prend en entrée:
        /LidarSpeedAngleCommand
        /MarcheArriereSpeedAngleCommand
        /State
        
    Les sorties de ce noeud d'interface sont:
        /SpeedCommand
        /AngleCommand
        
    Les entrées sont les commandes envoyées des différents noeuds de navigation et les sorties sont des commandes pour le robot choisies parmi les différentes navigations en fonction de l'état"""


    def __init__(self) :
        # Init ROS node
        rospy.init_node('navigation_haut_niveau', anonymous=True)

            #======Attributs======
        #Commandes par default
        self.speed = 0.0
        self.angle = 0.0
        #Initialisation des attributs qui vont prendre les valeurs de commandes des différentes navigations
        self.marche_arr = {"speed" : 0.0, "angle" : 0.0}
        self.nav_lidar = {"speed" : 0.0, "angle" : 0.0}
        #Etat présent
        self.EP=0
        #Variable d'activation de la navigation du robot
        self.start=False

            #======Publishers======
        self.pub_speed = rospy.Publisher("/SpeedCommand", Float32, queue_size = 1)
        self.pub_angle = rospy.Publisher("/AngleCommand", Float32, queue_size = 1)

            #======Subscribers======
        self.sub_marche_arr = rospy.Subscriber("/MarcheArriereSpeedAngleCommand", Float32MultiArray, self.callback_march_arr)
        self.sub_nav_lidar = rospy.Subscriber("/LidarSpeedAngleCommand", Float32MultiArray, self.callback_lidar)
        self.sub_state = rospy.Subscriber("/State", Int8, self.callback_state)
        self.sub_start = rospy.Subscriber("/Start_flag",Bool,self.start_stop_callback) #start/stop flag control from teleop_robot.py

# CALLBACKS ==============================================================================================================

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

# PUBLICATION DES COMMANDES DU ROBOT ==============================================================================================================

    def set_speed_angle(self, speed, angle) : 
        """ Set the speed and the angle of the car"""
        self.speed = speed
        self.angle = angle
        self.pub_speed.publish(self.speed)
        self.pub_angle.publish(self.angle)

# Algo ==============================================================================================================

    def run(self) :
        """ Main loop of the navigation"""

        # frequency of the loop
        HZ = 20 # Hz
        rate = rospy.Rate(HZ)
        nav=""

        # main loop
        while not rospy.is_shutdown() :
            #En fonction de l'état on choisit quelle commande de vitesse utiliser
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