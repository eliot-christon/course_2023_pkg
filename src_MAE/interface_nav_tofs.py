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
        les commandes données au robot vont dépendre de l'état donnée par la MAE:
        -Etat 0: Navigation lidar
        -Etat 1: Navigation Tofs
        -Etat 2: Demi-tour
        
    Ce noeud d'interface prend en entrée:
        /TofsSpeedAngleCommand
        /LidarSpeedAngleCommand
        /d_tourSpeedAngleCommand
        /Fin_d_tour
        /State
        
    Les sorties de ce noeud d'interface sont:
        /SpeedCommand
        /AngleCommand
        
    Les entrées sont les commandes envoyé des différents noeuds de navigation et les sorties sont des commandes pour le robots choisi parmis les différentes navigations en fonction de l'état"""

    def __init__(self) :
        # Init ROS node
        rospy.init_node('navigation_haut_niveau', anonymous=True)

            #======Attributs======
        #Commandes par default
        self.speed = 0.0
        self.angle = 0.0
        #Initialisations des attributs qui vont prendre les valeurs de commandes des différentes navigation
        self.nav_tofs = {"speed" : 0.0, "angle" : 0.0}
        self.nav_lidar = {"speed" : 0.0, "angle" : 0.0}
        self.d_tour = {"speed" : 0.0, "angle" : 0.0}
        #Etat présent
        self.EP=0
        #Variable d'activation de la navigation du robot
        self.start=False

            #======Publishers======
        self.pub_speed = rospy.Publisher("/SpeedCommand", Float32, queue_size = 1)
        self.pub_angle = rospy.Publisher("/AngleCommand", Float32, queue_size = 1)

            #======Subscribers======
        self.sub_nav_tofs = rospy.Subscriber("/TofsSpeedAngleCommand", Float32MultiArray, self.callback_tofs)
        self.sub_nav_lidar = rospy.Subscriber("/LidarSpeedAngleCommand", Float32MultiArray, self.callback_lidar)
        self.sub_d_tour = rospy.Subscriber("/d_tourSpeedAngleCommand", Float32MultiArray, self.callback_d_tour)
        self.sub_state = rospy.Subscriber("/State", Int8, self.callback_state)
        self.sub_start = rospy.Subscriber("/Start_flag",Bool,self.start_stop_callback) #start/stop flag control from teleop_robot.py

# CALLBACKS ==============================================================================================================

    def start_stop_callback(self,msg):
        self.start=msg.data

    def callback_lidar(self, msg) :
        """ Callback for the lidar commands"""
        self.nav_lidar["speed"] = msg.data[0]
        self.nav_lidar["angle"] = msg.data[1]

    def callback_tofs(self, msg) :
        """ Callback for the tofs commands"""
        self.nav_tofs["speed"] = msg.data[0]
        self.nav_tofs["angle"] = msg.data[1]

    def callback_d_tour(self, msg) :
        """Callback for the d_tour commands"""
        self.d_tour["speed"] = msg.data[0]
        self.d_tour["angle"] = msg.data[1] 

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
            #En fonction de l'état on choisi quel commande de vitesse utiliser
            if self.start==True:
                if self.EP == 0:
                    if nav!="lidar":
                        rospy.loginfo("NAV LIDAR")
                        nav="lidar"
                    self.set_speed_angle(self.nav_lidar["speed"], self.nav_lidar["angle"])
                elif self.EP == 1:
                    if nav!="tofs":
                        rospy.loginfo("NAV TOFS")
                        nav="tofs"
                    self.set_speed_angle(self.nav_tofs["speed"], self.nav_tofs["angle"])
                elif self.EP == 2:
                    if nav!="demi_tour":
                        rospy.loginfo("DEMI-TOUR")
                        nav="demi_tour"
                    self.set_speed_angle(self.d_tour["speed"], self.d_tour["angle"])

            else:
                self.set_speed_angle(0, 0)

            rate.sleep()
        
if __name__ == "__main__" :
    
        nav = Navigation()
        nav.run()
        rospy.spin()