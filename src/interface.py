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
        self.nav_tofs = {"speed" : 0.0, "angle" : 0.0}
        self.nav_lidar = {"speed" : 0.0, "angle" : 0.0}
        self.d_tour = {"speed" : 0.0, "angle" : 0.0}
        self.tofs = {"fl" : 0.0, "fr" : 0.0, "bl" : 0.0, "br" : 0.0}
        self.EP=0

        # Init ROS node
        rospy.init_node('navigation_haut_niveau', anonymous=True)

        # Init ROS publishers
        self.pub_speed = rospy.Publisher("/SpeedCommand", Float32, queue_size = 1)
        self.pub_angle = rospy.Publisher("/AngleCommand", Float32, queue_size = 1)
        self.pub_dist_lim = rospy.Publisher("/Dist_lim", Bool, queue_size = 1)
        self.pub_fin_d_tour = rospy.Publisher("/Fin_d_tour", Bool, queue_size = 1)
        self.pub_dir = rospy.Publisher("/Direction", Bool, queue_size = 1)

        # Init ROS subscribers
        self.sub_tfs_dist = rospy.Subscriber("/TofsDistance", Float32MultiArray, self.callback_tofs_dist)
        self.sub_nav_tofs = rospy.Subscriber("/TofsSpeedAngleCommand", Float32MultiArray, self.callback_tofs)
        self.sub_nav_lidar = rospy.Subscriber("/LidarSpeedAngleCommand", Float32MultiArray, self.callback_lidar)
        self.sub_d_tour = rospy.Subscriber("/d_tourSpeedAngleCommand", Float32MultiArray, self.callback_d_tour)
        self.sub_state = rospy.Subscriber("/State", Int8, self.callback_state)

        
        # other subscribers should be added here

    def callback_tofs_dist(self, msg) :
        """ Callback for the tofs distance"""

        tofs_lidar_threshold = 1.0 # m

        self.tofs["fl"] = msg.data[0]
        self.tofs["fr"] = msg.data[1]
        self.tofs["bl"] = msg.data[2]
        self.tofs["br"] = msg.data[3]

        if min(self.tofs["fl"], self.tofs["fr"]) < tofs_lidar_threshold:
            self.pub_dist_lim.publish(True)
        else:
            self.pub_dist_lim.publish(False)

    def callback_lidar(self, msg) :
        """ Callback for the lidar commands"""
        self.nav_lidar["speed"] = msg.data[0]
        self.nav_lidar["angle"] = msg.data[1]

    def callback_tofs(self, msg) :
        """ Callback for the tofs commands"""
        self.nav_tofs["speed"] = msg.data[0]
        self.nav_tofs["angle"] = msg.data[1]

    def callback_d_tour(self, msg) :
        """ Callback for the d_tour commands"""
        self.d_tour["speed"] = msg.data[0]
        self.d_tour["angle"] = msg.data[1]

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

        # main loop
        while not rospy.is_shutdown() :
            #En fonction de l'état on choisi quel commande de vitesse utiliser
            stop=rospy.get_param("Stop", default=1)
            if stop==0:
                if self.EP == 0:
                    self.set_speed_angle(self.nav_lidar["speed"], self.nav_lidar["angle"])
                elif self.EP == 1:
                    self.set_speed_angle(self.nav_tofs["speed"], self.nav_tofs["angle"])
                elif self.EP == 2:
                    self.set_speed_angle(self.d_tour["speed"], self.d_tour["angle"])
            else:
                self.set_speed_angle(0.0, 0.0)

                
            
            rate.sleep()
        

if __name__ == "__main__" :
    
        nav = Navigation()
        nav.run()

        rospy.spin()