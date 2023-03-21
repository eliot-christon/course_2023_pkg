#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
Projet PFE : Voiture autonome
@Author : Eliot CHRISTON FRAGA
"""

import rospy

from std_msgs.msg import Float32MultiArray, Float32

class Navigation() : 

    def __init__(self, ONLY_LIDAR=False, ONLY_TOFS=False) :

        # constants
        self.ONLY_LIDAR = ONLY_LIDAR
        self.ONLY_TOFS = ONLY_TOFS

        # variables
        self.speed = 0.0
        self.angle = 0.0
        self.nav_tofs = {"speed" : 0.0, "angle" : 0.0}
        self.nav_lidar = {"speed" : 0.0, "angle" : 0.0}
        self.tofs = {"fl" : 0.0, "fr" : 0.0, "bl" : 0.0, "br" : 0.0}

        # Init ROS node
        rospy.init_node('navigation_haut_niveau', anonymous=True)

        # Init ROS publishers
        self.pub_speed = rospy.Publisher("/SpeedCommand", Float32, queue_size = 1)
        self.pub_angle = rospy.Publisher("/AngleCommand", Float32, queue_size = 1)

        # Init ROS subscribers
        if self.ONLY_TOFS :
            self.sub_tfs_dist = rospy.Subscriber("/TofsDistance", Float32MultiArray, self.callback_tofs_dist)
            self.sub_nav_tofs = rospy.Subscriber("/TofsSpeedAngleCommand", Float32MultiArray, self.callback_tofs)
        if self.ONLY_LIDAR :
            self.sub_nav_lidar = rospy.Subscriber("/LidarSpeedAngleCommand", Float32MultiArray, self.callback_lidar)

        
        # other subscribers should be added here

    def callback_tofs_dist(self, msg) :
        """ Callback for the tofs distance"""
        self.tofs["fl"] = msg.data[0]
        self.tofs["fr"] = msg.data[1]
        self.tofs["bl"] = msg.data[2]
        self.tofs["br"] = msg.data[3]

    def callback_lidar(self, msg) :
        """ Callback for the lidar commands"""
        self.nav_lidar["speed"] = msg.data[0]
        self.nav_lidar["angle"] = msg.data[1]

    def callback_tofs(self, msg) :
        """ Callback for the tofs commands"""
        self.nav_tofs["speed"] = msg.data[0]
        self.nav_tofs["angle"] = msg.data[1]

    def set_speed_angle(self, speed, angle) : 
        """ Set the speed and the angle of the car"""
        self.speed = speed
        self.angle = angle
        self.pub_speed.publish(self.speed)
        self.pub_angle.publish(self.angle)


    def run(self) :
        """ Main loop of the navigation"""

        # constants
        tofs_lidar_threshold = 1.0 # m

        # frequency of the loop
        HZ = 20 # Hz
        rate = rospy.Rate(HZ)

        # main loop
        while not rospy.is_shutdown() :
            
            if min(self.tofs["fl"], self.tofs["fr"]) < tofs_lidar_threshold or self.ONLY_TOFS :
                self.set_speed_angle(self.nav_tofs["speed"], self.nav_tofs["angle"])
            else :
                self.set_speed_angle(self.nav_lidar["speed"], self.nav_lidar["angle"])
            
            rate.sleep()
        

if __name__ == "__main__" :
    
        nav = Navigation(ONLY_TOFS = True)
        nav.run()

        rospy.spin()