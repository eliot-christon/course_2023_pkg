#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
Projet PFE : Voiture autonome
@Author : Eliot CHRISTON FRAGA
"""

import rospy

from std_msgs.msg import Float32

class Controller() : 

    def __init__(self, max_speed, max_angle) : 
        self.max_speed = max_speed
        self.max_angle = max_angle
        self.speed = 0
        self.angle = 0

        # Init ROS node
        rospy.init_node('go_strait_node', anonymous=True)
        # Init ROS publishers
        self.pub_speed = rospy.Publisher("/SpeedCommand", Float32, queue_size = 1)
        self.pub_angle = rospy.Publisher("/AngleCommand", Float32, queue_size = 1) 

    def set_speed(self, speed) : 
        self.speed = speed
        self.pub_speed.publish(self.speed)

    def set_angle(self, angle) : 
        self.angle = angle
        self.pub_angle.publish(self.angle)

    def run(self) :
        hz = 10 # Hz
        rate = rospy.Rate(hz)
        while not rospy.is_shutdown() :
            self.set_speed(self.max_speed)
            self.set_angle(0.0)
            #print("Speed : ", self.speed)
            rate.sleep()
        

if __name__ == "__main__" :
    
        max_speed = rospy.get_param("max_speed", default=1.0)
        max_angle = rospy.get_param("max_angle", default=1.0)
    
        controller = Controller(max_speed, max_angle)
        controller.run()

        rospy.spin()