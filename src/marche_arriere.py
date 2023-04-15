#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
Projet PFE : Voiture autonome
@Author : Eliot CHRISTON FRAGA
"""

import rospy

from std_msgs.msg import Float32MultiArray, Float32, String, Bool

class MarcheArriere : 

    def __init__(self, nb_tofs=2) : 
        # variables
        self.speed = 0
        self.angle = 0
        self.compute = True

        # parameters
        self.actualize_params()

        # print the parameters
        print("PARAMETERS : ")
        print("BACKWARD_SPEED = %.2f" % self.BACKWARD_SPEED)

        # Init ROS node
        rospy.init_node('marche_arriere', anonymous=True)

        # Init ROS PUBLISHERS
        self.pub_rear_obstacle = rospy.Publisher("/RearObstacle", Bool, queue_size = 1)
        self.pub_nav = rospy.Publisher("/MarcheArriereSpeedAngleCommand", Float32MultiArray, queue_size = 1)

        # Init ROS SUBSCRIBERS
        self.sub_activation = rospy.Subscriber("/Marche_arriere", Bool, self.callback_nav)
    
    def callback_nav(self, msg) :
        """ Callback function for the navigation topic """
        self.compute = msg.data
    
    def actualize_params(self) : 
        """ Actualize the parameters """
        self.BACKWARD_SPEED = rospy.get_param("/BACKWARD_SPEED", -0.5)
        self.K_backwards = rospy.get_param("/K_backwards", 0.3)
    
    def publish_speed_angle(self) : 
        """ Publish the speed and the angle """
        # create the message
        msg = Float32MultiArray()
        msg.data = [self.speed, self.angle]

        # publish the message
        self.pub_nav.publish(msg)

    def run(self) : 
        """ Main loop """
        # set the rate
        hz = 25
        rate = rospy.Rate(hz)

        # main loop
        while not rospy.is_shutdown() : 
            # actualize the parameters
            self.actualize_params()

            # check if the navigation is activated
            if not self.compute :
                continue
            
            self.speed = (1-self.K_backwards)*self.speed + self.K_backwards*self.BACKWARD_SPEED
            self.angle = 0.0
            # publish the speed and the angle
            self.publish_speed_angle()

            # sleep
            rate.sleep()

if __name__ == "__main__" : 
    # create the object
    marche_arriere = MarcheArriere()

    # run the object
    marche_arriere.run()

    # spin
    rospy.spin()