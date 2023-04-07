#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
Projet PFE : Voiture autonome
@Author : Eliot CHRISTON FRAGA
"""

import rospy

from std_msgs.msg import Float32MultiArray, Float32, String

class Navigation() : 

    def __init__(self, MAX_SPEED=1.0, MAX_ANGLE=1.0, MAX_DIST=1.5, MIN_DIST=0.15, MIN_SPEED=0.2, BACKWARD_SPEED=0.5, LEFT_IS_GREEN=True) : 
        # variables
        self.speed = 0
        self.angle = 0
        self.dist = [10.0, 10.0, 10.0, 10.0]
        self.wall_color = ""

        # constants
        self.MAX_SPEED = MAX_SPEED # default max speed of the car
        self.MAX_ANGLE = MAX_ANGLE # default max angle of the car
        self.MAX_DIST  = MAX_DIST  # default max distance of the tofs
        self.MIN_DIST  = MIN_DIST  # minimum distance = too close
        self.MIN_SPEED = MIN_SPEED # minimum speed = too slow
        self.BACKWARD_SPEED = BACKWARD_SPEED # speed command when the car is going backward
        self.LEFT_IS_GREEN = LEFT_IS_GREEN # True if the left wall is green, False if the right wall is green

        # print the parameters
        print("PARAMETERS : ")
        print("MAX_SPEED = %.2f" % self.MAX_SPEED)
        print("MAX_ANGLE = %.2f" % self.MAX_ANGLE)
        print("MAX_DIST  = %.2f" % self.MAX_DIST)
        print("MIN_DIST  = %.2f" % self.MIN_DIST)
        print("MIN_SPEED = %.2f" % self.MIN_SPEED)
        print("BACKWARD_SPEED = %.2f" % self.BACKWARD_SPEED)
        print("LEFT_IS_GREEN = %s" % self.LEFT_IS_GREEN)

        # Init ROS node
        rospy.init_node('nav_tofs', anonymous=True)

        # Init ROS PUBLISHERS
        self.pub_nav = rospy.Publisher("/TofsSpeedAngleCommand", Float32MultiArray, queue_size = 1)
        self.pub_message = rospy.Publisher("/TofsNavMessage", String, queue_size = 1)

        # Init ROS SUBSCRIBERS
        self.sub_dist = rospy.Subscriber("/TofsDistance", Float32MultiArray, self.callback_dist)
        self.sub_wall = rospy.Subscriber("/WallColor", String, self.callback_wall)
    
    def callback_wall(self, msg) :
        """ Callback function for the wall color topic """
        self.wall_color = msg.data

    def callback_dist(self, msg) :
        """ Callback function for the distance topic """
        self.dist = msg.data

    def set_speed_angle(self, speed, angle) :
        """ Set the speed and angle of the car """
        self.speed = speed
        self.angle = angle
        self.pub_nav.publish(Float32MultiArray(data=[self.speed, self.angle]))

    def actualize_params(self) :
        """ Actualize the parameters of the car """
        topic_folder = rospy.get_param("topic_folder", default="nav_tofs/")
        self.MAX_SPEED = rospy.get_param(topic_folder+"max_speed",  default=self.MAX_SPEED)
        self.MAX_ANGLE = rospy.get_param(topic_folder+"max_angle",  default=self.MAX_ANGLE)
        self.MAX_DIST  = rospy.get_param(topic_folder+"max_dist",   default=self.MAX_DIST)
        self.MIN_DIST  = rospy.get_param(topic_folder+"min_dist",   default=self.MIN_DIST)
        self.MIN_SPEED = rospy.get_param(topic_folder+"min_speed",  default=self.MIN_SPEED)
        self.BACKWARD_SPEED = rospy.get_param(topic_folder+"backward_speed", default=self.BACKWARD_SPEED)
        self.LEFT_IS_GREEN = rospy.get_param("left_is_green", default=self.LEFT_IS_GREEN)


    def run(self) :
        """ Main loop of the navigation running with front and back tofs"""

        # lateral distance to the obstacle
        recul_dist = rospy.get_param("recul_dist",default=0.5)

        # boleans
        going_backwards = False
        negative_angle = False

        # coefficients : "how much the new value is important"
        K_angle = 0.20 # proportional coefficient for the angle
        K_speed_des = 0.20 # proportional coefficient for the speed desceleartion
        K_speed_acc = 0.10 # proportional coefficient for the speed acceleration

        # frequency of the loop
        HZ = 25 # Hz
        rate = rospy.Rate(HZ)

        # main loop
        while not rospy.is_shutdown() :

            # actualize the parameters
            self.actualize_params()

            sensor = {
                "fl" : self.dist[0], # front left
                "fr" : self.dist[1], # front right
                "rl" : self.dist[2], # rear left
                "rr" : self.dist[3], # rear right
            }

            # calculate the current shortest distance and normalize it between 0 and 1
            dist = min(sensor["fl"], sensor["fr"])
            norma_dist = dist / self.MAX_DIST

            # if the car is going backwards
            if going_backwards :
                mess = "Going backward "

                # if the obstacle is at recul_distance or more => go forward
                if min(sensor["fl"], sensor["fr"]) > recul_dist :
                    going_backwards = False
                    new_speed = 0.0
                    new_angle = 0.0

                # if the obstacle is too close and no obstacle behind => go backward
                elif sensor["rl"] > self.MIN_DIST and sensor["rr"] > self.MIN_DIST :
                    new_speed = -self.BACKWARD_SPEED
                    # going backward, we need to turn the other way
                    if negative_angle :
                        new_angle = self.MAX_ANGLE
                    else :
                        new_angle = -self.MAX_ANGLE

                # if the obstacle is too close and an obstacle is behind => stop
                else :
                    new_speed = 0.0
                    new_angle = 0.0
                    mess = "Too close on both sides, stoping"
            
            # elif a front obstacle is detected
            elif norma_dist < 0.999 :
                mess = "Obstacle detected, navigating around it"

                # calculate the new speed and angle
                new_speed = (self.MAX_SPEED - self.MIN_SPEED) * norma_dist + self.MIN_SPEED
                new_angle = (1-norma_dist) * self.MAX_ANGLE

                # angle limitation
                if new_angle > self.MAX_ANGLE :
                    new_angle = self.MAX_ANGLE
                
                # if the difference between the front sensors is too small => turn in the right direction
                if (abs(sensor["fr"] - sensor["fl"]) < 3.) and (self.wall_color in ["green", "red"]) :
                    mess = "Turning with wall color information"
                    if (self.LEFT_IS_GREEN and self.wall_color == "red") or ((not self.LEFT_IS_GREEN) and self.wall_color == "green") :
                        new_angle = -new_angle
                # if the right sensor is closer => turn right
                elif sensor["fr"] < sensor["fl"] :
                    new_angle = -new_angle

                if min(sensor["fl"], sensor["fr"]) < self.MIN_DIST :
                    # if the obstacle is under MIN_DIST => go backward
                    going_backwards = True
                    if new_angle < 0 : negative_angle = True
                    else :             negative_angle = False
                    new_angle = 0.0
                    new_speed = 0.0
            
            # no obstacle detected => go forward
            else :
                mess = "No information, navigating strait"
                new_speed = self.MAX_SPEED
                new_angle = 0.0
            
            # determine the speed coefficient
            if new_speed > self.speed :
                K_speed = K_speed_acc
            else :
                K_speed = K_speed_des

            # application of the coefficients
            new_speed = K_speed * new_speed + (1-K_speed) * self.speed
            new_angle = K_angle * new_angle + (1-K_angle) * self.angle
            
            # set the new speed and angle and publish the message
            self.set_speed_angle(new_speed, new_angle)
            self.pub_message.publish(mess)

            rate.sleep()
        

if __name__ == "__main__" :

        # get the parameters
        topic_folder = rospy.get_param("topic_folder", default="nav_tofs/")
        max_sp = rospy.get_param(topic_folder+"max_speed", default=1.0)
        min_sp = rospy.get_param(topic_folder+"min_speed", default=0.2)
        max_ds = rospy.get_param(topic_folder+"tofs_default_max_dist",  default=1.5)
        min_ds = rospy.get_param(topic_folder+"min_dist",  default=0.15)
        max_ag = rospy.get_param(topic_folder+"max_angle", default=1.0)
        back_sp = rospy.get_param(topic_folder+"backward_speed", default=0.5)
        left_is_green = rospy.get_param(topic_folder+"left_is_green", default=True)

        
        nav = Navigation(MAX_SPEED=max_sp, MAX_ANGLE=max_ag, MAX_DIST=max_ds, MIN_DIST=min_ds, MIN_SPEED=min_sp, BACKWARD_SPEED=back_sp, LEFT_IS_GREEN=left_is_green)
        nav.run()

        rospy.spin()