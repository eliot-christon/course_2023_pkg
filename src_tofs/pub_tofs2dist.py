#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
Projet PFE : Voiture autonome
@Author : Eliot CHRISTON FRAGA
"""

import rospy
from std_msgs.msg import Float32MultiArray, Int16MultiArray

class Distance() : 

    def __init__(self, MAX_DIST=10.0, nb_tofs=4, queue_size=5) : 
        
        self.nb_tofs = nb_tofs
        self.MAX_DIST = MAX_DIST # default max distance of the tofs
        self.dist = [MAX_DIST] * self.nb_tofs
        self.queue = [self.dist] * queue_size

        # Init ROS node
        rospy.init_node('pub_tofs2dist', anonymous=True)

        # Init ROS publishers
        self.pub_dist = rospy.Publisher("/TofsDistance", Float32MultiArray, queue_size = 1)

        # Init ROS subscribers
        # By default we're using the simulation topics
        self.tof_topic = rospy.get_param("tof_topic", default="/TofsScan")

        if(self.tof_topic == "/SensorsScan"):
            self.sub_tofs = rospy.Subscriber(self.tof_topic, Float32MultiArray, self.callback_tofs_simu)
        elif(self.tof_topic == "/TofsScan"):
            print("Real robot")
            self.sub_tofs = rospy.Subscriber(self.tof_topic, Int16MultiArray, self.callback_tofs)
        else:
            print("Error tof topic name")
            exit(1)

    def callback_tofs_simu(self, msg) :
        """ Callback of the simulated tofs subscriber """
        # [front_left, front_right, rear_left, rear_right]
        self.dist = [d if d > 0.001 else self.MAX_DIST for d in msg.data]
        self.movingAverage_filter()
        self.pub_dist.publish(Float32MultiArray(data=self.dist))

    def callback_tofs(self, msg) :
        """ Callback of the tofs subscriber """
        self.dist = [d/(1000) if d/(1000)<self.MAX_DIST else self.MAX_DIST for d in msg.data] # conversion to meters
        self.movingAverage_filter()
        self.pub_dist.publish(Float32MultiArray(data=self.dist))

    def movingAverage_filter(self) :
        """ Filter the distance with a moving average"""
        self.queue.pop(0)
        self.queue.append(self.dist)
        self.dist = [sum([d[i] for d in self.queue]) / len(self.queue) for i in range(self.nb_tofs)]
    


if __name__ == "__main__" :

    topic_folder = rospy.get_param("topic_folder", default="nav_tofs/")
    max_ds = rospy.get_param(topic_folder+"tofs_default_max_dist",  default=1.5)

    Distance(MAX_DIST=max_ds)
    # now we can use rospy.spin() to keep the node alive
    rospy.spin()