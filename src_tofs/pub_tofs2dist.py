#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
Projet PFE : Voiture autonome
@Author : Eliot CHRISTON FRAGA
"""

import rospy
from std_msgs.msg import Float32MultiArray, Int16MultiArray, Bool
import numpy as np

class Distance() : 

    def __init__(self, nb_tofs=4, queue_size=3) : 
        
        # Params
        self.tof_topic = rospy.get_param("tof_topic", default="/TofsScan")
        self.tofs_lidar_threshold = rospy.get_param("tofs_lidar_threshold", default=1)
        self.MAX_DIST = rospy.get_param("tofs_default_max_dist",  default=1.5) # default max distance of the tofs

        self.nb_tofs = nb_tofs
        self.dist = [self.MAX_DIST] * self.nb_tofs
        self.queue = [self.dist] * queue_size

        # Init ROS node
        rospy.init_node('pub_tofs2dist', anonymous=True)

        # Init ROS publishers
        self.pub_dist = rospy.Publisher("/TofsDistance", Float32MultiArray, queue_size = 1)
        self.pub_lim = rospy.Publisher("/Dist_lim", Bool, queue_size = 1)

        # Params
        self.tof_topic = rospy.get_param("tof_topic", default="/TofsScan")
        self.tofs_lidar_threshold = rospy.get_param("tofs_lidar_threshold", default=1)

        # Init ROS subscribers
        if(self.tof_topic == "/SensorsScan"):
            self.sub_tofs = rospy.Subscriber(self.tof_topic, Float32MultiArray, self.callback_tofs_simu)
        elif(self.tof_topic == "/TofsScan"):
            print("Real robot")
            self.sub_tofs = rospy.Subscriber(self.tof_topic, Int16MultiArray, self.callback_tofs)
        else:
            print("Error tof topic name")
            exit(1)

# CALLBACKS ==============================================================================================================

    def callback_tofs_simu(self, msg) :
        """ Callback of the simulated tofs subscriber """
        # [front_left, front_right, rear_left, rear_right]
        self.dist = [d if d > 0.001 else self.MAX_DIST for d in msg.data]
        self.global_publisher()

    def callback_tofs(self, msg) :
        """ Callback of the tofs subscriber """
        self.dist = [d/(1000) if d/(1000)<self.MAX_DIST else self.MAX_DIST for d in msg.data] # conversion to meters
        self.global_publisher()

# PUBLISHER ==============================================================================================================

    def global_publisher(self) :
        """ Callback of the global subscriber """
        self.eliminate_zero_values()
        self.ponderated_mean_filter()
        self.pub_dist.publish(Float32MultiArray(data=self.dist))
        # Check if the distance is under a threshold
        if min(self.dist[:2]) < self.tofs_lidar_threshold :
            self.pub_lim.publish(Bool(data=True))
        else :
            self.pub_lim.publish(Bool(data=False))

# FILTERS ================================================================================================================

    def movingAverage_filter(self) :
        """ Filter the distance with a moving average"""
        self.queue.pop(0)
        self.queue.append(self.dist)
        self.dist = [sum([d[i] for d in self.queue]) / len(self.queue) for i in range(self.nb_tofs)]
    
    def ponderated_mean_filter(self) :
        """ Filter the distance with a ponderated mean"""
        self.queue.pop(0)
        self.queue.append(self.dist)
        temp = np.array(self.queue).T
        self.dist = [np.mean(temp[i], weights=[i+1 for i in range(len(self.queue))]) for i in range(self.nb_tofs)]
    
    def median_filter(self) :
        """ Filter the distance with a median filter"""
        self.queue.pop(0)
        self.queue.append(self.dist)
        temp = np.array(self.queue).T
        self.dist = [np.median(temp[i]) for i in range(self.nb_tofs)]

    def eliminate_zero_values(self):
        """eliminate zero values"""
        for i in range(self.nb_tofs) :
            if self.dist[i] < 0.0001 :
                self.dist[i] = self.queue[-1][i]
    
    


if __name__ == "__main__" :

    Distance()
    # now we can use rospy.spin() to keep the node alive
    rospy.spin()