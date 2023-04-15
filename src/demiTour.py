#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
import numpy as np
from std_msgs.msg import Float32, Float32MultiArray, String, Bool

class DemiTour:
    def __init__(self):
        self.run = False
        self.right, self.left = False, False
        self.rear_obstacle = False
        self.count_unknown = 0

        rospy.init_node("demi_tour")

        self.max_speed = rospy.get_param("max_speed", default=0.25)

        self.pub = rospy.Publisher("/d_tourSpeedAngleCommand", Float32MultiArray, queue_size=1)
        self.pub_fin_dt = rospy.Publisher("/Fin_d_tour", Bool, queue_size=1)
        rospy.Subscriber("/D_tour", Bool, self.callback_dtour)
        rospy.Subscriber("/front_data", Float32MultiArray, self.callback_lidar)
        rospy.Subscriber("/TofsDistance", Float32MultiArray, self.callback_tofs)
        rospy.Subscriber("/Direction", String, self.callback_dir)

    def callback_dtour(self,msg):
        self.run=msg.data
        if not(self.run):
            self.reset()

    # On récupère la direction dans laquelle on doit faire le demi-tour
    def callback_lidar(self, lidar):
        # On calcul la moyenne des valeurs du lidar à gauche entre 22.5° et 157.5°  et à droite entre 202.5° et 337.5° pour la simulation
        # On prend évidemment pas en compte les "inf" qui signifient que le lidar est trop proche ou trop loin
        # pour la simu il faut prendre lidar.data[50:351] pour la gauche et lidar.data[450:751] pour la droite
        a0,a1=rospy.get_param("~angle0",default=45),rospy.get_param("~angle1",default=315)
        angles=np.linspace(a0,a1,len(lidar.data))
        if len(lidar.data)!=0:
            a135=np.where(angles>=135)[0][0]
            a225=np.where(angles>=225)[0][0]

            left_lidar = np.array(lidar.data[:a135]).mean()
            right_lidar = np.array(lidar.data[a225:]).mean()
            
            # On effectue le demi-tour dans la direction où il y a le plus d'espace
            if right_lidar < left_lidar:
                self.left = True
            elif right_lidar >= left_lidar:
                self.right = True

    # position des tofs [front left, front right, rear left, rear right]
    def callback_tofs(self, tofs):
        rear_sensi = 0.5
        if(0<tofs.data[0]<rear_sensi or 0<tofs.data[1]<rear_sensi):
            self.rear_obstacle = True

    def reset(self):
        self.run = False
        self.right, self.left = False, False
        self.rear_obstacle = False
        self.count_unknown, self.count_maneuver = 0, 0
        self.pub.publish(Float32MultiArray(data=[0, 0]))

    def callback_dir(self, dir):
        if dir.data == "???" and self.rear_obstacle:
            self.count_unknown += 1
        if self.count_unknown >= 40:
            self.reset()

    def run_node(self):
        while not rospy.is_shutdown():
            if self.run:
                if self.left:
                    starting_direction = -1
                elif self.right:
                    starting_direction = 1
                else:
                    starting_direction = 0

                if self.right or self.left:
                    self.pub.publish(Float32MultiArray(data=[-self.max_speed, starting_direction]))
                elif self.rear_obstacle:
                    self.pub.publish(Float32MultiArray(data=[self.max_speed, -starting_direction]))
                self.pub_fin_dt.publish(False)
            else:
                self.pub_fin_dt.publish(True)
            

if __name__ == '__main__':
    demi_tour = DemiTour()
    demi_tour.run_node()
    rospy.spin()