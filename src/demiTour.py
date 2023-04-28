#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
import numpy as np
from std_msgs.msg import Float32MultiArray, String, Bool

#  class qui permet de faire demi-tour


class DemiTour:

    #  On initialise les variables
    def __init__(self):
        self.run = False
        self.right, self.left = False, False
        self.rear_obstacle = False
        self.front_obstacle = False
        self.tof_sensi = 0.5
        self.count_unknown = 0
        self.lidar = []

        rospy.init_node("demi_tour")  #  On initialise le node

        self.max_speed = rospy.get_param(
            "max_speed", default=0.25)  #  On récupère la vitesse max

        # On récupère les angles pour le lidar
        self.a0, self.a1 = rospy.get_param("~angle0", default=45), rospy.get_param(
                "~angle1", default=315)

        #  On créee le publisher pour envoyer la vitesse et l'angle
        self.pub = rospy.Publisher(
            "/d_tourSpeedAngleCommand", Float32MultiArray, queue_size=1)
        #  On créee le publisher pour envoyer le statut du demi-tour
        self.pub_fin_dt = rospy.Publisher("/Fin_d_tour", Bool, queue_size=1)
        #  On créee le subscriber pour récupérer l'état de la MAE
        rospy.Subscriber("/D_tour", Bool, self.callback_dtour)
        #  On créee le subscriber pour récupérer les données du lidar
        rospy.Subscriber("/front_data", Float32MultiArray, self.callback_lidar)
        #  On créee le subscriber pour récupérer les données des tofs
        rospy.Subscriber("/TofsDistance", Float32MultiArray,
                         self.callback_tofs)
        # On créee le subscriber pour récupérer la direction
        rospy.Subscriber("/Direction", String, self.callback_dir)

# ===================== Callbacks =====================

    #  On récupère l'état de la MAE
    def callback_dtour(self, msg):
        self.run = msg.data
        if not (self.run):
            self.reset()
        else:
            self.pub_fin_dt.publish(False)

    #  On récupère la direction dans laquelle on doit faire le demi-tour
    def callback_lidar(self, lidar):
        self.lidar = lidar.data

    # position des tofs [front left, front right, rear left, rear right]
    def callback_tofs(self, tofs):
        #  On vérifie si il y a un obstacle derrière
        if (self.run):
            if (0 < tofs.data[0] < self.tof_sensi or 0 < tofs.data[1] < self.tof_sensi):
                self.rear_obstacle = True
            else:
                self.rear_obstacle = False
            if (0 < tofs.data[2] < self.tof_sensi or 0 < tofs.data[3] < self.tof_sensi):
                self.front_obstacle = True
            else:
                self.front_obstacle = False

    #  On récupère la direction
    def callback_dir(self, dir):
        #  On compte le nombre de fois où on a "???" pour savoir si on est bloqué
        #  Décommenter si la caméra n'est pas utilisée
        """ if dir.data == "???" and self.rear_obstacle:
            self.count_unknown += 1
        if self.count_unknown >= 40:
            self.reset() """
        if dir.data == "right":
            self.reset()

# ===================== Fonctions =====================

    #  On reset les variables
    def reset(self):
        self.run = False
        self.pub_fin_dt.publish(True)
        self.right, self.left = False, False
        self.rear_obstacle = False
        self.count_unknown, self.count_maneuver = 0, 0
        self.pub.publish(Float32MultiArray(data=[0, 0]))

    #  On traite les données du lidar
    def process_lidar(self):
        # On calcul la moyenne des valeurs du lidar à gauche entre 22.5° et 157.5°  et à droite entre 202.5° et 337.5° pour la simulation
        #  On prend évidemment pas en compte les "inf" qui signifient que le lidar est trop proche ou trop loin
        #  pour la simu il faut prendre lidar.data[50:351] pour la gauche et lidar.data[450:751] pour la droite

        # On effectue le traitement seulement si on veut faire le demi-tour
        angles = np.linspace(self.a0, self.a1, len(self.lidar))
        if len(self.lidar) != 0:
            a135 = np.where(angles >= 135)[0][0]
            a225 = np.where(angles >= 225)[0][0]

            left_lidar = np.array(self.lidar[:a135]).mean()
            right_lidar = np.array(self.lidar[a225:]).mean()

            #  On effectue le demi-tour dans la direction où il y a le plus d'espace
            if right_lidar < left_lidar:
                self.left = True
            elif right_lidar >= left_lidar:
                self.right = True

    #  On fait le demi-tour
    def run_node(self):
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            
            if self.run:
                self.process_lidar()
                if self.left:
                    starting_direction = -1
                elif self.right:
                    starting_direction = 1
                else:
                    starting_direction = 0

                if self.right or self.left or self.front_obstacle:
                    self.pub.publish(Float32MultiArray(
                        data=[-self.max_speed, starting_direction]))
                elif self.rear_obstacle:
                    self.pub.publish(Float32MultiArray(
                        data=[self.max_speed, -starting_direction]))
            rate.sleep()


if __name__ == '__main__':
    demi_tour = DemiTour()
    demi_tour.run_node()
    rospy.spin()
