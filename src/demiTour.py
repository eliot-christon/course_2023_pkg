#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
import numpy as np
from std_msgs.msg import Float32, Float32MultiArray, String

# Permet de déterminer le sens de départ du demi-tour
def callback_lidar(lidar):
    global right, left

    # On calcul la moyenne des valeurs du lidar à gauche entre 22.5° et 157.5°  et à droite entre 202.5° et 337.5° pour la simulation
    # On prend évidemment pas en compte les "inf" qui signifient que le lidar est trop proche ou trop loin
    # pour la simu il faut prendre lidar.data[50:351] pour la gauche et lidar.data[450:751] pour la droite
    left_lidar = np.ma.masked_invalid(lidar.data[50:351]).mean()
    right_lidar = np.ma.masked_invalid(lidar.data[450:751]).mean()
    
    # On effectue le demi-tour dans la direction où il y a le plus d'espace
    if right_lidar < left_lidar:
        left = True
    elif right_lidar >= left_lidar:
        right = True

# position des tofs [front left, front right, rear left, rear right]
def callback_tofs(tofs):
    global front_obstacle, rear_obstacle
    # simu publie à 10Hz donc sensi relativement haute
    # à modifier avec le robot réel !
    print(tofs.data)
    front_sensi = 0.7
    rear_sensi = 0.5
    if(0<tofs.data[0]<front_sensi or 0<tofs.data[1]<front_sensi):
        rear_obstacle = False
        front_obstacle = True
    if(0<tofs.data[2]<rear_sensi or 0<tofs.data[3]<rear_sensi):
        front_obstacle = False
        rear_obstacle = True

# Permet de changer le flag de direction de demi-tour si l'on se trouve dans le mauvais sens
def callback_dir(direction):
    global run

    if direction.data == "wrong":
        run = True
    else:
        run = False

if __name__ == '__main__':

    run = False
    right, left = False, False
    wall_close = False
    front_obstacle, rear_obstacle = False, False

    rospy.init_node("vroum")

    pub = rospy.Publisher("/SpeedCommand", Float32, queue_size=10)
    pub2 = rospy.Publisher("/AngleCommand", Float32, queue_size=10)
    rospy.Subscriber("/LidarScan", Float32MultiArray, callback_lidar)
    rospy.Subscriber("/SensorsScan", Float32MultiArray, callback_tofs)
    rospy.Subscriber("/Direction", String, callback_dir)
    rate = rospy.Rate(5)
    #time.sleep(5)

    while not (left or right or rospy.is_shutdown()):
        pass

    while not rospy.is_shutdown():
        try:    
            
            # On vérifie run à chaque fois car run peut changer à tout moment
            if left:
                starting_direction = -1
            else:
                starting_direction = 1

            ### Temporaire car normalement système de navigation prend le relais mais pour les test non
            if not run:
                velocity_msg = Float32()
                angular_msg = Float32()

                pub.publish(velocity_msg)
                pub2.publish(angular_msg)

            while run and not rospy.is_shutdown():
                velocity_msg = Float32()
                angular_msg = Float32()
                if not (rear_obstacle or front_obstacle):
                    velocity_msg.data = -1
                    angular_msg.data = starting_direction
                elif rear_obstacle:
                    velocity_msg.data = 1
                    angular_msg.data = -starting_direction
                elif front_obstacle:
                    velocity_msg.data = -1
                    angular_msg.data = starting_direction

                pub.publish(velocity_msg)
                pub2.publish(angular_msg)

            rate.sleep()
        except rospy.ROSInterruptException:
            break
        except KeyboardInterrupt:
            print("shuting down...")
            break
        except NameError:
            break
