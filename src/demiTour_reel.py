#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
import numpy as np
from std_msgs.msg import Float32, Float32MultiArray, String, Bool

# Permet de déterminer le sens de départ du demi-tour
def callback_lidar(lidar):
    global right, left

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
            left = True
        elif right_lidar >= left_lidar:
            right = True

# position des tofs [front left, front right, rear left, rear right]
def callback_tofs(tofs):
    global front_obstacle, rear_obstacle

    rear_sensi = 0.5
    if(0<tofs.data[2]<rear_sensi or 0<tofs.data[3]<rear_sensi):
        front_obstacle = False
        rear_obstacle = True

# Permet de changer le flag de direction de demi-tour si l'on se trouve dans le mauvais sens
def callback_dir(direction):
    global run

    if direction.data == "wrong":
        run = True
        pub_fin_dt.publish(False)
    elif direction.data == "right":
        pub_fin_dt.publish(True)
        run = False

if __name__ == '__main__':

    run = False
    right, left = False, False
    wall_close = False
    front_obstacle, rear_obstacle = False, False

    rospy.init_node("demi_tour")

    pub = rospy.Publisher("/d_tourSpeedAngleCommand", Float32MultiArray, queue_size=1)
    pub_fin_dt = rospy.Publisher("/Fin_d_tour", Bool, queue_size=1)
    #rospy.Subscriber("/LidarScan", Float32MultiArray, callback_lidar)
    rospy.Subscriber("/front_data",Float32MultiArray,callback_lidar)
    rospy.Subscriber("/TofsDistance", Float32MultiArray, callback_tofs)
    rospy.Subscriber("/Direction", String, callback_dir)
    rate = rospy.Rate(20)
    #time.sleep(5)

    while not (left or right or rospy.is_shutdown()):
        pass

    while not rospy.is_shutdown():
        try:    
            
            max_speed = rospy.get_param("max_speed", default=0.7)

            # On vérifie run à chaque fois car run peut changer à tout moment
            if left:
                starting_direction = -1
            else:
                starting_direction = 1

            ### Temporaire car normalement système de navigation prend le relais mais pour les test non
            # if not run:
            #     velocity_msg = Float32()
            #     angular_msg = Float32()

            #     speed_pub.publish(velocity_msg)
            #     angle_pub.publish(angular_msg)
            
            while run and not rospy.is_shutdown():
                command=Float32MultiArray()
                if not (rear_obstacle or front_obstacle):
                    velocity = -max_speed
                    angular = starting_direction
                elif rear_obstacle:
                    velocity = max_speed
                    angular = -starting_direction
                elif front_obstacle:
                    velocity = -max_speed
                    angular = starting_direction

                command.data=[velocity,angular]
                pub.publish(command)
                rate.sleep()
            rate.sleep()

        except rospy.ROSInterruptException:
            break
        except KeyboardInterrupt:
            print("shuting down...")
            break
        except NameError:
            break
