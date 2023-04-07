#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Projet PFE : Voiture autonome
@Authors : Julien JOYET
"""

import rospy

from std_msgs.msg import Float32MultiArray, Float32, Int8, Bool

class MAE:
    def __init__(self, EP=0, ONLY_LIDAR=False, ONLY_TOFS=False):

        #états
        self.EP=EP
        self.EF=EP

        #entrées
        self.dist_lim=False
        self.fin_d_tour=False
        self.dir=True

        #Sorties
        self.nav_tof=False
        self.nav_lid=True
        self.d_tour=False
        self.sensi=False

        # Init ROS node
        rospy.init_node('MAE', anonymous=True)

        # publisher sortie MAE
        self.pub_nav_tof = rospy.Publisher("/Nav_tof", Bool, queue_size = 1)
        self.pub_nav_lid = rospy.Publisher("/Nav_lid", Bool, queue_size = 1)
        self.pub_d_tour = rospy.Publisher("/D_tour", Bool, queue_size = 1)
        self.pub_sensi= rospy.Publisher("/Sensi", Bool, queue_size = 1)

        self.pub_state = rospy.Publisher("/State", Int8, queue_size = 1)

        # subscribers entrées MAE
        self.sub_dist_lim = rospy.Subscriber("/Dist_lim", Bool, self.callback_dist_lim)
        self.sub_fin_d_tour = rospy.Subscriber("/Fin_d_tour", Bool, self.callback_fin_d_tour)
        self.sub_dir = rospy.Subscriber("/Dir", Bool, self.callback_direction)

        
    #Methodes qui mettent à jour les entrées
    def callback_dist_lim(self, msg) :
        self.dist_lim=msg.data

    def callback_fin_d_tour(self, msg) :
        self.fin_d_tour=msg.data

    def callback_direction(self, msg) :
        self.dir=msg.data


    #calcul de l'état future
    def f_state(self):

        if self.EP == 0:

            if self.dist_lim and self.dir:
 
                self.EF=1
            elif not(self.dir):

                self.EF=2
            else:
                self.EF=0

        if self.EP == 1:
            if not(self.dist_lim) and self.dir:
                self.EF=0
                
            elif not(self.dir):
                self.EF=2
            else:
                self.EF == 1

        if self.EP == 2:
            if self.fin_d_tour:
                self.EF=1
            else:
                self.EF=2


    #Calcul des sorties de la MAE
    def s_MAE(self):

        self.nav_tof=False
        self.nav_lid=False
        self.d_tour=False
        self.sensi=False
        
        if self.EP == 0:
            self.nav_lid = True

        if self.EP == 1:
            self.nav_tof = True

        if self.EP == 2:
            self.d_tour = True
            self.sensi = True

    def pub(self):
        self.pub_nav_tof.publish(self.nav_tof)
        self.pub_nav_lid.publish(self.nav_lid)
        self.pub_d_tour.publish(self.d_tour)
        self.pub_sensi.publish(self.sensi)

    

    def run(self) :
        """ Main loop of the navigation"""

        # fréquence de laMAE
        HZ = 20 # Hz
        rate = rospy.Rate(HZ)

        # main loop
        while not rospy.is_shutdown() :
            #On calcul l'état futur
            self.f_state()
            #On actualise l'état présent
            self.EP=self.EF
            #On publie la l'état
            self.pub_state.publish(self.EP)
            #On calcul les sorties
            self.s_MAE()
            #On publie les sorties
            self.pub()
            rate.sleep()
        

if __name__ == "__main__" :
    
        nav = MAE()
        nav.run()

        rospy.spin()


