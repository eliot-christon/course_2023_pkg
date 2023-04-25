#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Projet PFE : Voiture autonome
@Authors : Julien JOYET
"""

import rospy

from std_msgs.msg import Float32MultiArray, Float32, Int8, Bool, String

class MAE:
    """Ce noeud fait fonctionner une machine à état à 3 états:
        -Etat 0: Navigation lidar
        -Etat 1: Navigation Tofs
        -Etat 2: Demi-tour
        
    La MAE prend en entrée:
        /Dist_lim
        /Direction
        /Fin_d_tour
        
    Les sorties de la MAE sont:
        /Nav_lid
        /Nav_tofs
        /D_tour
        /Sensi
        /State
        
    Les entrées de la MAE vont déterminer l'état de la voiture et les sorties sont des commandes pour les autres noeuds"""

    def __init__(self, EP=0, ONLY_LIDAR=False, ONLY_TOFS=False):

        # Init ROS node
        rospy.init_node('MAE', anonymous=True)

            #======Attributs======
        #Etat présent
        self.EP=EP
        #Etat futur
        self.EF=EP
        #entrées
        self.dist_lim=False
        self.dir="right"
        self.fin_d_tour=False
        #Sorties
        self.marche_arr=False
        self.nav_lid=True
        self.d_tour=False
        self.sensi=False

            #======Publishers======
        self.pub_nav_lid = rospy.Publisher("/Nav_lid", Bool, queue_size = 1)
        self.pub_nav_tofs = rospy.Publisher("/Nav_tofs", Bool, queue_size = 1)
        self.pub_d_tour = rospy.Publisher("/D_tour", Bool, queue_size = 1)
        self.pub_sensi= rospy.Publisher("/Sensi", Bool, queue_size = 1)
        self.pub_state = rospy.Publisher("/State", Int8, queue_size = 1)

            #======Subscribers======
        self.sub_dist_lim = rospy.Subscriber("/Dist_lim", Bool, self.callback_dist_lim)
        self.sub_dir = rospy.Subscriber("/Direction", String, self.callback_dir)
        self.sub_fin_d_tour = rospy.Subscriber("/Fin_d_tour", Bool, self.callback_fin_d_tour)
        
# CALLBACKS ==============================================================================================================

    def callback_dist_lim(self, msg) :
        self.dist_lim=msg.data

    def callback_dir(self, msg) :
        self.dir=msg.data 

    def callback_fin_d_tour(self, msg):
        self.fin_d_tour=msg.data 

# CALCUL ETAT FUTUR ======================================================================================================

    def f_state(self):

        if self.EP == 0:
            #Transition d'état demi_tour si mauvaise direction
            if self.dir=="wrong":
               self.EF=2

            #Transition d'état Nav_tofs si proche d'un obstacle
            elif self.dist_lim:
                self.EF=1

            #Sinon on reste dans le même état
            else:
                self.EF=0

        if self.EP == 1:
            #Transition d'état demi_tour si mauvaise direction
            if self.dir=="wrong":
                self.EF=2

            #Transition d'état Nav_lidar si loin d'obstacles
            elif not(self.dist_lim):
                self.EF=0

            #Sinon on reste dans le même état
            else:
                self.EF = 1

        if self.EP == 2:
            #Transition d'état Nav_tofs si bonne direction, on passe d'abord par l'état Nav_tofs avant de repasser à la navigation lidar 
            if (self.dir=="right" or self.fin_d_tour):
                self.EF=1
            
            #Sinon on reste dans le même état
            else:
                self.EF=2 

# CALCUL DES SORTIES ======================================================================================================

    def s_MAE(self):

        self.nav_tofs=False
        self.nav_lid=False
        self.d_tour=False
        self.sensi=False 
        
        if self.EP == 0:
            #On active la navigation lidar
            self.nav_lid = True

        if self.EP == 1:
            #On active la navigation Tofs
            self.nav_tofs = True

        if self.EP == 2:
            #On active le demi_tour
            self.d_tour = True
            #On active une plus grande sensibilité pour l'indicateur de direction
            self.sensi = True
 
 # PUBLICATION DES SORTIES ======================================================================================================

    def pub(self):
        self.pub_nav_tofs.publish(self.nav_tofs)
        self.pub_nav_lid.publish(self.nav_lid)
        self.pub_d_tour.publish(self.d_tour)
        self.pub_sensi.publish(self.sensi) 

# Algo ==============================================================================================================

    def run(self) :
        """ Main loop of the navigation"""

        # fréquence de la MAE
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


