#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Projet PFE : Voiture autonome
@Authors : Julien JOYET
"""

import rospy

from std_msgs.msg import Float32MultiArray, Float32, Int8, Bool, String

class MAE:
    """Ce noeud fait fonctionner une machine à état à 2 états:
        -Etat 0: Navigation lidar
        -Etat 1: Marche_arrière
        
    La MAE prend en entrée:
        /Obstacle_warning
        /Freepath
        
    Les sorties de la MAE sont:
        /Marche_arriere
        /Nav_lid
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
        self.ow=False
        self.dist_lim=False
        self.fp=True
        #Sorties
        self.marche_arr=False
        self.nav_lid=True

            #======Publishers======
        self.pub_nav_lid = rospy.Publisher("/Nav_lid", Bool, queue_size = 1)
        self.pub_marche_arr = rospy.Publisher("/Marche_arriere", Bool, queue_size = 1)
        self.pub_state = rospy.Publisher("/State", Int8, queue_size = 1)

            #======Subscribers======
        self.sub_ow = rospy.Subscriber("/Obstacle_warning", Bool, self.callback_ow)
        self.sub_fp = rospy.Subscriber("/Freepath", Bool, self.callback_fp)

# CALLBACKS ==============================================================================================================

    def callback_ow(self, msg) :
        self.ow=msg.data

    def callback_fp(self, msg) :
        self.fp=msg.data

# CALCUL ETAT FUTUR ======================================================================================================

    def f_state(self):

        if self.EP == 0:
            #Transition d'état marche_arrière si il y a un obstacle devant et si la voie n'est pas libre
            if self.ow and not(self.fp):
                self.EF=1

            #Sinon on reste dans le même état
            else:
                self.EF=0

        if self.EP == 1:
            #Si la voie est libre on repasse en navigation lidar
            if self.fp:
                self.EF=0

            #Sinon on reste dans le même état
            else:
                self.EF = 1






# CALCUL DES SORTIES ======================================================================================================

    def s_MAE(self):

        self.marche_arr=False
        self.nav_lid=False
        
        if self.EP == 0:
            #On active la navigation lidar
            self.nav_lid = True

        if self.EP == 1:
            #On active la marche arrière
            self.marche_arr = True

        
 # PUBLICATION DES SORTIES ======================================================================================================

    def pub(self):
        self.pub_marche_arr.publish(self.marche_arr)
        self.pub_nav_lid.publish(self.nav_lid)

# Algo ==============================================================================================================

    def run(self) :
        """ Main loop of the navigation"""

        # fréquence de laMAE
        HZ = 20 # Hz
        rate = rospy.Rate(HZ)

        # main loop
        while not rospy.is_shutdown() :
            #On calcule l'état futur
            self.f_state()
            #On actualise l'état présent
            self.EP=self.EF
            #On publie la l'état
            self.pub_state.publish(self.EP)
            #On calcule les sorties
            self.s_MAE()
            #On publie les sorties
            self.pub()
            rate.sleep()
        

if __name__ == "__main__" :
    
        nav = MAE()
        nav.run()
        rospy.spin()

