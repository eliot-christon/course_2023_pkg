#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Projet PFE : Voiture autonome
@Authors : Julien JOYET
"""

import rospy

from std_msgs.msg import Float32MultiArray, Float32, Int8, Bool, String

class MAE:
    def __init__(self, EP=0, ONLY_LIDAR=False, ONLY_TOFS=False):

        #états
        self.EP=EP
        self.EF=EP

        #entrées
        self.ow=False
        self.dist_lim=False
        self.fp=True
        self.dir="right"
        self.fin_d_tour=False
        

        #Sorties
        self.marche_arr=False
        self.nav_lid=True
        self.d_tour=False
        self.sensi=False

        # Init ROS node
        rospy.init_node('MAE', anonymous=True)

        # publisher sortie MAE

        self.pub_nav_lid = rospy.Publisher("/Nav_lid", Bool, queue_size = 1)
        self.pub_nav_tofs = rospy.Publisher("/Nav_tofs", Bool, queue_size = 1)
        self.pub_d_tour = rospy.Publisher("/D_tour", Bool, queue_size = 1)
        self.pub_sensi= rospy.Publisher("/Sensi", Bool, queue_size = 1)
        self.pub_state = rospy.Publisher("/State", Int8, queue_size = 1)

        # subscribers entrées MAE
        self.sub_dist_lim = rospy.Subscriber("/Dist_lim", Bool, self.callback_dist_lim)
        self.sub_dir = rospy.Subscriber("/Direction", String, self.callback_dir)
        self.sub_fin_d_tour = rospy.Subscriber("/Fin_d_tour", Bool, self.callback_fin_d_tour)
        


        


        
    #Methodes qui mettent à jour les entrées
    def callback_dist_lim(self, msg) :
        self.dist_lim=msg.data

    def callback_dir(self, msg) :
        self.dir=msg.data 

    def callback_fin_d_tour(self, msg):
        self.fin_d_tour=msg.data 




    #calcul de l'état future
    def f_state(self):

        if self.EP == 0:

            if self.dir=="wrong":
               self.EF=2

            elif self.dist_lim:
                self.EF=1

            else:
                self.EF=0

        if self.EP == 1:

            if self.dir=="wrong":
                self.EF=2

            elif not(self.dist_lim):
                self.EF=0

            else:
                self.EF = 1

        if self.EP == 2:
            if (self.dir=="right" or self.fin_d_tour):
                self.EF=1
            else:
                self.EF=2 

    #Calcul des sorties de la MAE
    def s_MAE(self):

        self.nav_tofs=False
        self.nav_lid=False
        self.d_tour=False
        self.sensi=False 
        
        if self.EP == 0:
            self.nav_lid = True

        if self.EP == 1:
            self.nav_tofs = True

        if self.EP == 2:
            self.d_tour = True
            self.sensi = True
 
    def pub(self):
        self.pub_nav_tofs.publish(self.nav_tofs)
        self.pub_nav_lid.publish(self.nav_lid)
        self.pub_d_tour.publish(self.d_tour)
        self.pub_sensi.publish(self.sensi) 

    

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


