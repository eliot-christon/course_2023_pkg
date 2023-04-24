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
        

        #Sorties
        self.marche_arr=False
        self.nav_lid=True

        # Init ROS node
        rospy.init_node('MAE', anonymous=True)

        # publisher sortie MAE

        self.pub_nav_lid = rospy.Publisher("/Nav_lid", Bool, queue_size = 1)
        self.pub_marche_arr = rospy.Publisher("/Marche_arriere", Bool, queue_size = 1)

        self.pub_state = rospy.Publisher("/State", Int8, queue_size = 1)

        # subscribers entrées MAE
        self.sub_ow = rospy.Subscriber("/Obstacle_warning", Bool, self.callback_ow)
        self.sub_fp = rospy.Subscriber("/Freepath", Bool, self.callback_fp)


        
    #Methodes qui mettent à jour les entrées
    def callback_ow(self, msg) :
        self.ow=msg.data

    def callback_fp(self, msg) :
        self.fp=msg.data




    #calcul de l'état future
    def f_state(self):

        if self.EP == 0:

            if self.ow and not(self.fp):
                self.EF=1
            else:
                self.EF=0

        if self.EP == 1:
            if self.fp:
                self.EF=0
            else:
                self.EF = 1






    #Calcul des sorties de la MAE
    def s_MAE(self):

        self.marche_arr=False
        self.nav_lid=False
        
        if self.EP == 0:
            self.nav_lid = True

        if self.EP == 1:
            self.marche_arr = True

        

    def pub(self):
        self.pub_marche_arr.publish(self.marche_arr)
        self.pub_nav_lid.publish(self.nav_lid)


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
            #print(f"E:{self.EP} ow={self.ow} dist_lim={self.dist_lim} fp={self.fp} dir={self.dir} self.fin_d_tour={self.fin_d_tour}")

            rate.sleep()
        

if __name__ == "__main__" :
    
        nav = MAE()
        nav.run()

        rospy.spin()


