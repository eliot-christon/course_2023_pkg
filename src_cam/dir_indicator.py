#!/usr/bin/env python3
# -*- coding: utf-8 -*-


import rospy
import numpy as np

from std_msgs.msg import Int16MultiArray, String, Bool
from sensor_msgs.msg import Image as SensorImage
from cv_bridge import CvBridge, CvBridgeError
import cv2
import time


def bgr2hsv (pix, rgb=0):
    """fonction qui convertit les valeurs d'un pixel bgr en leur valeur hsv"""
    #Dans ce code on ne convertit pas l'emsemble de l'image de bgr/rgb afin de limiter le nombre de calcul, 
    # on indique juste lorqu'on recupère les valeurs hsv si le pixel d'origine est en bgr ou rgb 
    p=pix.copy()
    p=p/255

    if rgb == 1 :
        r,g,b=p[0],p[1],p[2]
    else:
        b,g,r=p[0],p[1],p[2]
    v=np.max(p)
    if v != 0:
        s=(v-np.min(p))/v
    else:
        s=0
    if b==g and g==r:
        h=0
    elif v==r:
        h=60*(g-b)/(v-np.min(p))
    elif v==g:
        h=120+60*(b-r)/(v-np.min(p))
    elif v==b:
        h=240+60*(r-g)/(v-np.min(p))
    if h < 0:
        h+=360
    v=255*v
    s=255*s
        
    return int(h),int(s),int(v)


class Dir_indicator : 

    """Ce noeud a pour but d'indiquer si le véhicule est dans le bon ou dans le mauvais sens par rapport au sens de circulation et également d'indiquer la couleur qu'il a en face de lui.

    Ce noeud publie sur deux topics:
    ->/Direction :
        Les messages publiés sont de type String()  "right" ou "wrong"
    ->/Wallcolor :
        Les messages publiés sont de type String()  "red" ou "green"
        Ce topic peut permettre de savoir dans quelle direction il faut tourner si on se retrouve face à un mur 
    """

    def __init__(self, w, h) : 
        #On initie le noeud
        rospy.init_node('dir_indicator', anonymous = True)

            #======Parameters======

        #Paramètre qui détermine si on est dans le cas d'un robot réel
        self.reelparam = rospy.get_param("reel", default=1)

        #Paramètres de valeurs de teinte
        self.max_hue_red = rospy.get_param('max_hue_red', default=20)
        self.min_hue_red = rospy.get_param('min_hue_red', default=300)
        self.max_hue_green = rospy.get_param('max_hue_green', default=150)
        self.min_hue_green = rospy.get_param('min_hue_green', default=70)

        #Paramètres de valeurs de saturation
        self.min_sat_red = rospy.get_param('min_sat_red', default=60)
        self.min_sat_green = rospy.get_param('min_sat_green', default=60)
        self.max_sat_red = rospy.get_param('max_sat_red', default=255)
        self.max_sat_green = rospy.get_param('max_sat_green', default=255)

        #Paramètres de valeurs de luminosité
        self.min_val_red = rospy.get_param('min_val_red', default=20)
        self.min_val_green = rospy.get_param('min_val_green', default=20)
        self.max_val_red = rospy.get_param('max_val_red', default=255)
        self.max_val_green = rospy.get_param('max_val_green', default=255)

        #Paramètres qui déterminent les cadres pour la vision du vehicule
        self.limite_haute=rospy.get_param('lim_haut', default=52)
        self.limite_basse=rospy.get_param('lim_bas', default=100)
        self.thick=rospy.get_param('thick', default=10)

        #Paramètre qui prend en compte si la caméra est en rgb ou en bgr
        self.rgb=rospy.get_param('rgb',default=1)

        #Paramètre qui détermine le sens de circulation
        self.left_is_green=rospy.get_param('left_is_green', default=True)


            #======Subscribers======

        #Paramètre qui détermine le nom du topic pour les données d'images
        sub_topic = rospy.get_param("image_datas", default="/ImageScan")    

        #Paramètre qui détermine le nom du topic pour le mode de sensibilité
        sub_sensi_topic= rospy.get_param("sensi_topic", default="/Sensi")

        #On crée le subscriber des données d'images dans le cas d'un robot réel ou d'une simu
        if self.reelparam == 1:
            self.sub=rospy.Subscriber(sub_topic, SensorImage, self.callback)
        else:
            self.sub=rospy.Subscriber(sub_topic, Int16MultiArray, self.callback)

        #subscriber pour le topic de sensibilité de la direction
        self.sub_sensi=rospy.Subscriber(sub_sensi_topic, Bool, self.callback_sensi)
        

            #======Publishers======

        pubdir_topic = "/Direction"
        pubwcolor_topic = "/WallColor"
        self.pubdirection=rospy.Publisher(pubdir_topic, String, queue_size=1)
        self.pubwcolor=rospy.Publisher(pubwcolor_topic, String, queue_size=1)

            #======Attributs de Dir_indicator======
        #publishers
        self.direction=String()
        self.wcolor=String()
        self.dir=Bool()

        #valeurs pour redimensionner l'image récupéré
        self.w, self.h = w, h
        self.left = np.zeros((self.h,10,3))
        self.right = np.zeros((self.h,10,3))
        self.middle = np.zeros((self.h,10,3))

        #sensibilité
        self.sensi=False

        #conversion opencv
        self.cv_bridge = CvBridge()

# CALLBACKS ==============================================================================================================

    def callback_sensi(self, msg):
        """Attribut de sensibilité"""
        self.sensi=msg.data
        
    def callback(self, msg) : 
        """Récupération des données d'images"""


        #start_time = time.time()

        if self.reelparam==0 :
            scan = msg.data		
            image= np.array(scan).reshape((self.h, self.w, 4))[:,:,0:3]

        else :
            image = self.cv_bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")[:,:,0:3]

        #On redimensionne l'image pour avoir moins de pixels à traiter
        img = cv2.resize(image.astype('float32'), (75, 60), interpolation=cv2.INTER_LINEAR).astype('int')

        #Dans ce code on ne convertit pas l'ensemble de l'image de bgr/rgb afin de limiter le nombre de calcul 

        #On récupère trois rectangles à gauche, à droite et au centre de l'image définis par limite_haute, limite_basse et thick
        self.left =img[self.limite_haute:self.limite_basse,0:self.thick,0:3]
        self.right = img[self.limite_haute:self.limite_basse,img.shape[1]-self.thick:img.shape[1],0:3]
        self.middle = img[self.limite_haute:self.limite_basse,int(img.shape[1]//2-(self.thick/2)):int(img.shape[1]//2+(self.thick/2)),0:3]

        # end_time = time.time()  
        # print("Temps d'exécution du callback: {} secondes".format(end_time - start_time))
        #Temps d'exécution à partir d'un bag : 2.3e-4 s  , négligeable devant une boucle du run


# Algo ==============================================================================================================
    def run(self):
        """Fonction qui effectue les opérations sur les données d'image et qui en déduit la direction et la couleur du mur face au robot"""
        

        rate = rospy.Rate(10)

        while not rospy.is_shutdown() :
            #loop_start_time = time.time() 

            count_green_left=0
            count_red_left=0
            for i in self.left:
                for p in i:
                    h,s,v=bgr2hsv (p, self.rgb)
                    #On compte le nombre de pixel vert à l'aide de leur valeur hsv
                    if h < self.max_hue_green and h > self.min_hue_green:
                        if s>self.min_sat_green and v>self.min_val_green:
                            count_green_left+=1

                    #On compte le nombre de pixel rouge selon leur valeur hsv
                    if h < self.max_hue_red or h > self.min_hue_red:
                        if s>self.min_sat_red and v>self.min_val_red:
                            count_red_left+=1
        
            count_green_right=0
            count_red_right=0
            for i in self.right:
                for p in i:
                    h,s,v=bgr2hsv (p, self.rgb)
                    #On compte le nombre de pixel vert à l'aide de leur valeur hsv
                    if h < self.max_hue_green and h > self.min_hue_green:
                        if s>self.min_sat_green and v>self.min_val_green:
                            count_green_right+=1

                    #On compte le nombre de pixel rouge selon leur valeur hsv
                    if h < self.max_hue_red or h > self.min_hue_red:
                        if s>self.min_sat_red and v>self.min_val_red:
                            count_red_right+=1

            count_green_middle=0
            count_red_middle=0
            for i in self.middle:
                for p in i:
                    h,s,v=bgr2hsv (p, self.rgb)
                    #On compte le nombre de pixel vert à l'aide de leur valeur hsv
                    if h < self.max_hue_green and h > self.min_hue_green:
                        if s>self.min_sat_green and v>self.min_val_green:
                            count_green_middle+=1

                    #On compte le nombre de pixel rouge selon leur valeur hsv
                    if h < self.max_hue_red or h > self.min_hue_red:
                        if s>self.min_sat_red and v>self.min_val_red:
                            count_red_middle+=1

            #Publication par défaut
            self.wcolor.data="???"
            self.direction.data="???"

            #On passe d'une sensibilité à l'autre en fonction de self.sensi
            if self.sensi:
                #On détermine la direction prise par le véhicule
                if ((count_green_right < 0.75*count_green_left) or (count_red_left < 0.75*count_red_right)) :
                    self.direction.data="wrong"

                elif (0.75*count_green_right > count_green_left) or (0.75*count_red_left > count_red_right):
                    self.direction.data="right"

                else:
                    self.direction.data="???"

            else:
                #On détermine la direction prise par le véhicule
                if (count_red_right > 5 and count_green_left > 5):
                    self.direction.data="wrong"

                elif (count_green_right > 5 and count_red_left > 5):
                    self.direction.data="right"

                else:
                    self.direction.data="???"

            #On détermine la couleur en face du véhicule
            if (count_green_middle > count_red_middle) and count_green_middle > 5:
                self.wcolor.data="green"
            
            elif (count_red_middle > count_green_middle) and count_red_middle > 5:
                self.wcolor.data="red"

            else:
                self.wcolor.data="???"

            #On inverse le sens déduit en fonction du sens de circulation
            if self.left_is_green:
                if self.direction.data=="wrong":
                    self.direction.data="right"
                elif self.direction.data=="right":
                    self.direction.data="wrong"

            if self.direction.data=="wrong" :
                self.dir.data = False
            else:
                self.dir.data = True

            self.pubdirection.publish(self.direction)
            self.pubwcolor.publish(self.wcolor)

            #print(count_green_left,count_red_left,count_green_middle,count_red_middle,count_green_right,count_red_right)

            # loop_end_time = time.time() 
            # print("Temps d'exécution d'une boucle du run: {} secondes".format(loop_end_time - loop_start_time))

            #Temps de calcul de la loop : 5 ms ce qui est mieux que l'an dernier

            rate.sleep()


if __name__ == '__main__':
    w = rospy.get_param("image_width", default=640)
    h = rospy.get_param("image_height", default=480)
    try:
        d = Dir_indicator(w, h) 
        d.run()
        rospy.spin()

    except rospy.ROSInterruptException:
        pass