#!/usr/bin/env python3
# -*- coding: utf-8 -*-


import rospy
import numpy as np

from std_msgs.msg import Int16MultiArray, String, Bool
from sensor_msgs.msg import Image as SensorImage
from cv_bridge import CvBridge, CvBridgeError
import cv2

def bgr2hsv (colonne, rgb=0):
    """fonction qui convertit les valeurs d'une colonne de pixel bgr en leur valeur hsv"""
    colonne=colonne/255
    hsv=np.zeros(colonne.shape)
    for i,p in enumerate(colonne):
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
        hsv[i]=[h,s,v]
        
    return hsv


class Dir_indicator : 

    """Ce noeud a pour but d'indiquer si le véhicule est dans le bon ou dans le mauvais sens par rapport au circuit et également d'indiquer la couleur qu'il a en face de lui.
Ce noeud publie sur deux topics:
    ->/Direction :
        Les messages publiés sont de type String()  "right" ou "wrong"
    ->/Wallcolor :
        Les messages publiés sont de type String()  "red" ou "green"
        Ce topic peut permettre de savoir dans quelle direction il faut tourner si on se retrouve face à un mur
    """

    def __init__(self, w, h) : 

        rospy.init_node('dir_indicator', anonymous = True)

        #subscriber
        sub_topic = rospy.get_param("image_datas", default="/ImageScan")
        sub_sensi_topic= rospy.get_param("sensi_topic", default="/Sensi")
        reelparam = rospy.get_param("reel", default="1")
        if reelparam == 1:
            self.sub=rospy.Subscriber(sub_topic, SensorImage, self.callback)
        else:
            self.sub=rospy.Subscriber(sub_topic, Int16MultiArray, self.callback)

        #sub pour le topic de sensibilité de la direction
        self.sub_sensi=rospy.Subscriber(sub_sensi_topic, Bool, self.callback_sensi)
        
        
        

        #publisher
        pubdir_topic = "/Direction"
        pubwcolor_topic = "/WallColor"
        self.pubdirection=rospy.Publisher(pubdir_topic, String, queue_size=1)
        self.pubwcolor=rospy.Publisher(pubwcolor_topic, String, queue_size=1)
        self.direction=String()
        self.wcolor=String()

        #Entrée de MAE
        self.dir=Bool()
        self.pubdir=rospy.Publisher("/Dir",Bool,queue_size=1)

        #valeurs pour redimensionner l'image récupéré
        self.w, self.h = w, h
        self.lefthsv = np.zeros((self.h,3))
        self.righthsv = np.zeros((self.h,3))
        self.middlehsv = np.zeros((self.h,3))

        #sensibilité
        self.sensi=False

        self.cv_bridge = CvBridge()

    def callback_sensi(self, msg):
        self.sensi=msg.data
        
    def callback(self, msg) : 
        limite_haute=rospy.get_param('lim_haut', default=480)
        limite_basse=rospy.get_param('lim_bas', default=0)

        # scan = self.cv_bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")
        # print("yes")
        #On récupère deux lignes verticales à gauche et à droite
        scan = msg.data
        if len(scan)>0:	
            leftscan = np.array(scan).reshape((self.h, self.w, 4))[limite_haute:limite_basse,10,0:3]
            rightscan = np.array(scan).reshape((self.h, self.w, 4))[limite_haute:limite_basse,self.w-10,0:3]
            middlescan = np.array(scan).reshape((self.h, self.w, 4))[limite_haute:limite_basse,self.w//2,0:3]

            # leftscan=np.array(scan)[:,10,0:3]
            # rightscan=np.array(scan)[:,scan.shape[1]-10,0:3]
            # middlescan=np.array(scan)[:,scan.shape[1]//2,0:3]

            #On convertie leurs valeur bgr en valeur hsv
            rgb=rospy.get_param('rgb',default=0)
            self.lefthsv = bgr2hsv(leftscan,rgb)
            self.righthsv = bgr2hsv(rightscan,rgb)
            self.middlehsv = bgr2hsv(middlescan,rgb)
            rospy.loginfo(self.middlehsv)

    def run(self):
        rate = rospy.Rate(20)

        while not rospy.is_shutdown() :
            left_is_green=rospy.get_param('left_is_green',default=True)

            #On compte le nombre de pixel rouge selon leur valeur hsv
            count_red_left=0
            for p in self.lefthsv:
                if p[0]<14 or p[0]>280:
                    if p[1]>90 and p[2]>50:
                        count_red_left+=1
        
            count_red_right=0
            for p in self.righthsv:
                if p[0]<14 or p[0]>330:
                    if p[1]>90 and p[2]>50:
                        count_red_right+=1

            count_red_middle=0
            for p in self.middlehsv:
                if p[0]<14 or p[0]>330:
                    if p[1]>90 and p[2]>50:
                        count_red_middle+=1

        
        
            #On compte le nombre de pixel vert à l'aide de leur valeur hsv
            count_green_left=0
            for p in self.lefthsv:
                if p[0]>90 and p[0]<160:
                    if p[1]>50 and p[2]>50:
                        count_green_left+=1
        
            count_green_right=0
            for p in self.righthsv:
                if p[0]>90 and p[0]<160:
                    if p[1]>50 and p[2]>50:
                        count_green_right+=1

            count_green_middle=0
            for p in self.middlehsv:
                if p[0]>90 and p[0]<160:
                    if p[1]>50 and p[2]>50:
                        count_green_middle+=1
                

            #Publication par default
            self.wcolor.data="???"
            self.direction.data="???"

            
            
            # if (count_green_right > 30 and count_green_left > 30) or (count_red_right > 30 and count_red_left > 30) or (count_red_right > 30 and count_green_left > 30) or (count_green_right > 30 and count_red_left > 30):
            rospy.loginfo(f"yesred_l={count_red_left} red_r{count_red_right} green_l{count_green_left} green_r{count_green_right}")

            #On passe d'une sensibilité à l'autre en fonction de self.sensi

            if self.sensi:
                #On détermine la direction prise par le véhicule
                if ((count_green_right < count_green_left) or (count_red_left < count_red_right)) :
                    self.direction.data="wrong"
                    self.dir.data=False
                
                elif (count_green_right > count_green_left) or (count_red_left > count_red_right):
                    self.direction.data="right"
                    self.dir.data=True

                else:
                    self.direction.data="???"
                    self.dir.data=True

            
            else:
                #On détermine la direction prise par le véhicule
                if (count_red_right > 40 and count_green_left > 40):
                    self.direction.data="wrong"
                    self.dir.data=False
                    
                elif (count_green_right > 40 and count_red_left > 40):
                    self.direction.data="right"
                    self.dir.data=True

                else:
                    self.direction.data="???"
                    self.dir.data=True

            #On indique la couleur en face du véhicule

            if (count_green_middle > count_red_middle) and count_green_middle > 40:
                self.wcolor.data="green"
            
            elif (count_red_middle > count_green_middle) and count_red_middle > 40:
                self.wcolor.data="red"

            else:
                self.wcolor.data="???"

            if not(left_is_green):
                if self.direction.data=="wrong":
                    self.direction.data="right"
                if self.direction.data=="right":
                    self.direction.data="wrong"
                self.dir.data=not(self.dir.data)


            self.pubdirection.publish(self.direction)
            self.pubdir.publish(self.dir)
            self.pubwcolor.publish(self.wcolor)

            rate.sleep()




if __name__ == '__main__':
    
    w, h = 640, 480
    try:
        d = Dir_indicator(w, h) 
        d.run()
        rospy.spin()

    except rospy.ROSInterruptException:
        pass