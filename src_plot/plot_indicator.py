#!/usr/bin/env python3
# -*- coding: utf-8 -*-

# PFE - Autonomous vehicle / UE ROS 
# Author : Tom DA SILVA-FARIA, Julien JOYET

import rospy
import sys, io
import numpy as np
import matplotlib.pyplot as plt

from matplotlib.animation import FuncAnimation
from sensor_msgs.msg import Image as SensorImage
from std_msgs.msg import Int16MultiArray
from cv_bridge import CvBridge, CvBridgeError
from course_2023_pkg.srv import UpdateParameterfloat
from course_2023_pkg.srv import GetParameters
import cv2

def rgb2hsv(p):
    """fonction qui convertit les valeurs d'une colonne de pixel rgb en leur valeur hsv"""
    p=p[0:3]
    p = p / 255

    r,g,b=p[0],p[1],p[2]

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

class Plot_indicator :
    """Ce noeud a pour but d'afficher ce que détecte l'indicateur de direction et d'être utilisé pour le calibrage de la détection de couleurs"""

    def __init__(self, w, h) :
        self.fig, self.ax = plt.subplots(figsize=(7, 4))
        self.w, self.h = w, h
        self.image = np.zeros((60, 75,3))
        self.imagehsv = np.zeros((60, 75,3))
        self.ln = plt.imshow(self.image)



            #======Service======

        #On déclare un service pour modifier les paramètres pour la détection des couleurs
        self.service = rospy.Service('update_color_parameter', UpdateParameterfloat, self.update_parameter_handler)

        self.service = rospy.Service('get_color_parameter', GetParameters, self.get_parameters)


        #conversion opencv
        self.cv_bridge = CvBridge()

            #======Parameters======

        #Paramètre qui détermine si on est dans le cas d'un robot réel
        self.reelparam = rospy.get_param("reel", default=1)

        #Paramètres de valeurs de teinte
        self.max_hue_red = rospy.get_param('max_hue_red', default=20)
        self.min_hue_red = rospy.get_param('min_hue_red', default=280)
        self.max_hue_green = rospy.get_param('max_hue_green', default=190)
        self.min_hue_green = rospy.get_param('min_hue_green', default=90)

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

        #Paramètres qui détermine les cadres pour la vision du vehicule
        self.limite_haute=rospy.get_param('lim_haut', default=52)
        self.limite_basse=rospy.get_param('lim_bas', default=100)
        self.thick=rospy.get_param('thick', default=10)

        #Paramètre qui prend en compte si la caméra est en rgb ou en bgr
        self.rgb=rospy.get_param('rgb',default=1)
    

    def update_parameter_handler(self, req):
        """Fonction permettant de mettre à jour les paramètres pour la détection des couleurs"""

    #Paramètres de la couleur rouge
        if req.parameter_name == "max_hue_red":
            self.max_hue_red = int(req.new_value)

        elif req.parameter_name == "min_hue_red":
            self.min_hue_red = int(req.new_value)

        elif req.parameter_name == "min_sat_red":
            self.min_sat_red = int(req.new_value)

        elif req.parameter_name == "min_val_red":
            self.min_val_red = int(req.new_value)

    #Paramètres de la couleur verte
        elif req.parameter_name == "max_hue_green":
            self.max_hue_green = int(req.new_value)

        elif req.parameter_name == "min_hue_green":
            self.min_hue_green = int(req.new_value)

        elif req.parameter_name == "min_sat_green":
            self.min_sat_green = int(req.new_value)

        elif req.parameter_name == "min_val_green":
            self.min_val_green = int(req.new_value)

    #RGB/BGR
        elif req.parameter_name == "rgb":
            self.rgb = int(req.new_value)

    #Cadre pour limiter la détection
        elif req.parameter_name == "lim_haut":
            self.limite_haute = int(req.new_value)

        elif req.parameter_name == "lim_bas":
            self.limite_basse = int(req.new_value)
                                    
        elif req.parameter_name == "thick":
            self.thick = int(req.new_value)
        
        else:
            return False, f"Parameter {req.parameter_name} not found."
        return True, f"Parameter {req.parameter_name} updated successfully."
    
    def get_parameters(self, req):
        """Cette méthode permet de récupérer les valeurs des paramètres, ils sont affichés dans le terminal du noeud et non dans le terminal où on appelle le service"""
        print(
        f"<param name='max_hue_red' value='{self.max_hue_red}'/>\n"
        f"<param name='min_hue_red' value='{self.min_hue_red}'/>\n"
        f"<param name='max_hue_green' value='{self.max_hue_green}'/>\n"
        f"<param name='min_hue_green' value='{self.min_hue_green}'/>\n"
        
        f"<param name='min_sat_red' value='{self.min_sat_red}'/>\n"
        f"<param name='min_sat_green' value='{self.min_sat_green}'/>\n"
        
        f"<param name='min_val_red' value='{self.min_val_red}'/>\n"
        f"<param name='min_val_green' value='{self.min_val_green}'/>\n"

        f"<param name='lim_haut' value='{self.limite_haute}'/>\n"
        f"<param name='lim_bas' value='{self.limite_basse}'/>\n"
        f"<param name='thick' value='{self.thick}'/>\n"

        f"<param name='rgb' value='{self.rgb}'/>\n"
        )
        return True
    
    def format_coord(self,x, y):
        x, y = int(x), int(y)
        if 0 <= x < self.imagehsv.shape[1] and 0 <= y < self.imagehsv.shape[0]:
            h, s, v = self.imagehsv[y, x]
            return f"x={x}, y={y}, H={h}, S={s}, V={v}"
        else:
            return f"x={x}, y={y}"




    def initPlot(self)  :
        self.ax.set_title("Camera video")
        self.fig.set_facecolor((207/255, 106/255, 4/255))
        return self.ln

    def callback(self, msg):
        """Récupération des données d'images"""
        if self.reelparam == 0:
            scan = msg.data
            image = np.array(scan).reshape((self.h, self.w, 4))
        else:
            scan = self.cv_bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")
            image = np.array(scan)

        img = cv2.resize(image.astype('float32'), (75, 60), interpolation=cv2.INTER_LINEAR)

        if self.rgb == 0:
            img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)

        self.image = img[:, :, 0:3].astype('uint8')

        #La fonction rgb2hsv consomme plus de ressources que la fonction de opencv mais les valeurs données sont toujours correctes 
        # alors que les valeurs données par opencv sont parfois incohérentes => à voir comment corriger ça et quand même utiliser opencv 
        self.imagehsv = np.array([[rgb2hsv(p) for p in row] for row in self.image])

        #self.imagehsv = cv2.cvtColor(self.image.astype('uint8'), cv2.COLOR_RGB2HSV)[:,:,0:3]
        #self.imagehsv[:,:,1] = (self.imagehsv[:,:,1] * 255)
        #self.imagehsv[:,:,0] = (self.imagehsv[:,:,0] * 3.6)
        #self.imagehsv=self.imagehsv.astype('uint8')

    def updatePlot(self, frame) :
        img=self.image.copy()
        #On indique sur l'image quels pixels sont repérés par l'algo
        for i in range(len(img)):
            for j in range(len(img[0])):
                h,s,v=rgb2hsv(self.image[i][j])

                if h < self.max_hue_red or h > self.min_hue_red:
                    if s>self.min_sat_red and v>self.min_val_red:
                        img[i,j]=[255,0,0]

                if h < self.max_hue_green and h > self.min_hue_green:
                    if s>self.min_sat_green and v>self.min_val_green:
                        img[i,j]=[0,255,0]

        #On dessine les cadres utilisés pour repérer les couleurs

        if self.limite_haute == 0 and self.limite_basse == 60:

            img[self.limite_haute:self.limite_basse,self.thick,:]=255
            img[self.limite_haute:self.limite_basse,img.shape[1]-self.thick-1,:]=255

            img[self.limite_haute:self.limite_basse,int(img.shape[1]//2-(self.thick/2))-1,:]=255
            img[self.limite_haute:self.limite_basse,int(img.shape[1]//2+(self.thick/2)),:]=255


        elif self.limite_haute == 0:
            img[self.limite_basse+1,0:self.thick+1,:]=255
            img[self.limite_basse+1,img.shape[1]-self.thick-1:img.shape[1],:]=255

            img[self.limite_haute:self.limite_basse+1,self.thick,:]=255
            img[self.limite_haute:self.limite_basse+1,img.shape[1]-self.thick-1,:]=255

            img[self.limite_haute:self.limite_basse+1,int(img.shape[1]//2-(self.thick/2))-1,:]=255
            img[self.limite_haute:self.limite_basse+1,int(img.shape[1]//2+(self.thick/2)),:]=255
            img[self.limite_basse+1,int(img.shape[1]//2-(self.thick/2))-1:int(img.shape[1]//2+(self.thick/2))+1,:]=255

        elif self.limite_basse == 60:
            img[self.limite_haute-1,0:self.thick+1,:]=255
            img[self.limite_haute-1,img.shape[1]-self.thick-1:img.shape[1],:]=255

            img[self.limite_haute-1:self.limite_basse,self.thick,:]=255
            img[self.limite_haute-1:self.limite_basse,img.shape[1]-self.thick-1,:]=255

            img[self.limite_haute-1:self.limite_basse,int(img.shape[1]//2-(self.thick/2))-1,:]=255
            img[self.limite_haute-1:self.limite_basse,int(img.shape[1]//2+(self.thick/2)),:]=255
            img[self.limite_haute-1,int(img.shape[1]//2-(self.thick/2))-1:int(img.shape[1]//2+(self.thick/2))+1,:]=255

        else:
            img[self.limite_haute-1,0:self.thick+1,:]=255
            img[self.limite_haute-1,img.shape[1]-self.thick-1:img.shape[1],:]=255
            img[self.limite_basse+1,0:self.thick+1,:]=255
            img[self.limite_basse+1,img.shape[1]-self.thick-1:img.shape[1],:]=255

            img[self.limite_haute-1:self.limite_basse+1,self.thick,:]=255
            img[self.limite_haute-1:self.limite_basse+1,img.shape[1]-self.thick-1,:]=255

            img[self.limite_haute-1:self.limite_basse+1,int(img.shape[1]//2-(self.thick/2))-1,:]=255
            img[self.limite_haute-1:self.limite_basse+1,int(img.shape[1]//2+(self.thick/2)),:]=255
            img[self.limite_haute-1,int(img.shape[1]//2-(self.thick/2))-1:int(img.shape[1]//2+(self.thick/2))+1,:]=255
            img[self.limite_basse+1,int(img.shape[1]//2-(self.thick/2))-1:int(img.shape[1]//2+(self.thick/2))+1,:]=255


        #Cette ligne permet d'afficher les valeurs hsv en passant le curseur sur l'image qui sera affichée
        self.ax.format_coord = self.format_coord

        self.ln.set_data(img)
        return self.ln


def listener(p) :
    topic = rospy.get_param("image_datas", default="/ImageScan")
    if p.reelparam == 1:
        rospy.Subscriber(topic, SensorImage, p.callback)
    else:
        rospy.Subscriber(topic, Int16MultiArray, p.callback)
    plt.show(block=True)



if __name__ == "__main__" :
    rospy.init_node('camera_plot', anonymous = True)
    w = rospy.get_param("image_width", default=640)
    h = rospy.get_param("image_height", default=480)
    p = Plot_indicator(w, h)

    ani = FuncAnimation(p.fig, p.updatePlot, init_func = p.initPlot)
    try : listener(p)
    except (rospy.ROSInterruptException, KeyboardInterrupt) : sys.exit()