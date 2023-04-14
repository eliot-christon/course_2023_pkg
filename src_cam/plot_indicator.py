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
import cv2

# def bgr2hsv (colonne, rgb=0):
#     """fonction qui convertit les valeurs d'une colonne de pixel bgr en leur valeur hsv"""
#     colonne=colonne/255
#     hsv=np.zeros(colonne.shape)
#     for i,p in enumerate(colonne):
#         if rgb == 1 :
#             r,g,b=p[0],p[1],p[2]
#         else:
#             b,g,r=p[0],p[1],p[2]
#         v=np.max(p)
#         if v != 0:
#             s=(v-np.min(p))/v
#         else:
#             s=0
#         if b==g and g==r:
#             h=0
#         elif v==r:
#             h=60*(g-b)/(v-np.min(p))
#         elif v==g:
#             h=120+60*(b-r)/(v-np.min(p))
#         elif v==b:
#             h=240+60*(r-g)/(v-np.min(p))
#         if h < 0:
#             h+=360
#         v=255*v
#         s=255*s
#         hsv[i]=[h,s,v]
        
#     return hsv

def bgr2hsv (p, rgb=0):
    """fonction qui convertit les valeurs d'une colonne de pixel bgr en leur valeur hsv"""
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
        
    return h,s,v

class ImagePlot :

    def __init__(self, w, h) :
        self.fig, self.ax = plt.subplots(figsize=(7, 4))
        self.w, self.h = w, h
        self.image = np.zeros((self.h, self.w))
        self.ln = plt.imshow(self.image)

        self.cv_bridge = CvBridge()

        self.max_hue_red = rospy.get_param('max_hue_red', default=14)
        self.min_hue_red = rospy.get_param('min_hue_red', default=280)
        self.max_hue_green = rospy.get_param('max_hue_green', default=160)
        self.min_hue_green = rospy.get_param('min_hue_green', default=90)


        self.min_sat_red = rospy.get_param('min_sat_red', default=90)
        self.min_sat_green = rospy.get_param('min_sat_green', default=50)
        self.max_sat_red = rospy.get_param('max_sat_red', default=255)
        self.max_sat_green = rospy.get_param('max_sat_green', default=255)

        self.min_val_red = rospy.get_param('min_val_red', default=50)
        self.min_val_green = rospy.get_param('min_val_green', default=50)
        self.max_val_red = rospy.get_param('max_val_red', default=255)
        self.max_val_green = rospy.get_param('max_val_green', default=255)

    def initPlot(self)  :
        self.ax.set_title("Camera video")
        self.fig.set_facecolor((207/255, 106/255, 4/255))
        return self.ln

    def callback(self, msg) :
        #parametres détection de couleurs
        self.max_hue_red = rospy.get_param('max_hue_red', default=14)
        self.min_hue_red = rospy.get_param('min_hue_red', default=280)
        self.max_hue_green = rospy.get_param('max_hue_green', default=160)
        self.min_hue_green = rospy.get_param('min_hue_green', default=90)

        

        self.min_sat_red = rospy.get_param('min_sat_red', default=90)
        self.min_sat_green = rospy.get_param('min_sat_green', default=50)
        self.max_sat_red = rospy.get_param('max_sat_red', default=255)
        self.max_sat_green = rospy.get_param('max_sat_green', default=255)

        

        self.min_val_red = rospy.get_param('min_val_red', default=50)
        self.min_val_green = rospy.get_param('min_val_green', default=50)
        self.max_val_red = rospy.get_param('max_val_red', default=255)
        self.max_val_green = rospy.get_param('max_val_green', default=255)

        limite_haute=rospy.get_param('lim_haut', default=0)
        limite_basse=rospy.get_param('lim_bas', default=0)

        rgb=rospy.get_param('rgb',default=0)

        scan = self.cv_bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")

        image = scan[:,:,::-1]
        img=image.copy()
        img1=image.copy()



        for i in range(len(img)):
            for j in range(len(img[0])):
                h,s,v=bgr2hsv (img[i][j],rgb)

                if i == limite_haute and j== len(img[0])//2:
                    print(img[i][j],h,s,v)

                if h < self.max_hue_red or h > self.min_hue_red:
                    if s>self.min_sat_red and v>self.min_val_red:
                        img[i,j,:]=255

                # print(self.max_hue_green,self.min_hue_green, h)
                # if h < self.max_hue_green and h > self.min_hue_green:
                #     if s>self.min_sat_green and v>self.min_val_green:
                #         img[i,j,:]=255

    

        img[limite_haute,:,:]=255
        img[limite_basse,:,:]=255

        self.image=img




    def updatePlot(self, frame) :
        self.ln.set_data(self.image)
        return self.ln

def listener(p) :
    topic = rospy.get_param("image_datas", default="/ImageScan")
    rospy.Subscriber(topic, SensorImage, p.callback)
    plt.show(block=True)

if __name__ == "__main__" :
    rospy.init_node('camera_plot', anonymous = True)
    w = rospy.get_param("image_width", default=640)
    h = rospy.get_param("image_height", default=480)
    p = ImagePlot(w, h)

    ani = FuncAnimation(p.fig, p.updatePlot, init_func = p.initPlot)
    try : listener(p)
    except (rospy.ROSInterruptException, KeyboardInterrupt) : sys.quit()
