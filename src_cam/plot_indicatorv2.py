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

class ImagePlot :

    def __init__(self, w, h) :
        self.fig, self.ax = plt.subplots(figsize=(7, 4))
        self.w, self.h = w, h
        self.image = np.zeros((self.h, self.w))
        self.ln = plt.imshow(self.image)

        self.reelparam = rospy.get_param("reel", default=0)
        
        self.cv_bridge = CvBridge()

    def initPlot(self)  :
        self.ax.set_title("Camera video")
        self.fig.set_facecolor((207/255, 106/255, 4/255))
        return self.ln

    def callback(self, msg) :
        limite_haute=rospy.get_param('lim_haut', default=0)
        limite_basse=rospy.get_param('lim_bas', default=0)
        scan = msg.data		
        self.image = np.array(scan).reshape((self.h, self.w))


        self.image[limite_haute,:]=255
        self.image[limite_basse,:]=255

    def updatePlot(self, frame) :
        self.ln.set_data(self.image)
        return self.ln

def listener(p) :
    topic = rospy.get_param("scan_image", default="/Scan_image")
    reelparam = rospy.get_param("reel", default=0)

    rospy.Subscriber(topic, Int16MultiArray, p.callback)


    plt.show(block=True)

if __name__ == "__main__" :
    rospy.init_node('camera_plot', anonymous = True)
    w = rospy.get_param("image_width", default=640)
    h = rospy.get_param("image_height", default=480)
    p = ImagePlot(w, h)

    ani = FuncAnimation(p.fig, p.updatePlot, init_func = p.initPlot)
    try : listener(p)
    except (rospy.ROSInterruptException, KeyboardInterrupt) : sys.quit()
