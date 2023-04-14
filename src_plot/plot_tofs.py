#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
Projet PFE : Voiture autonome
@Author : Eliot CHRISTON FRAGA
"""


import rospy
import sys 
import matplotlib.pyplot as plt
import numpy as np

from matplotlib.animation import FuncAnimation 
from std_msgs.msg import Float32MultiArray, Int16MultiArray

class Plot : 

    def __init__(self) :
        print("init")
        self.fig, self.ax = plt.subplots(figsize=(6, 6))
        self.nums = ["fl", "fr", "bl", "br"]
        self.raw_data, self.processed_data = [0, 0, 0, 0], [0, 0, 0, 0]

    def initPlot(self)  :
        print("init plot")
        self.ax.set_xlabel("tof num") ; self.ax.set_ylabel("distance (m)")
        self.ax.set_title("Tofs plot")

        self.ax.grid(color="gray")
        self.bars()

        plt.legend(loc='upper right')
        return self.fig

    def bars(self) :
        self._p1 = self.ax.bar(self.nums, self.raw_data, alpha=0.5, label='raw_data', color = 'r', align='center', width=0.25)
        self._p2 = self.ax.bar(self.nums, self.processed_data, alpha=0.5, label='processed_data', color = 'b', align='edge', width=0.25)

    def callback_raw(self, msg) :
        self.raw_data = [d/1000 for d in msg.data]
    
    def callback_processed(self, msg) :
        self.processed_data = msg.data

    def updatePlot(self, frame) :
        print("update plot")
        try : 
            self._p1.remove()
            self._p2.remove()
            self.bars()

        except : print("[WRN] - No data got to be plotted")
        return self.fig


def listener(p) :
    print("listener")
    topic_raw_data = rospy.get_param("tof_topic", default="/SensorsScan")
    topic_processed_data = rospy.get_param("tof_processed_topic", default="/TofsDistance")
    if topic_raw_data == "/TofsScan" :
        rospy.Subscriber(topic_raw_data, Int16MultiArray, p.callback_raw)
        print("Real robot")
    else :
        rospy.Subscriber(topic_raw_data, Float32MultiArray, p.callback_raw)
        print("Simulated robot")
    rospy.Subscriber(topic_processed_data, Float32MultiArray, p.callback_processed)
    plt.show(block = True)
	
		
if __name__ == '__main__' :

	rospy.init_node('plot_sensors', anonymous = True)
	
	p = Plot() 
	
	ani = FuncAnimation(p.fig, p.updatePlot, init_func = p.initPlot)

	try : listener(p)
	except (rospy.ROSInterruptException, KeyboardInterrupt) : sys.quit() 
		

