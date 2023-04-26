#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
import sys
import numpy as np
import matplotlib.pyplot as plt

from matplotlib.animation import FuncAnimation
from std_msgs.msg import Float32MultiArray
from sensor_msgs.msg import LaserScan


class Plot :

	def __init__(self, xlim, ylim) :

		self.fig, self.ax = plt.subplots(figsize=(6, 6))
		self.ln, = plt.plot([], [], '.b')
		self.x_data, self.y_data = [], []

		self.xlim, self.ylim = [-xlim, xlim], [-ylim, ylim]
		self.angles = None

	def initPlot(self)  :
		self.ax.set_xlim(self.xlim) ; self.ax.set_ylim(self.ylim)
		self.ax.set_xlabel("distance (m)") ; self.ax.set_ylabel("distance (m)")
		self.ax.invert_yaxis() ; self.ax.invert_xaxis()
		self.ax.set_title("Lidar Plot")
		self.ax.grid(color="gray")
		self.ax.set_facecolor((0.0, 0.0, 0.0))
		self.fig.set_facecolor((207/255, 106/255, 4/255))
		return self.ln

	def callback(self, msg) :

		scan = msg.data #le lidar envoit un scan 360° autour de la voiture sur un array de longueur len(scan)
		if len(scan)>0:
			a0,a1=rospy.get_param("angle0",default=120),rospy.get_param("angle1",default=240)#rospy.get_param("angle0",default=120),rospy.get_param("angle1",default=240)
			if self.angles is None : self.angles = np.linspace(np.deg2rad(a0), np.deg2rad(a1),len(scan))#(np.deg2rad(120), np.deg2rad(240), len(scan)) #pour front data
			
			self.x_data = [] ; self.y_data = []
			for i in range(len(scan)-1) :
				#On converti la distance renvoyee par data en coord x,y dans le referentiel du robot
				self.x_data.append(scan[i] * np.cos(self.angles[i]))
				self.y_data.append(scan[i] * np.sin(self.angles[i]))
		
	def callback_scan(self, msg) :

		scan = np.array(msg.ranges) #le lidar envoit un scan 360° autour de la voiture sur un array de longueur len(scan)
		if len(scan)>0:
			scan=np.roll(scan,len(scan)//2)
			scan=np.flip(scan)
			a0,a1=rospy.get_param("angle0",default=120),rospy.get_param("angle1",default=240)#rospy.get_param("angle0",default=120),rospy.get_param("angle1",default=240)
			if self.angles is None : self.angles = np.linspace(np.deg2rad(a0), np.deg2rad(a1),len(scan))#(np.deg2rad(120), np.deg2rad(240), len(scan)) #pour front data
			
			self.x_data = [] ; self.y_data = []
			for i in range(len(scan)-1) :
				#On converti la distance renvoyee par data en coord x,y dans le referentiel du robot
				self.x_data.append(scan[i] * np.cos(self.angles[i]))
				self.y_data.append(scan[i] * np.sin(self.angles[i]))
			
			
	def updatePlot(self, frame) :
		self.ln.set_data(self.y_data, self.x_data)
		return self.ln 	
			
			
def listener(p) : 
	topic = rospy.get_param("lidar_datas", default="/front_data") #/front_data
	
	#rospy.Subscriber(topic,LaserScan,p.callback_scan)

	rospy.Subscriber(topic, Float32MultiArray, p.callback)
	plt.show(block = True)
	
		
if __name__ == '__main__' :
	

	rospy.init_node('plot_lidar', anonymous = True)
	
	xlim, ylim = 5, 5
	p = Plot(xlim, ylim) 
	
	ani = FuncAnimation(p.fig, p.updatePlot, init_func = p.initPlot)
	try : listener(p)
	except (rospy.ROSInterruptException, KeyboardInterrupt) : sys.quit() 
		

		 
