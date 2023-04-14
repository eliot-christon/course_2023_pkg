#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
Projet PFE : Voiture autonome
@Author : Tom DA SILVA-FARIA
"""

import sys
import rospy
import numpy as np
from PyQt5 import QtCore, QtGui, QtWidgets 

from std_msgs.msg import Float32, Bool

class Controller(QtWidgets.QMainWindow) : 

	pressed_keys = {QtCore.Qt.Key_Left : False, QtCore.Qt.Key_Right : False, 
					QtCore.Qt.Key_Up   : False,   QtCore.Qt.Key_Down  : False,
					QtCore.Qt.Key_Space : False}
					
	def __init__(self, max_speed, max_angle) : 
		
		## GUI INIT #######################
		QtWidgets.QMainWindow.__init__(self)
		self.setWindowTitle("TELEOP - Robot Teleoperation")
		self.resize(400, 200)
		QtWidgets.QApplication.setOverrideCursor(QtCore.Qt.ArrowCursor)
		self.setStyleSheet(""" QMainWindow { background-color: rgb(207, 106, 4); }
			QMenuBar { background-color: rgb(255,255,255); color: rgb(0, 0, 0); }
			QMenuBar::item { background-color: rgb(255,255,255); color: rgb(0,0,0); }
			QMenuBar::item::selected { background-color: rgb(245,245,245); border: 1px solid #202020}
			QMenu { background-color: rgb(245,245,245); color: rgb(0,0,0); }
			QMenu::item::selected { background-color: rgb(235,235,235); } 
			QFrame {background-color: rgb(255, 255, 255); border: 4px rgb(57, 56, 54); 
					border-top-left-radius: 4px; 
					border-top-right-radius: 4px; 
					border-bottom-left-radius: 4px; 
					border-bottom-right-radius: 4px;
					border-style: outset;
					border-color: rgb(245, 245, 245); } 
			QLabel {background-color: rgb(255, 255, 255); color: rgb(120, 120, 120);
					border: 0px solid #202020; font-size: 13pt; font-family: Arial; font-weight: bold}
			""")
			
		wid = QtWidgets.QWidget(self)
		self.setCentralWidget(wid)
		grid_layout = QtWidgets.QGridLayout()   

		# Setup Menu bar
		menu_bar = self.menuBar()
		file_menu = QtWidgets.QMenu("Menu", self)
		self.__mn_action("Exit", "Ctrl+C", self.close, file_menu)
		menu_bar.addMenu(file_menu)
		
		# Setup main frame
		param_frame = QtWidgets.QFrame() 
		param_frame.setSizePolicy(QtWidgets.QSizePolicy.Fixed, QtWidgets.QSizePolicy.Fixed)
		param_layout = QtWidgets.QVBoxLayout()
		param_layout.setContentsMargins(5, 5, 5, 5) 
		param_layout.setSpacing(5)
		param_label = QtWidgets.QLabel() ; param_label.setText("Robot Teleoperation : use keyboard arrows") 
		param_label.setSizePolicy(QtWidgets.QSizePolicy.Fixed, QtWidgets.QSizePolicy.Fixed)
		param_layout.addWidget(param_label, stretch=1)
		self.b_left = QtWidgets.QCheckBox("Left") ; self.b_left.setChecked(False)  
		self.b_left.setEnabled(False)
		self.b_right = QtWidgets.QCheckBox("Right") ; self.b_right.setChecked(False)  
		self.b_right.setEnabled(False)
		self.b_up = QtWidgets.QCheckBox("Up") ; self.b_up.setChecked(False)  
		self.b_up.setEnabled(False)
		self.b_down = QtWidgets.QCheckBox("Down") ; self.b_down.setChecked(False)  
		self.b_down.setEnabled(False)

		#ajout space_bar
		self.b_space = QtWidgets.QCheckBox("Start/Stop") ; self.b_space.setChecked(False)  
		self.b_space.setEnabled(False)

		param_layout.addWidget(self.b_left) ; param_layout.addWidget(self.b_up) 
		param_layout.addWidget(self.b_right) ; param_layout.addWidget(self.b_down)
		param_layout.addWidget(self.b_space)
		param_frame.setLayout(param_layout)
		
		# Adding frames to GUI
		grid_layout.addWidget(param_frame, 0, 0, 1, 1)
		
		wid.setLayout(grid_layout) 
		QtWidgets.QApplication.processEvents()
		
		self.timer = QtCore.QTimer(self)
		self.timer.timeout.connect(self.key_action)
		self.timer.start(100)
			
		self.angles = None
		self.x_data, self.y_data = [], [] 
		self.max_speed, self.max_angle = max_speed, max_angle 
		self.speed_msg, self.angle_msg = Float32(), Float32() 
		self.pub_speed = rospy.Publisher("/SpeedCommand", Float32, queue_size = 1)
		self.pub_angle = rospy.Publisher("/AngleCommand", Float32, queue_size = 1) 

		#commande a distance
		self.pub_start=rospy.Publisher("/Start_flag",Bool, queue_size=1)
		self.start=False
		
	
	def __mn_action(self, text, short_cut, connection, menu) :
		""" Add an option in the menu """
		action = QtWidgets.QAction(text, self)
		action.setShortcut(short_cut)
		action.triggered.connect(connection)
		menu.addAction(action)
	   					 
	def keyPressEvent(self, event) : 
		key = event.key() 
		if key in self.pressed_keys.keys() : self.pressed_keys[key] = True 
	
	def keyReleaseEvent(self, event) : 
		key = event.key() 
		if key in self.pressed_keys.keys() : self.pressed_keys[key] = False 
		
	def key_action(self) : 
		""" Callback publisher function : called at each timestep """
		
		if (self.pressed_keys[QtCore.Qt.Key_Up]) : 
			self.speed_msg = self.max_speed 
			self.b_up.setEnabled(True) ; self.b_up.setChecked(True) ; self.b_up.setEnabled(False)
		else : 
			if (self.pressed_keys[QtCore.Qt.Key_Down]) : 
				self.speed_msg = -0.2*self.max_speed  
				self.b_down.setEnabled(True) ; self.b_down.setChecked(True) ; self.b_down.setEnabled(False)
			else : 
				self.speed_msg = 0.0 
				self.b_up.setEnabled(True) ; self.b_up.setChecked(False) ; self.b_up.setEnabled(False)
				self.b_down.setEnabled(True) ; self.b_down.setChecked(False) ; self.b_down.setEnabled(False)
			
		if (self.pressed_keys[QtCore.Qt.Key_Right]) : 
			self.angle_msg = self.max_angle 
			self.b_right.setEnabled(True) ; self.b_right.setChecked(True) ; self.b_right.setEnabled(False)
		else : 
			if (self.pressed_keys[QtCore.Qt.Key_Left]) : 
				self.angle_msg = -1*self.max_angle 
				self.b_left.setEnabled(True) ; self.b_left.setChecked(True) ; self.b_left.setEnabled(False) 
			else : 
				self.angle_msg = 0.0 
				self.b_right.setEnabled(True) ; self.b_right.setChecked(False) ; self.b_right.setEnabled(False)
				self.b_left.setEnabled(True) ; self.b_left.setChecked(False) ; self.b_left.setEnabled(False)

		if (self.pressed_keys[QtCore.Qt.Key_Space]):
			#ICI faire commande de start -> lancer un flag sur topic qui permet a navigation haut niveau de demarrer our arreter moteurs
			self.b_space.setEnabled(True);self.b_space.setChecked(True);self.b_space.setEnabled(False)
			self.start= not self.start
			
		else:
			self.b_space.setEnabled(True);self.b_space.setChecked(False);self.b_space.setEnabled(False)
			


		self.pub_speed.publish(self.speed_msg)	
		self.pub_angle.publish(self.angle_msg)	
		self.pub_start.publish(self.start)
				
if __name__ == "__main__" :
	
	rospy.init_node('teleop_robot', anonymous = True)
	app = QtWidgets.QApplication(sys.argv)
	app.setApplicationName("Teleop Controller")

	max_speed = rospy.get_param("max_speed", default=1.0)
	max_angle = rospy.get_param("max_angle", default=1.0)

	window = Controller(max_speed, max_angle)
	window.show()

	sys.exit(app.exec_())
