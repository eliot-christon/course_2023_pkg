#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
import numpy as np


from std_msgs.msg import Float32, Float32MultiArray, Bool
import message_filters 


class Controller:
    def __init__(self):
        self.speed=0.0#Float32()
        self.ang=0.0#Float32()
        self.command=Float32MultiArray()
        self.last_err=0
        self.last_command=0
        self.F=10 #valeur par defaut, mise a jour dans main p.rapp a freq de pub
        self.run=True
        #controler gains
        self.k_c=rospy.get_param("kc",default=-0.85)
        self.k_d=rospy.get_param("kd",default=8)
        self.tau=rospy.get_param("tau",default=1e-6) #valeur a determiner pour lead-controller->en lien avec vitesse de reaction <T/2=0.05
        self.a=rospy.get_param("a",default=10) #en lien avec la freq a laquelle on veut gain de phase

def angle_regulator_callback(msg_dir,c):
    if c.run:
        #d_center=msg_center.data #centering err : left-right -> d_c>0 : go left, d_c<0 : go right
        d_dir=msg_dir.data #orientation err : desired_orientation-current_orientation -> d_dir>0 : go right, d_dir<0 : go left 

        #controler gains
        k_c=c.k_c
        k_d=c.k_d
        tau=c.tau #valeur a determiner pour lead-controller->en lien avec vitesse de reaction <T/2=0.05
        a=c.a #en lien avec la freq a laquelle on veut gain de phase

        #le gain pour dir doit etre plus grand que celui du centrage pour garantir evitement d'obstacle avant de se centrer
        u=1/(1-2*tau*c.F)*(k_d*(d_dir*(1-2*a*tau*c.F)+c.last_err*(1+2*a*tau*c.F))-c.last_command*(1+2*tau*c.F))#+ k_c*d_center  #k_d*d_dir +k_c*d_center 
        c.last_command=u
        c.last_err=d_dir

        #saturator
        command=np.tanh(u)

        
        c.ang=command*rospy.get_param("MAX_ANGLE",default=1)


def speed_regulator_callback(msg_front_dist,c):
    if c.run:
        MIN_SPEED=rospy.get_param("MIN_SPEED",default=0.0)
        front_dist=msg_front_dist.data
        
        k_s=rospy.get_param("ks",default=1)

        u=k_s*front_dist #PEUT ETRE METTRE DIST MINIMALE->EN DESSOUS U=0 : ON S'ARRETE

        c.speed=np.clip(u,MIN_SPEED,rospy.get_param("MAX_SPEED",default=1))#np.tanh(u)*rospy.get_param("MAX_SPEED",default=1)
        #c.speed = rospy.get_param("~speed" ,default=0.3)


def onrun_callback(msg,c):
    c.run=msg.data 

if __name__=='__main__':
    try:
        
        #init node
        rospy.init_node("lidar_nav_control")

        #init controller
        c = Controller()
        #c.speed=0.65

        #check haut niveau navigation
        haut_niv=rospy.get_param("haut_niv",default=False)
        #print("Navigation haut_niv:",haut_niv)

        #subscribe to lidar_dir and lidar_center topics
        dir_topic=rospy.get_param("~dir_topic",default="/lidar_dir")
        center_topic=rospy.get_param("~center_topic",default="/lidar_center")

        dir_sub=rospy.Subscriber(dir_topic,Float32,angle_regulator_callback,c, queue_size=1)
        #center_sub=message_filters.Subscriber(center_topic,Float32, queue_size=1)

        #ts=message_filters.ApproximateTimeSynchronizer([dir_sub, center_sub], queue_size=1,slop=0.1, allow_headerless=True)

        #callback
        #ts.registerCallback(angle_regulator_callback,c)

        #subscribe to front_dist topic
        front_dist_topic="/front_dist"
        front_dist_sub=rospy.Subscriber(front_dist_topic,Float32,speed_regulator_callback,c)

        #define rate
        HZ=2
        rate=rospy.Rate(HZ)

        #assign frequency to control
        c.F=HZ

        #publish on angular and speed command topic
        
        if haut_niv==False:
            angle_pub=rospy.Publisher("/AngleCommand",Float32,queue_size=1)
            speed_pub=rospy.Publisher("/SpeedCommand",Float32,queue_size=1)

        command_pub=rospy.Publisher("/LidarSpeedAngleCommand",Float32MultiArray,queue_size=1)

        #subscribe MAE state
        rospy.Subscriber("/Nav_lid",Bool,onrun_callback,c)

        

        while not rospy.is_shutdown():
            if haut_niv==False:
                angle_pub.publish(c.ang)
                speed_pub.publish(c.speed)
            c.command.data=[c.speed, c.ang]

            command_pub.publish(c.command)

            rate.sleep()
        
        


        
    

    except rospy.ROSInterruptException:
        pass

    finally:
        print("exitting lidar navigation. All commands set to 0")
        
        #comment fair epour envoyer dernier msg de shutdown??????
        rospy.Publisher("/AngleCommand",Float32,queue_size=1).publish(0)
        rospy.Publisher("/SpeedCommand",Float32,queue_size=1).publish(0)
