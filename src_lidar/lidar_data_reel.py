#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
import numpy as np

from std_msgs.msg import Float32MultiArray, Bool
from sensor_msgs.msg import LaserScan

CLIP_DIST=3

class Control():
    def __init__(self):
        self.front_dist=Float32MultiArray()
        self.front_angle=[]
        self.side_dist=Float32MultiArray()
        self.topic=None
        self.run=True

#We're listenning to the /LidarScan topic, processing the data and publishing to /front_data and /side_data

def interpolate(data, ind):

    for i in range(len(data)): #on interpole 1x par scan
        if data[i]==float('inf'):
    
            s,e=i,i
            #rospy.loginfo(f"ind={i}")
            #on cherche debut
            while data[s]==float('inf'):
                s-=1

            #if s==-1:s=0
            #chercher fin du trou   
            while e<=len(data)-1 and data[e]==float('inf'):
                e+=1

            if e==len(data):e=len(data)-1

            dx=e-s
            dy=data[e]-data[s]
            x0,y0=s,data[s]

            #print(s,e,dx,dy)
            #interpolation du trou
            if dx!=0: #safety
                for j in range(s+1,e):
                    
                    data[j]=data[s] + dy/dx * (j-s)
            
            i=e

    
    #FAIRE ATTENTIO DES FOIS INTERPOLATION COMPLETE DES TROUS QUI SONT VRM PRESENTS DANS LE CIRCUIT
    #PAS CRITIQUE MAIS FAUT FAIRE ATTENTION-> CONDITION DE SECURITE 
    # -> P.EX. SI LA PENTE DY/DX EST TROP GRANDE => PROBABLEMENT VRAI TROU


   
    
    return data #interpolated data with no holes


def lidar_preprocess_callback(msg,c):
    if c.run:
        scan=[]
        #on recupere bonne data 
        if c.topic=="/scan":
            scan=np.array(msg.ranges) 
        elif c.topic=="/LidarScan":
            scan=np.array(msg.data)
        #scan reel a devant a 0 donc on shift tous l'array de n/2 pour garder les autres noeuds focntionnels
        n=len(scan)

        #si /scan
        if c.topic=="/scan":
            scan=np.roll(scan,-n//2)#on shift l'array pour avoir devant a n//2
            scan=np.flip(scan)#on inverse l'ordre -> gauche droite

        intv=[int((rospy.get_param("angle0",default=120))/90 * n/4),
            int(rospy.get_param("angle1",default=240)/270 * 3*n/4)] #angles interval
        
        
        angles=np.linspace(0,2*np.pi,n)
        c.front_angles=angles[intv[0]:intv[1]] #on va pas regarder tout le cadran avant mais, a nouveau, un sous ensmeble
        
        c.front_dist.data=np.zeros(len(c.front_angles))
        iter=0
        scan=interpolate(scan,0) #on interpole une fois
        
        for i in range(n)[intv[0]:intv[1]]:
            #PLUS BESOIN DU IF-ELSE CAR ON INTERPOLE TOUTE LA DATA UNE SEULE FOIS
            if scan[i]!=float('inf'):
                c.front_dist.data[i-intv[0]]=np.clip(scan[i],0,CLIP_DIST) #on clip pour s'orienter par rapport a l'env "local"
            
            else:
                #chercher la bonne valeur a prendre
                #on a inf pour certaines sections du scan -> on va interpoler lineairement (approx) pour completer ces points
                
                scan=interpolate(scan,i)
                iter+=1
                print(iter)
                c.front_dist.data[i-intv[0]]=(np.clip(scan[i],0,CLIP_DIST))

        #side data pour rester au milieu
        c.side_dist.data=[0,0]
        for ind,i in enumerate([n//4,3*n//4]):
            if scan[i]!=float('inf'):
                c.side_dist.data[ind]=(np.clip(scan[i],0,CLIP_DIST)) 
            else:
                scan=interpolate(scan,i)
                c.side_dist.data[ind]=(np.clip(scan[i],0,CLIP_DIST))

        
        #print(len(c.front_dist.data))       
            


def onrun_callback(msg,c):
    c.run=msg.data           
    

if __name__=='__main__':

    try:
        #init le noeud
        rospy.init_node("LidarData_node")

        #topic d'ecoute
        lidar_topic = rospy.get_param("~lidar_topic",default="/LidarScan")    #en faire un parametre de launchfile

        #classe de controle, stock data etc
        c=Control()
        c.topic=lidar_topic
        
        #subscriber sur lidar topic qui range dans control la valeur du front data
        #si reel
        if lidar_topic=="/scan":
            rospy.Subscriber(lidar_topic, LaserScan, lidar_preprocess_callback,c)
        
        #si simu
        elif lidar_topic=="/LidarScan":
            rospy.Subscriber(lidar_topic, Float32MultiArray, lidar_preprocess_callback,c)

        #subscribe MAE state
        rospy.Subscriber("/Nav_lid",Bool,onrun_callback,c)




        #publisher de front data
        front_topic=rospy.get_param("~front_topic",default="/front_data")    
        front_pub=rospy.Publisher(front_topic,Float32MultiArray,queue_size=10)
        
        #publisher side_data
        side_topic=rospy.get_param("~side_topic",default="/side_data") 
        side_pub=rospy.Publisher(side_topic,Float32MultiArray,queue_size=10)

        rate=rospy.Rate(10)
        #COMMENT BIEN SYCHRONISER LA PUBLICATION? -> LIMITE PAR /LidarScan -> pas besoin de rate.sleep?

        while not rospy.is_shutdown():

            
            front_pub.publish(c.front_dist)
            side_pub.publish(c.side_dist)
            
            rate.sleep()
    
    except rospy.ROSInterruptException:
        pass
