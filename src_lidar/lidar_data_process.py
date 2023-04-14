#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
import numpy as np
import message_filters

from std_msgs.msg import Float32MultiArray, Float32, Bool

MIN_DIST=0.7
SAFETY_DIST=rospy.get_param("SAETY_DIST",default=0.5)

class Control:
    def __init__(self):
        self.angle_command=Float32()
        self.speed_command=Float32()
        self.dir=Float32()
        self.center=Float32()
        self.front_dist=Float32()
        self.offset=0.  #en faire parametre
        self.obstacle_ahead=False
        self.print=False #pour afficher msg de manoeuvre 1 fois par manoeuvre
        self.run=True


#navigation par defaut
def default_nav(front_data,quadran=[]):

    step_size=rospy.get_param("step_size",default=10)#step interval for front_data
    steps=len(front_data)//step_size
    avg,sum=0,0

    a0,a1=rospy.get_param("angle0",default=120),rospy.get_param("angle1",default=240)  #pour qualif sans obstacle mieux avec 130,230°
    
    if len(quadran)==0:
        angles=np.linspace(np.deg2rad(a0), np.deg2rad(a1), len(front_data))
    else: angles=quadran

    for i in range(steps):
        avg+=angles[i*step_size]*front_data[i*step_size]
        sum+=front_data[i*step_size]
    
    if sum!=0:
        avg/=sum
    else : avg=np.pi

    direction=avg-np.pi

    return direction

#SI IL Y A EVITEENT REFERMER LANGLE DE VISION POUR NE PAS REVENIR DIRECT
#SUR LE CHEMIN DE L'OBSTACLE 
#-> UN MOYEN D'EVITER CA SE SERAIT DE SMOOTH LES COMMANDES QUI PASSENT DE +1 A -1 ETC
#COMME CA SI IL EVITE OBSTACLE EN PARTANT A GAUCHE ET ENSUITE IL VEUT ALLER 
#A DROITE LA COMMANDE SERA PAS AUSSI BRUSQUE 
def quadran_nav(front_data,N):
    rospy.loginfo("MANOEUVRE D'EVITEMENT")
    step_size=rospy.get_param("step_size",default=10)#step interval for front_data
    steps=len(front_data)//step_size

    quadran_size=len(front_data)//N #taille d'un cadran 

    a0,a1=rospy.get_param("angle0",default=120),rospy.get_param("angle1",default=240) #ON PEUT MODIFIER CES ANGLES EN FONCTION DU QUADRAN OU SE TROUVE OBSTACLE
    angles=np.linspace(np.deg2rad(a0), np.deg2rad(a1), len(front_data))


    #TESTER NOUVEL ALGO SUR ROBOT EN SIMU PAS TOP MAIS PEUT ETRE EN VRAI C EST PAS MAL

    sums=np.zeros(N)
    for n in range(N):
        #ajouter penalite si certaines dit sont <safety_dist
        if n==0:
            sums[n]=sum(front_data[:quadran_size:step_size])
        elif n==N-1:
            sums[n]=sum(front_data[n*quadran_size::step_size])
        else: 
            sums[n]=sum(front_data[n*quadran_size:(n+1)*quadran_size:step_size])
    
    sums[n//2]=0 #on veut pas regarder ua centre
    
    #if steps//(N-1)<N : avgs[N-1],sums[N-1]= angles[(N-1)*step_size]*front_data[(N-1)*step_size],front_data[(N-1)*step_size]
    #ca marche mieux en simu sans la 3eme composante
    
    #print(sums,"\n")
    best_ind=np.argmax(sums)

    if best_ind==0:
        best_quadran=front_data[:quadran_size]
        angles=angles[:quadran_size]
    elif best_ind==N-1:
        best_quadran=front_data[best_ind*quadran_size:]
        angles=angles[best_ind*quadran_size:]
    else:
        best_quadran=front_data[best_ind*quadran_size:(best_ind+1)*quadran_size]
        angles=angles[best_ind*quadran_size:(best_ind+1)*quadran_size]

    direction=default_nav(best_quadran,angles)#avg-np.pi
    return direction


def analyze_front(front_data,c):
    #calcule la distance moyenne a l'avant et chercho obstacle pour mode evitement ou default
    
    step_size=rospy.get_param("step_size",default=10)

    a0,a1=rospy.get_param("~angle0",default=120),rospy.get_param("~angle1",default=240) #ON PEUT MODIFIER CES ANGLES EN FONCTION DU QUADRAN OU SE TROUVE OBSTACLE
    i0,i1=rospy.get_param("i0",default=165),rospy.get_param("i1",default=195) 

    
    angles=np.linspace(a0,a1,len(front_data))

    ind0=np.where(angles>=i0)[0][0]
    ind1=np.where(angles>=i1)[0][0]

    front_dist=0
    r=range(ind0,ind1,3)
    for i in r:
        if front_data[i]<SAFETY_DIST and front_data[i]>0: c.obstacle_ahead=True #front_data[i]>0 pour eviter bug en reel
        
        front_dist+=front_data[i]
    
    front_dist/=len(r) #on moyenne sur le nomnre de points utilises
    
    #!!!!RECUPERER INDICE OBSTACLE -> LOCALISATION -> S'ELOIGNER ->EVITER DE CHOSIR UN CADRAN QUI A PLUS DE SITANCE ALORS QUE L'OBSTACLE S'Y TROUVE
    if np.all(np.array(front_data[ind0:ind1])>SAFETY_DIST) and c.obstacle_ahead==True:
        c.obstacle_ahead=False
    
    #print(front_dist,c.obstacle_ahead)
    return front_dist


def data_process_callback(msg_f,msg_s,c):
    #PARCOURIR LE TABLEAU DANS msg.data ET DETERMINER LA DIRECTION INSTANTANNEE A PRENDRE 
    #ET LA VITESSE QU'ON PEUT AVOIR

    #idee : parcourir le tableau et determiner dans quelle direction les distances sont les  plus
    #importantes-> moyenne ponderee des angles p.ex
    #-> parcourt le tableau des angles et on le multiplie par la distance, on fait la moyenne puis sur le nombre total
    #de distances et ca devrait nous donner la direction a prendre
    #
    #step_size=rospy.get_param("step_size",default=10)#step interval for front_data 

    #si lidar nav par MAE
    if c.run:
        front_data=msg_f.data #tableau contient dist autour du robot entre [a0,a1]

        if len(front_data)!=0:
            #si objet au milieu <-> front_data[diag_gauche]>front_data[n//2]<front_data[diag_droite]
            #probleme est qu'en faisant moyenne la direction a prendre est quand meme le milieu du a la symetrie
            #devier-> ajouter offfset a avg qui fera qu'en faisant moy on ira vers gauche ou droite
            #METTRE EN PLACE UN FLAG RECUPERE DANS nv_control QUI AJUSTE VITESSE ET COMMANDE DE BRAQUAGE
            
            
            front_dist=analyze_front(front_data,c)
            
            
            #if front_dist>SAFETY_DIST and c.obstacle_ahead==True: c.obstacle_ahead=False
            
            c.front_dist=front_dist

            
            N=rospy.get_param("N_cadrans",default=3) 
            
            direction=quadran_nav(front_data,N) if c.obstacle_ahead==True else default_nav(front_data)#avg-np.pi #>0 : droite, <0 : gauche => donne la direction a prendre

            #equivalent to orientation error
            c.dir=direction-c.offset #-offset car defini t.q. offset>0 => gauche


        side_data=msg_s.data
        #equivalent to centering error
        c.center=side_data[0]-side_data[1] +c.offset if len(side_data)!=0 else 0  #left-right + offset
        
    
def onrun_callback(msg,c):
    c.run=msg.data  


if __name__=='__main__':
    try:
        
        #init node
        rospy.init_node("lidar_data_process_node")

        #init control
        c=Control()


        #subscribe to front_data and side_data
        front_data_topic=rospy.get_param("~front_data_topic", default="/front_data")
        side_data_topic=rospy.get_param("~side_data_topic", default="/side_data")

        front_sub=message_filters.Subscriber(front_data_topic,Float32MultiArray, queue_size=1)
        side_sub=message_filters.Subscriber(side_data_topic,Float32MultiArray,queue_size=1)

        ts = message_filters.ApproximateTimeSynchronizer([front_sub, side_sub], queue_size=1, slop=0.1, allow_headerless=True)
        ts.registerCallback(data_process_callback,c)
  
        #subscribe MAE state
        rospy.Subscriber("/Nav_lid",Bool,onrun_callback,c)

        #publish commands - test
        rate=rospy.Rate(10)


        #publish direction
        dir_pub=rospy.Publisher("/lidar_dir",Float32,queue_size=1)

        #publish center
        center_pub=rospy.Publisher("/lidar_center",Float32,queue_size=1)

        #publidhs front dist
        front_dist_pub=rospy.Publisher("/front_dist",Float32,queue_size=1)

        while not rospy.is_shutdown():

            dir_pub.publish(c.dir)
            center_pub.publish(c.center)
            front_dist_pub.publish(c.front_dist)

            rate.sleep()
        
        


    except rospy.ROSInterruptException:
        pass