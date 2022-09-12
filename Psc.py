#!/usr/bin/env python2

import rospy #librairie de ros
import math
import subprocess #librairie pour lancer des programmes a partir d'un script python
import signal
import os # librairie pour commander le systeme ie. terminer des programmes lancees etc...
import roslaunch # librairie pour lancer des fichiers ros de type launch (dans notre cas c'est le fichier launch du lidar donnee par l'entreprise)
import time
import RPi.GPIO as GPIO #pour le cordon de lancement et le choix de l'equipe

# importations des differents types de messages qui seronts utilises
from sensor_msgs import msg
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String
from geometry_msgs.msg import Point
from std_msgs.msg import UInt16


#TO RUN PROGRAM CONNECT TO RASPI VIA SSH AND RUN: rosrun ubiquity_motors Psy.py
#SI IL Y A UNE ERREU DE MASTER RUN DANS UN AUTRE TERMINAL: roscore

"""
On va creer deux classes pour controler le robot :

    La classe Mobile se chargera des commandes a envoyer au robot.
    Dans celle-ci on y trouvera le publisher '/instructions' qui nous permettra d'envoyer des informations a l'arduino
    et on trouvera aussi le subscriber 'pos' qui nous permettra de recuperer la position du robot de l'arduino 

    La classe mobile_lidar_fouille contiendra toutes les parties restantes du robot:
        Les topics '/inst_fouille' et '/etat_fouille' nous permettrons de comuniquer avec l'arduino charge de la fouille en envoyant des messages de type String
        Le topic '/bras' sera utilise pour comuniquer avec l'arduino du bras en envoyant des messages de type int 
        Le topic '/scan' sera utilise pour recuperer les donnees du lidar sous la forme de message de type LaserScan

"""

class Mobile(object): #Classe du deplacement
    largeurTable  = 2
    longueurTable  = 1.5
    
    posRobot  =[0,0,0]
    posRobotObjectif =[0,0,0]

    def __init__(self):
        #initialisation des subs et pubs
        self.pub=rospy.Publisher('/instructions',String,queue_size=50)
        self.sub=rospy.Subscriber('/pos',Point,self.position_callback)

    def position_callback(self,pos_msg):
        #recupere la position envoyee par l'arduino et l'enregistre dans l'array posRobot
        X = float(pos_msg.x)/1000000   #conversion
        Y = float(pos_msg.y)/1000000
        A = (float(pos_msg.z)/10000)*180/math.pi
        self.posRobot = [X,Y,A]
        print("en position  " + str(self.posRobot))

    def avancer(self,pos):   #pos = [X,Y,A] en metre et degre
        message = "D"+ str(int(pos[0]*1000000))+";"+str(int(pos[1]*1000000))+";"+str(int((pos[2]*10000)*math.pi/180))+";"
        print("avance en " + str(pos))
        self.pub.publish(message)
        rospy.wait_for_message('/pos',Point)
        print('avancer : done\n')
        
    def reculer(self,pos):   #pos = [X,Y,A] en metre et radian
        message = "R" + str(int(pos[0]*1000000))+";"+str(int(pos[1]*1000000))+";"+str(int((pos[2]*10000)*math.pi/180))+ ";"
        print("recule en " + str(pos))
        self.pub.publish(message)
        rospy.wait_for_message('/pos',Point)
        print('reculer : done\n')
        
    def stopper(self):
        message = "S"
        self.pub.publish(message)
        rospy.wait_for_message('/pos',Point)
        print('stopper : done\n')

    def updatePosition(self,pos):
        message = "N" + str(int(pos[0]*1000000))+";"+str(int(pos[1]*1000000))+";"+str(int((pos[2]*10000)*math.pi/180))+ ";"
        print("update de la position " +str(pos))
        self.pub.publish(message)
        rospy.wait_for_message('/pos',Point)
        print('updatePosition : done\n')
        
    def calageGauche(self):
        print("Calage position de depart gauche\n")
        self.updatePosition([0.2,1.3,0])
        self.calage(3,0.02)
        self.calage(2,0.02)
        print("Calage position de depart gauche : DONE\n")
    
    def retourMaison(self):
        print("Retour Maison\n")
        self.avancer([0.3,1.3,0])
        self.reculer([0.2,1.3,0])
        print("Retour maison: DONE\n")
        
    def positionFouille1(self):
        print("Direction carre de fouille 1\n")
        self.avancer([0.67,0.35,90])
        self.reculer([0.67,0.12,90])
        #rospy.sleep(5)
        #self.avancer([0.67,0.35,90])
        print("En position fouille 1\n")

    def calage(self,numeroMur,depasssement):    #se calle sur un mur update sa position et reviens
                                                #numeroMur :0 bas, 1 droit, 2 haut, 3 gauche , depassement:distance  rouler en
                                                #dehors du mur
        x = self.posRobot[0]
        y = self.posRobot[1]
        a = self.posRobot[2]
        profondeurTable = 2
        if numeroMur == 0:
            print("calage sur le mur du bas")
            self.reculer([x,-depasssement,90])
            self.updatePosition([x,0,90])
            self.avancer([x,y,a])

        if numeroMur == 1:
            self.reculer([self.largeurTable+depasssement,y,180])
            self.updatePosition([self.largeurTable,y,180])
            self.avancer([x,y,a])

        if numeroMur == 2:
            print("calage sur le mur du haut")
            self.reculer([x,profondeurTable+depasssement,-90])
            self.updatePosition([x,profondeurTable,-90])
            self.avancer([x,y,a])

        if numeroMur == 3:
            self.reculer([-depasssement,y,0])
            self.updatePosition([0,y,0])
            self.avancer([x,y,a])

#---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------

class mobile_lidar_fouille(object): #Classe pour le robot complet (lidar,bras,carre_fouille)
    mobile = Mobile()
    couleur="V"
    def __init__(self):
        """
        Creation des subs et pubs pour la fouille le lidar et le bras
        """
        pub_fouille=rospy.Publisher("/inst_fouille",String) #fouille
        scan_sub=rospy.Subscriber("/scan",LaserScan, self.scan_callback) #lidar
        pub_bras=rospy.Publisher('/bras',UInt16) #bras
        """
        Team Violette ou Jaune
        """
        #if(GPIO.input(channel)) ...

    """
    Fonction pour l'evitement
    """
    def scan_callback(self,scan_msg):
        angle_min= scan_msg.angle_min
        angle_max= scan_msg.angle_max
        increment= scan_msg.angle_increment
        ang_range=(int((math.pi/4-angle_min)/increment)+int((-math.pi/4-angle_min)/increment))/2
        counter_droite=0
        counter_gauche=0
        for i in range(int((-math.pi/4-angle_min)/increment),int((math.pi/4-angle_min)/increment)):
            if (scan_msg.ranges[i]<0.5 and i<=ang_range):
                counter_droite+=1
            elif (scan_msg.ranges[i]<0.5 and i>=ang_range):
                counter_gauche+=1
        if(counter_droite>counter_gauche):
            #evitement par la gauche
            self.mobile.stopper()
        else:
            #evitement par la gauche
            self.mobile.stopper()
    
    """
    Fonctions pour le bras
    """
    def depo_hor(self):
        print("Deposer l'echantillon a l'horizontal")
        self.pub_bras.publish(0)
    def prendre_hor(self):
        print("Prendre l'echantillon depose horizontalement")
        self.pub_bras.publish(1)
    def depo_vert(self):
        print("Deposer l'echantillon verticalement")
        self.pub_bras.publish(2)
    def prendre_vert(self):
        print("Prendre l'echantillon depose verticalement")
        self.pub_bras.publish(3)
    def retourner(self):
        print("Retourner l'echantillon")
        self.pub_bras.publish(4)
    """
    Fonction pour les carres de fouille
    """
    def basculer(self):        #couleur V ou J en  string
        print("Procedure carre de fouille, couleur : " + self.couleur)
        self.pub_fouille.publish(self.couleur)
        msg=rospy.wait_for_message("/etat_fouille",String)
        msg=msg.data
        if(msg == "B"):
            print("procedure bascule du carre de fouille\n")
            self.mobile.avancer([0.67,0.35,90])
            self.mobile.reculer([0.67,0.12,90])
            self.mobile.avancer([0.67,0.35,90])
            print("procedure bascule du carre de fouille : DONE\n")

        elif(msg == "R"):
            print("Pas le bon carre")

#---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------

def main():

    """
    Lancement du Serial et de la node et du lidar
    """
    #Lidar (commenter si lidar non utilise)
    
    uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
    roslaunch.configure_logging(uuid)
    cli_args = ['rplidar_ros', 'rplidar.launch']
    roslaunch_file = roslaunch.rlutil.resolve_launch_arguments(cli_args)
    parent = roslaunch.parent.ROSLaunchParent(uuid, roslaunch_file)
    parent.start()
    rospy.sleep(5)
    print("\n")
    
    #Serial (commenter les serial non utilisees)
    Serial1=subprocess.Popen('rosrun  rosserial_python serial_node.py __name:="node_mobile" /dev/ttyACM0', shell= True,preexec_fn=os.setsid) #base_mobile
    Serial2=subprocess.Popen('rosrun  rosserial_python serial_node.py __name:="node_fouille" /dev/ttyACM1', shell= True,preexec_fn=os.setsid) #fouille
    Serial3=subprocess.Popen('rosrun  rosserial_python serial_node.py __name:="node_bras" /dev/ttyACM2', shell= True,preexec_fn=os.setsid) #bras
    rospy.sleep(8) #attendre la connexion

    #Node:
    rospy.init_node('cerveau') #initialisation du script
    
    #Creation de l'objet robot:

    robot = Mobile()#je lance uniquement la base mobile ici
    #robot = mobile_lidar_fouille() #robot complet

    """
    Verification des connections avec le bras, la base mobile, et le Lidar
    """
    timer=0
    print("\n"+'Connection a la Base Mobile')
    while robot.pub.get_num_connections() < 1 : #changer en robot.mobile.pub.get_num_connections() pour robot complet
     # wait for a connection to publisher
        rospy.sleep(2)
        timer+=1
        if timer>3:
            raise Exception("Echec de la connection avec la Base Mobile")
    print("Connection etablie\n")

    print('Connection au Lidar')
    try:
        rospy.wait_for_message('/scan',LaserScan,timeout=1)# on attend pour "timeout" secondes pour voir si le lidar envoie des messages 
        print("Connection etablie\n")
    except:
        print("Echec de la connection au Lidar\n")

    #---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
    """
    Cordon de Lancement
    """
    #while(GPIO.input(channel)) ...

    """
    Instructions a donner au robot:
    """
    print("Debut du mouvement\n")

    #ALLER RETOUR:
    #robot.updatePosition([1,0,0])
    #robot.avancer([2,0,0])
    #robot.reculer([0,0,0])
    #robot.avancer([0,0,120])
    #robot.avancer([0,0,240])
    #robot.avancer([0,0,0])

    #MOUVEMENT EN CARRE:

    #robot.avancer([1,0,0])
    #robot.avancer([1,1,90])
    #robot.avancer([0,1,180])
    #robot.avancer([0,0,0])
    
    #DEPART ET POSITIONNENMENT SUR CARRE DE FOUILLE
    robot.calageGauche()
    robot.positionFouille1()
    robot.retourMaison()
    print("Deplacement termine")

    print('Robot shutting down')

    # Arreter les Serials
    os.killpg(os.getpgid(Serial1.pid), signal.SIGTERM)
    os.killpg(os.getpgid(Serial2.pid), signal.SIGTERM)
    os.killpg(os.getpgid(Serial3.pid), signal.SIGTERM)

#Verfication que le code a bien ete lance par l'utilisateur et pas par un autre script
if __name__ == '__main__':
    main()