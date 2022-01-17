#!/usr/bin/python
# -*- coding: utf-8 -*-
import rospy
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from actionlib_msgs.msg import GoalStatus
import sys

import roslib
import cv2
import numpy as np 
from cv_bridge import CvBridge, CvBridgeError
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image
import math
from practica3.msg import mensajeInt

# Variable global que se usara para comprobar si se ha llegado al destino
ARRIVED = 0

# Definicion de las posiciones de interes: home, mesa 1 y mesa 2
##POSES DE LAS MESAS##
#Mesa1: x: -3.832 y: -1.5689 | x: 0.00159 y: 0.0 z: -0.9983 w: -0.0579
#Estacion: x: 3.0619 y: -3.4841 |  x: -0.0009 y: 0.0013 z: 0.5691 w: 0.82226
#Mesa2: x: 2.0827 y: 5.2294 | x: 0.0016 y: 0.0 z: -0.9987 w: 0.0508

# Clase Mesa: creada para crear las distintas poses con este formato
# incluye la posicion x,y y la orientacion del cauternio
class Mesa:
    def __init__(self,xp,yp,xq,yq,zq,wq):
        self.xp = xp
        self.yp = yp
        self.xq = xq
        self.yq = yq
        self.zq = zq
        self.wq = wq

#Uso de la acción move_base en ROS para moverse a un punto determinado
#En ROS una acción es como una petición de un "cliente" a un "servidor"
#En este caso este código es el cliente y el servidor es ROS
#(en concreto el nodo de ROS 'move_base')
class ClienteMoveBase:
    def __init__(self):
        #creamos un cliente ROS para la acción, necesitamos el nombre del nodo 
        #y la clase Python que implementan la acción
        #Para mover al robot, estos valores son "move_base" y MoveBaseAction
        self.client =  actionlib.SimpleActionClient('move_base',MoveBaseAction)
        #esperamos hasta que el nodo 'move_base' esté activo`
        self.client.wait_for_server()

    # Metodo para mover al robot a un destino
    def moveTo(self, dst):
        #un MoveBaseGoal es un punto objetivo al que nos queremos mover
        goal = MoveBaseGoal()
        #sistema de referencia que estamos usando
        goal.target_pose.header.frame_id = "map"
        
        # Se completan los distintos campos del goal en funcion del destino definido
        goal.target_pose.pose.position.x = dst.xp   
        goal.target_pose.pose.position.y = dst.yp
        goal.target_pose.pose.orientation.x = dst.xq
        goal.target_pose.pose.orientation.y = dst.yq
        goal.target_pose.pose.orientation.z = dst.zq
        goal.target_pose.pose.orientation.w = dst.wq

        #enviamos el goal 
        self.client.send_goal(goal)
        #vamos a comprobar cada cierto tiempo si se ha cumplido el goal
        #get_state obtiene el resultado de la acción 
        state = self.client.get_state()
        #ACTIVE es que está en ejecución, PENDING que todavía no ha empezado
        while state==GoalStatus.ACTIVE or state==GoalStatus.PENDING:
            rospy.Rate(10)   #esto nos da la oportunidad de escuchar mensajes de ROS
            state = self.client.get_state()
        return self.client.get_result()

# Esta funcion se ejecuta cada vez que se publica un nuevo mensaje en el topic de 
# confirmacion de llegada al destino
def callback(data):
    # Si se ha llegado, se cambia el valor de la variable global que se usara para
    # salir del bucle en el main
    if data.dst == 1:
        global ARRIVED
        ARRIVED = 1
        
# se suscribe al topic donde se confirma que se ha llegado 
def wait_for_confirmation_qr():
    rospy.Subscriber("topic_arrived",mensajeInt,callback)
    
if __name__ == "__main__":
    if len(sys.argv) <= 1:
        print("Uso: " + sys.argv[0] + " 0(estacion) 1(mesa1) 2(mesa2)")
        exit()      

    # Define el nodo y las 3 posiciones obtenidas del mapeado
    rospy.init_node('prueba_clientemovebase')

    mesa1 = Mesa(-3.832,-1.5689,0.00159,0.0,-0.9983,-0.0579)

    mesa2 = Mesa(1.801,4.998,-0.00159,0.0,0.9999,-0.0402)

    estacion = Mesa(3.0619,-3.4841,-0.0009,0.0013,0.5691,0.8222)
    
    # En funcion del argumento de entrada (destino), el destino se iguala a una pose distinta
    if int(sys.argv[1]) == 0:
        destino = Mesa(estacion.xp,estacion.yp,estacion.xq,estacion.yq,estacion.zq,estacion.wq)

    elif int(sys.argv[1]) == 1:
        destino = Mesa(mesa1.xp,mesa1.yp,mesa1.xq,mesa1.yq,mesa1.zq,mesa1.wq)

    elif int(sys.argv[1]) == 2:
        destino = Mesa(mesa2.xp,mesa2.yp,mesa2.xq,mesa2.yq,mesa2.zq,mesa2.wq)
    
    # Se crea el cliente de la accion y se le indica que inicie su movimiento
    cliente = ClienteMoveBase()
    result = cliente.moveTo(destino)

    # Cuando llega al destino, espera en un bucle la confirmacion del nodo de vision
    while ARRIVED == 0:
        wait_for_confirmation_qr()
    
    print("Se ha llegado a la posicion DESTINO...")
    print("Volviendo a home...")
    # Se le envia a la posicion home
    result = cliente.moveTo(estacion)
    
    print(result)
    if result:
        rospy.loginfo("Goal conseguido!")