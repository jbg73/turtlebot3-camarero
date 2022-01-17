#!/usr/bin/python
# -*- coding: utf-8 -*-
import rospy
from practica3.msg import mensajeInt
import sys

# Este nodo publica continuamente el destino al que debe ir el robot
# 0(home) , 1 (mesa 1) , 2 (mesa 2)
def nodo_envia(destino):
    # Se crea el mensaje que se va a publicar que es de tipo mensajeInt
    # que contiene un unico campo que es de tipo int32
    mensaje = mensajeInt()
    mensaje.dst = destino

    # indica en que topic va a publicar y se inicializa
    pub = rospy.Publisher('/topic_set_destiny', mensajeInt, queue_size=10)
    rospy.init_node("nodo_publicador_destino", anonymous=True)
    rate = rospy.Rate(10)

    # mientras no se detenga la ejecucion, publica sin parar
    while not rospy.is_shutdown():
        pub.publish(mensaje)
        rate.sleep()


if __name__ == "__main__":
    # captura el argumento de entrada
    destino = int(sys.argv[1])
    try:
        nodo_envia(destino)
    except rospy.ROSInterruptException:
        pass
