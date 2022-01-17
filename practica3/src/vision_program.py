#!/usr/bin/env python
import roslib
import sys
import rospy
import cv2
import numpy as np 
from cv_bridge import CvBridge, CvBridgeError
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image
import math
from practica3.msg import mensajeInt
import time

# Variable global que hace referencia al destino
# Se usa para comprobar si se ha leido bien el destino enviado como argumento
DESTINO = -1

# Clase que contiene los metodos usados para las tareas de vision
class CameraVisualization:
    # Constructor: crea un objeto y lo suscribe al topic donde se publican las imagenes capturadas por la camara
    def __init__(self):
        self.bridge_object = CvBridge()
        self.image_sub = rospy.Subscriber("/camera/rgb/image_raw", Image,self.camera_callback)

    # callback: se ejecuta cada vez que se publica un mensaje nuevo en el topic al que esta suscrito
    def camera_callback(self, data):
        # mensaje que se publicara cuando se detecte que se ha llegado al destino
        ARRIVED = mensajeInt()
        ARRIVED.dst = 0
        
        # Convertir la imagen a bgr para poder usar OpenCV
        try:
            cv_image = self.bridge_object.imgmsg_to_cv2(data, desired_encoding="bgr8")
        except CvBridgeError as e:
            print(e)
        
        # Se redimensiona la imagen y se intenta detectar un QR en la imagen obtenida
        cv_image = cv2.resize(cv_image, (int(cv_image.shape[1] * 35 / 100),int(cv_image.shape[0] * 35 / 100)), interpolation = cv2.INTER_AREA)
        qrCodeDetector = cv2.QRCodeDetector()
        decodedText, points, _ = qrCodeDetector.detectAndDecode(cv_image)
    
        global DESTINO
        # Si ha detectado algo
        if points is not None:
            nPoints = len(points)
            print(nPoints)
            #cv2.line(cv_image,tuple(points[0][0]), tuple(points[1][0]),(255,0,0),5)
            #print(points)

            # Calcula el valor del area del cuadrado que envuelve el QR
            area = pow(np.sqrt(pow(points[0][0][0]-points[1][0][0],2) +  pow(points[0][0][1]-points[1][0][1],2)),2)
            print(area)

            # Si el texto detectado no esta vacio ( a veces hace falsas detecciones de QR donde el texto extraido es nulo)
            if decodedText is None:
                print("El texto esta vacio")

            # Si el area supera cierto valor, confirma que ha llegado
            if area > 30000.0:
                if decodedText != "":
                    if int(decodedText) > 0:
                        if DESTINO == int(decodedText):
                            # publicar en el topic pertinente un 1 en referencia a que ha llegado a su destino
                            ARRIVED.dst = 1
                            pub = rospy.Publisher('/topic_arrived',mensajeInt,queue_size=10)
                            pub.publish(ARRIVED)
                            print("Estoy cerca de la mesa...", decodedText)

        # Muestra la imagen por pantalla        
        cv2.imshow("Image Window", cv_image)
        cv2.waitKey(1)

# Esta funcion se ejecuta cada vez que se lea un nuevo mensaje en el topic en que se publica el destino
def callback(data):
    #print("TOPIC SET_DESTINY ACTIVADO")
    #print("Recibo: ",data.dst)

    # Actualiza el valor de la variable global DESTINO que se utiliza en la clase que hace las tareas de vision
    global DESTINO 
    DESTINO = data.dst
    #print("Destino Recibido: ", DESTINO)

def main():    
    camera_visualization_object = CameraVisualization()

    # Inicializa el nodo y se suscribe al topic donde se publica el destino
    rospy.init_node('camera_visualization_node', anonymous=True)
    rospy.Subscriber("topic_set_destiny",mensajeInt,callback)

    # Entra en un bucle para poder disponer de info visual en tiempo real
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main()