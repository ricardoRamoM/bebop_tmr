#!/usr/bin/env python3
# camera_node.py

# -----------------------------------
# Se encarga de recibir imágenes del dron Parrot Bebop, procesarlas mediante una clase 
# llamada BebopCameraProcessor, mostrar el video procesado en una ventana y 
# publicar comandos (por ejemplo “takeoff”, “land”, etc.) según lo que detecte la cámara.
# 
# Los comandos se publican en '/bebop/command'
# -----------------------------------

import rospy                       # Librería principal de ROS en Python
import cv2                         # OpenCV para procesamiento de imágenes
import os, sys                     # Manejo de rutas y sistema
from cv_bridge import CvBridge     # Convierte entre ROS Image y OpenCV
from sensor_msgs.msg import Image  # Tipo de mensaje para imágenes
from std_msgs.msg import String    # Tipo de mensaje para comandos en texto


# ------------------------------------------------------------
# Configuración de ruta para importar el archivo camera_class.py
# Import and find BebopCameraProcessor class
# ------------------------------------------------------------

# Obtiene la ruta actual de este archivo
current_dir = os.path.dirname(os.path.abspath(__file__))
# Retrocede un nivel (la carpeta raíz del proyecto)
project_root = os.path.abspath(os.path.join(current_dir, '..'))
# Agrega esa ruta al sistema para poder importar la clase personalizada
sys.path.append(project_root)

# ------------------------------------------------------------
# Clase principal del nodo de cámara del dron
# ------------------------------------------------------------
from scripts.classes.camera_class import BebopCameraProcessor


class BebopCameraNode:
    def __init__(self):
        # Inicializa el nodo ROS
        rospy.init_node('bebop_camera_node', anonymous=True)

        # Crea el objeto puente entre ROS Image y OpenCV
        self.bridge = CvBridge()

        # Crea un publicador en el tópico /bebop/command
        # donde se enviarán comandos como "takeoff", "land", "forward", etc.
        self.command_pub = rospy.Publisher('/bebop/command', String, queue_size=1)

        # Intenta inicializar el procesador de cámara de la clase camera_class
        try:
            self.processor = BebopCameraProcessor()
        except Exception as e:
            rospy.logerr(f"Error initializing BebopCameraProcessor: {e}")
            self.processor = None

        # Se suscribe al tópico de imágenes del dron
        # Cada nueva imagen llamará automáticamente a image_callback()
        self.image_sub = rospy.Subscriber('/bebop/image_raw', Image, self.image_callback)

        # Mantiene el nodo corriendo (escuchando los tópicos)
        rospy.spin()


    # ------------------------------------------------------------
    # Callback que se ejecuta cada vez que llega una imagen del dron
    # ------------------------------------------------------------
    def image_callback(self, msg):
        # Si el procesador no se pudo inicializar, ignora las imágenes
        if self.processor is None:
            rospy.logerr("Processor not initialized. Skipping image processing.")
            return
        

        try:
            # Convierte el mensaje ROS a una imagen de OpenCV (BGR)
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")

            # Process image using the camera_class processor. Llama al método process_image de BebopCameraProcessor
            # que devuelve:
            # - processed_image: la imagen analizada
            # - command: un comando de texto (o None si no hay)
            processed_image, command = self.processor.process_image(cv_image)

            # Muestra la imagen procesada en una ventana
            cv2.imshow("Bebop Camera", processed_image)
            cv2.waitKey(1) # Necesario para refrescar la ventana

            # Publish the command if any. Si el procesador generó un comando, publícalo
            if command:
                rospy.loginfo(f"Command: {command}")
                self.command_pub.publish(command)

        except Exception as e:
            # Si ocurre un error en el procesamiento o conversión
            rospy.logerr(f"Error processing image: {e}")

# ------------------------------------------------------------
# Ejecución principal del nodo
# ------------------------------------------------------------
if __name__ == '__main__':
    try:
        BebopCameraNode() # Crea e inicia el nodo
    except rospy.ROSInterruptException:
        # Captura la interrupción si el usuario presiona Ctrl + C
        pass