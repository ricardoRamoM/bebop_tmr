#!/usr/bin/env python3
#throttle.py

# -----------------------------------
# Este nodo de ROS actúa como un "filtro" o "limitador de frecuencia" (throttler)
# para los comandos que llegan al tópico /bebop/command.
# En lugar de reenviar cada mensaje inmediatamente, los publica a una velocidad reducida (0.5 Hz = 1 mensaje cada 2 segundos)
# en el nuevo tópico /bebop/command_throttled.


# Entonces, aunque el dron reciba comandos continuamente en /bebop/command 
# (por ejemplo, cada 0.1 segundos), este nodo solo reenvía uno cada 2 segundos a /bebop/command_throttled.


# La frecuencia por eso se ajusta según el uso:
#   Automático con control PID o IA → mayor frecuencia (por ejemplo 10 Hz o más)
#   Modo de comandos discretos (takeoff, land, rotate) → menor frecuencia (0.5–1 Hz)

# El sistema automático puede generar comandos a cualquier velocidad.
#  pero el nodo throttle se encarga de enviarlos de forma segura, controlada y constante.

# El nodo throttle.py escucha el tópico: /bebop/command
# y publica (a menor frecuencia) en: /bebop/command_throttled
# Entonces, solo los mensajes que lleguen a /bebop/command pasan por el “filtro” o “limitador”.
#
# -----------------------------------

import rospy
from std_msgs.msg import String

# -----------------------------------
# Clase principal del nodo 
# -----------------------------------
class ThrottleCommandNode:
    def __init__(self):
        # Inicializa el nodo con el nombre 'throttle_command_node'
        # 'anonymous=True' permite lanzar múltiples instancias del nodo sin conflicto de nombres
        rospy.init_node('throttle_command_node', anonymous=True)

        # Define la tasa de publicación: 0.5 Hz = 1 mensaje cada 2 segundos
        self.rate = rospy.Rate(0.5)

        # Suscriptor al tópico original donde se publican comandos para el dron
        # Cada vez que llega un mensaje en /bebop/command, se llama a self.command_callback()
        self.sub = rospy.Subscriber('/bebop/command', String, self.command_callback)

        # Publicador al nuevo tópico /bebop/command_throttled
        # Este tópico tendrá los mismos mensajes, pero publicados a menor frecuencia
        self.pub = rospy.Publisher('/bebop/command_throttled', String, queue_size=1)

        # Variable para almacenar el último comando recibido
        self.last_command = None

    # -----------------------------------
    # Callback: se ejecuta cuando llega un nuevo mensaje a /bebop/command. Devolución de llamada de comando
    # -----------------------------------
    def command_callback(self, msg):
        # Guarda el último mensaje recibido
        self.last_command = msg

    def start(self):
        # Mientras ROS no se haya detenido (Ctrl+C o cierre del nodo)
        while not rospy.is_shutdown():
            # Si ya se recibió al menos un comando
            if self.last_command:
                # Publica ese último comando en el nuevo tópico "throttled"
                self.pub.publish(self.last_command)  
                rospy.loginfo(f"Last Command: {self.last_command}")
            # Espera el tiempo necesario para mantener la frecuencia de 0.5 Hz
            self.rate.sleep()  


# -----------------------------------
# Punto de entrada del programa
# -----------------------------------
if __name__ == '__main__':
    try:
        # Crea una instancia del nodo
        throttle_command_node = ThrottleCommandNode()
        # Inicia el bucle principal
        throttle_command_node.start()
    except rospy.ROSInterruptException:
        # Maneja la interrupción del programa (Ctrl+C)
        pass