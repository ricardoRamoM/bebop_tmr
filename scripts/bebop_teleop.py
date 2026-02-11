#!/usr/bin/env python3
# bebop_teleop.py

# Control manual y automático de un dron Parrot Bebop mediante teclado y ROS.

import os
import sys
import rospy
import select
import termios
import tty
from std_msgs.msg import Empty, String
from geometry_msgs.msg import Twist

# Importa la clase BebopMovements desde la carpeta "scripts/classes"
current_dir = os.path.dirname(os.path.abspath(__file__))
project_root = os.path.abspath(os.path.join(current_dir, '..'))
sys.path.append(project_root)
from scripts.classes.bebop_movements import BebopMovements

# ------------------------------
# DICCIONARIOS DE TECLAS
# ------------------------------
# Relaciona teclas con vectores de movimiento (x, y, z, rotación angular)
# Los valores son del % de la vel maxima, osea 1=100%
moveBindings = {
    'w': (1, 0, 0, 0),    # Adelante
    'a': (0, 1, 0, 0),    # Izquierda
    'd': (0, -1, 0, 0),   # Derecha
    's': (-1, 0, 0, 0),   # Atrás
    'q': (0, 0, 0, 1),    # Girar izquierda
    'e': (0, 0, 0, -1),   # Girar derecha
    '+': (0, 0, 1, 0),    # Subir
    '-': (0, 0, -1, 0),   # Bajar
}

# Controles para mover la cámara del dron
cameraBindings = {
    'i': (0, 5),    # Inclinación hacia arriba (tilt)
    'k': (0, -5),   # Inclinación hacia abajo
    'j': (-5, 0),   # Pan izquierda
    'l': (5, 0),    # Pan derecha
}

# Mensaje de ayuda que se muestra en consola
msg = """
---------------------------
   q    w   e       +: up
   a        d       -: down
        s   
---------------------------
Take off: 1
Land: 2
---------------------------
Teleop mode: t
Automatic mode: y
---------------------------
Cam control:
---------------------------
   j    i    l       
        k    
---------------------------
CTRL-C to exit and land.
"""

# ===========================================================
# CLASE PRINCIPAL DE CONTROL
# ===========================================================
class BebopTeleop:
    def __init__(self):
        # Inicializa el nodo ROS
        rospy.init_node('teleop_control')

        # Publicadores para los distintos comandos del dron
        self.pub = rospy.Publisher('bebop/cmd_vel', Twist, queue_size=1)          # Movimiento
        self.pub_takeoff = rospy.Publisher('bebop/takeoff', Empty, queue_size=10) # Despegue
        self.pub_land = rospy.Publisher('bebop/land', Empty, queue_size=10)       # Aterrizaje
        self.pub_camera = rospy.Publisher('bebop/camera_control', Twist, queue_size=1) # Cámara

        # Crea objeto para manejar movimientos de aterrizaje, despegue y los movimientos automaticos (usa BebopMovements)
        self.movements = BebopMovements(self.pub, self.pub_takeoff, self.pub_land, self.pub_camera)

        # Seleccion del modo inicial 
        self.mode_flag = 'telepop'  # Podría ser 'automatic' o 'teleop'

        # Subscripción al tópico de comandos automáticos
        rospy.Subscriber('/bebop/command_throttled', String, self.command_callback, queue_size=1)

        # Guarda configuración del teclado (para restaurarla luego)
        self.settings = termios.tcgetattr(sys.stdin)

        # Posiciona la cámara al iniciar
        self.init_camera_position()

    # ----------------------------------------------------------
    # Inicializa la cámara mirando hacia abajo, y luego ajusta
    # ligeramente el ángulo para visualizar al frente.
    # ----------------------------------------------------------
    def init_camera_position(self):
        self.movements.camera_tilt(-90)  # Mira totalmente hacia abajo
        rospy.sleep(2)
        self.movements.initial_takeoff(self.mode_flag)  # (Despegue inicial opcional)
        rospy.sleep(2)
        self.movements.camera_tilt(-5)   # Ajusta para ver hacia el frente, antes era 10
        rospy.sleep(2)
        rospy.loginfo("Inicio Ejecutado")


    # ----------------------------------------------------------
    # CALLBACK para modo AUTOMÁTICO
    # Recibe comandos de texto desde /bebop/command_throttled
    # ----------------------------------------------------------
    def command_callback(self, msg):
        # Debido a que si por alguna razon /bebop/command_throttled recibe un mensaje y no está en modo automatico
        # se debio a algun error porque command throttle solo se ejecuta cuando '/bebop/command' cambia y esto cambia
        # de acuerdo al procesamiento de la camara que decide moverse o no en MODO AUTOMATICO.

        # Y como el callback se manda a llamar cada vez que hay algun cambio en el topico, por eso se pone esto, por seguridad
        if self.mode_flag != 'automatic':
            return  # Ignora si no está en modo automático

        command = msg.data
        rospy.loginfo(f"Comando recibido: {command}")

        # Mapeo de comandos automáticos con métodos de movimiento
        command_to_method_mapping = {
            'w': 'forward',
            'a': 'left',
            'd': 'right',
            's': 'backwards',
            '+': 'up',
            '-': 'down',
            'q': 'turn_left',
            'e': 'turn_right',
        }

        # Si en command_throttle recibe el 2 va a aterrizar
        if command == '2':  # Aterriza
            self.movements.landing(self.mode_flag)
            rospy.loginfo("Landing...")

        elif command in command_to_method_mapping:
            # Llama al método correspondiente en BebopMovements
            method_name = command_to_method_mapping[command] # Traduce la tecla/comando recibido a un nombre de función.
            
            # getattr(obj, 'nombre_metodo') devuelve el método del objeto cuyo nombre es el string dado. P.E.:
                #method_name = 'forward'
                #movement_method = getattr(self.movements, 'forward') 
                # Ahora movement_method apunta a self.movements.forward
            #Esto permite llamar a métodos dinámicamente sin usar un montón de if-else.
            movement_method = getattr(self.movements, method_name)

            rospy.loginfo(f"Moving: {method_name}")

            #self en método -> Siempre se pasa automáticamente cuando llamas obj.metodo(...), 
            # por lo que al usarlo solo colocas el resto de argumentos
            movement_method(self.mode_flag)


        # Si el command_throttle recibe algo como lo que dice cameraBindings: 'i', 'k', 'j', 'l'
        # esto hará que se mueva como lo indica los valores predeterminados de cameraBindings  
        elif command in cameraBindings:
            # Se mueve si el mensaje recibido coincide con cameraBindings
            pan, tilt = cameraBindings[command]
            if pan != 0:
                self.movements.camera_pan(pan)
            if tilt != 0:
                self.movements.camera_tilt(tilt)
            rospy.loginfo(f"Moving cam: {command}")

        else:
            rospy.loginfo(f"Unknown command: {command}")


    # ----------------------------------------------------------
    # Lee una tecla sin esperar ENTER (modo raw del teclado)
    # ----------------------------------------------------------
    def getKey(self):
        tty.setraw(sys.stdin.fileno())
        select.select([sys.stdin], [], [], 0)
        try:
            key = sys.stdin.read(1)
        except Exception:
            key = ''
        finally:
            # Restaura configuración normal del teclado
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)
        return key

    # ----------------------------------------------------------
    # BUCLE PRINCIPAL: lee las teclas y actúa según el modo
    # ----------------------------------------------------------
    def run(self):
        while not rospy.is_shutdown():
            key = self.getKey()

            # Cambia a modo teleop (manual)
            if key == 't':
                self.mode_flag = 'teleop'
                print("\n--- TELEOP mode activated ---")
                print(msg)
                print("Press 'y' to enter automatic mode.")

            # Cambia a modo automático
            elif key == 'y':
                self.mode_flag = 'automatic'
                print("\n--- AUTOMATIC mode activated --- ")
                print("Press 't' to enter teleop mode.")
                self.movements.reset_twist()  # Detiene cualquier movimiento residual

            # Ctrl+C → aterriza y sale
            elif key == '\x03':
                print("\nLanding before exiting...")
                self.movements.landing(self.mode_flag)
                rospy.signal_shutdown("\nEnded by Ctrl-C")
                break

            # Si está en modo TELEOP, procesa las teclas manuales
            elif self.mode_flag == 'teleop':
                if key == '1':
                    self.movements.initial_takeoff(self.mode_flag)
                    print("\nTaking off...")

                elif key == '2':
                    self.movements.landing(self.mode_flag)
                    print("\nLanding...")

                elif key in moveBindings:
                    # Publica el vector Twist para moverse con los valores obtenidos de moveBindings
                    x, y, z, th = moveBindings[key]
                    self.movements.twist.linear.x = x
                    self.movements.twist.linear.y = y
                    self.movements.twist.linear.z = z
                    self.movements.twist.angular.z = th
                    self.pub.publish(self.movements.twist)

                elif key in cameraBindings:
                    # Control manual de la cámara
                    pan, tilt = cameraBindings[key]
                    if pan != 0:
                        self.movements.camera_pan(pan)
                    if tilt != 0:
                        self.movements.camera_tilt(tilt)
                    print(f'\nCamera control: pan {pan}°, tilt {tilt}°')

                else:
                    # Si se presiona una tecla desconocida → detiene el dron en ese punto, mas no lo aterriza
                    self.movements.reset_twist()
                    self.pub.publish(self.movements.twist)

            else:
                pass  # Si está en automático, no se hace nada con teclas

# ===========================================================
# EJECUCIÓN PRINCIPAL
# ===========================================================
if __name__ == "__main__":
    teleop = BebopTeleop()
    try:
        teleop.run()
    except rospy.ROSInterruptException:
        pass
