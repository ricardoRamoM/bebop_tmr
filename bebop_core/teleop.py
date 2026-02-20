#!/usr/bin/env python3
# teleop_node.py

# Control manual + disparador de misiones (Supervisor) de un dron Parrot Bebop mediante teclado y ROS.

import os
import sys
import rospy
import select
import termios
import tty
from std_msgs.msg import Empty
from geometry_msgs.msg import Twist

# Importar controlador
current_dir = os.path.dirname(os.path.abspath(__file__))
project_root = os.path.abspath(os.path.join(current_dir, '..'))
sys.path.append(project_root)

from control.bebop_advanced_controller import BebopAdvancedController
from bebop_core.mission_supervisor import MissionSupervisor

msg = """
=========================================
ADVANCED CONTROLLER TEST MODE
=========================================

TAKEOFF / LAND
1 → Takeoff
2 → Land
x → Emergency

MOVEMENTS
w → Forward
s → Backward
a → Left
d → Right
q → Rotate Left
e → Rotate Right
+ → Ascend
- → Descend
space → Stop (reset twist)

CAMERA
i → Tilt Up
k → Tilt Down
j → Pan Left
l → Pan Right

CTRL+C → Exit safely
=========================================
"""

# ===========================================================
# CLASE TELEOP
# ===========================================================

class BebopTeleopTest:
    def __init__(self):

        
        # Inicializa el nodo ROS
        rospy.init_node('teleop_test_controller')

        # Publicadores para los distintos comandos del dron
        self.pub_cmd_vel = rospy.Publisher('bebop/cmd_vel', Twist, queue_size=1)          # Movimiento
        self.pub_takeoff = rospy.Publisher('bebop/takeoff', Empty, queue_size=10) # Despegue
        self.pub_land = rospy.Publisher('bebop/land', Empty, queue_size=10)       # Aterrizaje
        self.pub_camera = rospy.Publisher('bebop/camera_control', Twist, queue_size=1) # Cámara

        # Movements (bajo nivel)
        # Crea objeto para manejar movimientos de aterrizaje, despegue y los movimientos automaticos 
        self.missions_controller = BebopAdvancedController(
            self.pub_cmd_vel,
            self.pub_takeoff,
            self.pub_land,
            self.pub_camera
        )
        
        # Guarda configuración del teclado (para restaurarla luego)
        self.settings = termios.tcgetattr(sys.stdin)

        print(msg)

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
        rate = rospy.Rate(30)

        while not rospy.is_shutdown():

            key = self.getKey()

            # =========================
            # TAKEOFF / LAND
            # =========================

            if key == '1':
                rospy.loginfo("TAKEOFF")
                self.missions_controller.takeoff()

            elif key == '2':
                rospy.loginfo("LAND")
                self.missions_controller.land()

            elif key == 'x':
                rospy.logwarn("EMERGENCY")
                self.missions_controller.activate_emergency()

            # =========================
            # MOVEMENT (manual velocity)
            # =========================

            elif key == 'w':
                self.missions_controller.send_manual_velocity(0.3, 0, 0)

            elif key == 's':
                self.missions_controller.send_manual_velocity(-0.3, 0, 0)

            elif key == 'a':
                self.missions_controller.send_manual_velocity(0, 0.3, 0)

            elif key == 'd':
                self.missions_controller.send_manual_velocity(0, -0.3, 0)

            elif key == 'q':
                self.missions_controller.send_manual_velocity(0, 0, 0, 0.5)

            elif key == 'e':
                self.missions_controller.send_manual_velocity(0, 0, 0, -0.5)

            elif key == '+':
                self.missions_controller.send_manual_velocity(0, 0, 0.3)

            elif key == '-':
                self.missions_controller.send_manual_velocity(0, 0, -0.3)

            elif key == ' ':
                self.missions_controller.stop()

            # =========================
            # CAMERA
            # =========================

            elif key == 'i':
                self.missions_controller.camera_tilt(5)

            elif key == 'k':
                self.missions_controller.camera_tilt(-5)

            elif key == 'j':
                self.missions_controller.camera_pan(-5)

            elif key == 'l':
                self.missions_controller.camera_pan(5)

            # =========================
            # EXIT
            # =========================

            elif key == '\x03':
                rospy.loginfo("Landing before exit...")
                self.missions_controller.land()
                rospy.signal_shutdown("Ctrl+C pressed")
                break

            # IMPORTANTE:
            # si estás usando navegación por target,
            # debes llamar update() en cada ciclo
            self.missions_controller.update()

            rate.sleep()

# ===========================================================
# EJECUCIÓN PRINCIPAL
# ===========================================================

if __name__ == "__main__":
    teleop = BebopTeleopTest()
    teleop.run()

