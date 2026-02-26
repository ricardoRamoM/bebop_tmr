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
---------------------------
Manual Control:
   q    w   e       +: up
   a        d       -: down
        s   
---------------------------
space → Stop (reset twist)

Cam control:
---------------------------
   j    i    l       
        k    
---------------------------

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
        self.missions_movements = BebopAdvancedController(
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
                self.missions_movements.takeoff()

            elif key == '2':
                rospy.loginfo("LAND")
                self.missions_movements.land()

            elif key == 'x':
                rospy.logwarn("EMERGENCY")
                self.missions_movements.activate_emergency()

            elif key == 'z':
                rospy.logwarn("CLEAR EMERGENCY")
                self.missions_movements.clear_emergency()

            # =========================
            # MOVEMENT (manual velocity)
            # =========================

            elif key == 'w':
                self.missions_movements.send_velocity(0.3, 0, 0)

            elif key == 's':
                self.missions_movements.send_velocity(-0.3, 0, 0)

            elif key == 'a':
                self.missions_movements.send_velocity(0, 0.3, 0)

            elif key == 'd':
                self.missions_movements.send_velocity(0, -0.3, 0)

            elif key == 'q':
                self.missions_movements.send_velocity(0, 0, 0, 0.5)

            elif key == 'e':
                self.missions_movements.send_velocity(0, 0, 0, -0.5)

            elif key == '+':
                self.missions_movements.send_velocity(0, 0, 0.3)

            elif key == '-':
                self.missions_movements.send_velocity(0, 0, -0.3)

            elif key == ' ': # tecla space 
                self.missions_movements.stop()

            # =========================
            # CAMERA
            # =========================

            elif key == 'i':
                self.missions_movements.camera_tilt(5)

            elif key == 'k':
                self.missions_movements.camera_tilt(-5)

            elif key == 'j':
                self.missions_movements.camera_pan(-5)

            elif key == 'l':
                self.missions_movements.camera_pan(5)
            
            # =========================
            # NAVEGACIÓN POR TARGET
            # =========================

            # NAVEGACIÓN POR TARGET
            elif key == '8':
                rospy.loginfo("Forward 1m")
                rx, ry, rz = self.missions_movements.get_relative_position()
                rospy.loginfo(f"Current RX: {rx}")
                rospy.loginfo(f"Setting target X: {rx + 1.0}")
                self.missions_movements.set_target_position(rx + 1.0, ry, rz)

            elif key == '9':
                rospy.loginfo("Left 1m")
                rx, ry, rz = self.missions_movements.get_relative_position()
                rospy.loginfo(f"Current RY: {ry}")
                rospy.loginfo(f"Setting target Y: {ry + 1.0}")
                self.missions_movements.set_target_position(rx, ry + 1.0, rz)

            elif key == '0':
                #rospy.loginfo(self.missions_controller.get_relative_position())
                rospy.loginfo("Up 1m")
                rx, ry, rz = self.missions_movements.get_relative_position()
                rospy.loginfo(f"Current RZ: {rz}")
                rospy.loginfo(f"Setting target Z: {rz + 1.0}")
                self.missions_movements.set_target_position(rx, ry, rz + 1.0)

            elif key == 'p':
                rospy.loginfo("Turn 90°")
                self.missions_movements.set_target_yaw(90)


            # =========================
            # NAVEGACIÓN POR TARGET
            #elif key == '8':
            #    rospy.loginfo("SET TARGET +1m X")
            #    self.missions_controller.wait_for_odometry()
            #    rx, ry, rz = self.missions_controller.get_relative_position()
            #    self.missions_controller.set_target(1.0, ry, rz)
            #elif key == '9':
            #    rospy.loginfo("SET TARGET +1m Y")
            #    self.missions_controller.wait_for_odometry()
            #    rx, ry, rz = self.missions_controller.get_relative_position()
            #    self.missions_controller.set_target(rx, 1.0, rz)
            #elif key == '0':
            #    rospy.loginfo("SET TARGET +1m Z")
            #    self.missions_controller.wait_for_odometry()
            #    rx, ry, rz = self.missions_controller.get_relative_position()
            #    self.missions_controller.set_target(rx, ry, 1.0)
            # =========================
            # HOVER
            #elif key == 'h':
            #    rospy.loginfo("ACTIVATE HOVER")
            #    self.missions_controller.activate_hover()   
            #elif key == 'g':
            #    rospy.loginfo("DEACTIVATE HOVER")
            #    self.missions_controller.deactivate_hover() 
            # =========================
            # MOVEMENTS
            #elif key == 'm':
            #    rospy.loginfo("Forward")
            #    self.missions_controller.move_forward(0.75)   
            #elif key == 'n':
            #    rospy.loginfo("Backward")
            #    self.missions_controller.move_backward(0.75)
            #elif key == 'b':
            #    rospy.loginfo("Right")
            #    self.missions_controller.move_right(0.75)    
            #elif key == 'v':
            #    rospy.loginfo("Left")
            #    self.missions_controller.move_left(0.75)
            #elif key == 'p':
            #    rospy.loginfo("turn left")
            #    self.missions_controller.turn(90)
            #elif key == 'c':
            #    rospy.loginfo("mant alt")
            #    vz = self.missions_controller.maintain_altitude(1.5)
            #    self.missions_controller.send_manual_velocity(0, 0, vz)
            #elif key == 'f':
            #    rospy.loginfo("go_to_height")
            #    self.missions_controller.go_to_height(1.75)

            # =========================
            # EXIT
            # =========================

            elif key == '\x03':
                rospy.loginfo("Landing before exit...")
                self.missions_movements.land()
                rospy.signal_shutdown("Ctrl+C pressed")
                break

            # IMPORTANTE:
            # si estás usando navegación por target,
            # debes llamar update() en cada ciclo
            self.missions_movements.update()

            rate.sleep()

# ===========================================================
# EJECUCIÓN PRINCIPAL
# ===========================================================

if __name__ == "__main__":
    teleop = BebopTeleopTest()
    teleop.run()

