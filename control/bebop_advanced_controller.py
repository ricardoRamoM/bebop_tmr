#!/usr/bin/env python3
# bebop_advanced_controller.py

# Clase para controlar movimientos básicos del dron Parrot Bebop usando ROS 

# Más que nada es para autonomo de las misiones

# Las velocidades son tienen estos rangos [-1,1]
# Los valores >1 o <-1 se saturan automáticamente y evitan por ende esos valores máximos en lugar del que sobrepasa.
# Esto es % de la velocidad maxima del dron

# linear.x   # Avance y retroceso (x positivo adelante)
# linear.y   # Movimiento lateral (y positivo izquierda)
# linear.z   # Subir o bajar (z positivo arriba)
# angular.x  # Rotación sobre eje X (rara vez usada) 
# angular.y  # Rotación sobre eje Y (rara vez usada)
# angular.z  # Giro (yaw) (positivo giro izquierda)

import rospy
import math
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from std_msgs.msg import Empty
from tf.transformations import euler_from_quaternion

class BebopAdvancedController:

    def __init__(self, pub_cmd_vel, pub_takeoff, pub_land, pub_camera):

        self.pub_cmd_vel = pub_cmd_vel
        self.pub_takeoff = pub_takeoff
        self.pub_land = pub_land
        self.pub_camera = pub_camera

        rospy.Subscriber("/bebop/odom", Odometry, self.odom_callback)

        self.current_position = None
        self.current_yaw = 0.0
        self.initial_position = None

        self.target_position = None
        self.navigation_active = False
        self.hover_active = False

        self.emergency_stop = False

        self.rate = rospy.Rate(30)

    # =====================================================
    # ODOM
    # =====================================================

    def odom_callback(self, msg):

        self.current_position = msg.pose.pose.position

        orientation_q = msg.pose.pose.orientation
        orientation_list = [
            orientation_q.x,
            orientation_q.y,
            orientation_q.z,
            orientation_q.w
        ]

        (_, _, yaw) = euler_from_quaternion(orientation_list)
        self.current_yaw = yaw

        if self.initial_position is None:
            self.initial_position = self.current_position
            rospy.loginfo("✔ Posición inicial fijada")

    def wait_for_odometry(self):
        while self.current_position is None and not rospy.is_shutdown():
            rospy.sleep(0.1)

    # =====================================================
    # EMERGENCIA
    # =====================================================

    def activate_emergency(self):
        rospy.logwarn("🚨 EMERGENCY STOP ACTIVATED")
        self.emergency_stop = True
        self.navigation_active = False
        self.hover_active = False
        self.stop()

    def clear_emergency(self):
        self.emergency_stop = False

    # =====================================================
    # BÁSICOS
    # =====================================================

    def takeoff(self):
        if not self.emergency_stop:
            self.pub_takeoff.publish(Empty())

    def land(self):
        self.pub_land.publish(Empty())

    def stop(self):
        self.pub_cmd_vel.publish(Twist())

    # =====================================================
    # POSICIÓN RELATIVA
    # =====================================================

    def get_relative_position(self):

        rx = self.current_position.x - self.initial_position.x
        ry = self.current_position.y - self.initial_position.y
        rz = self.current_position.z - self.initial_position.z

        return rx, ry, rz

    # =====================================================
    # MOVIMIENTO MANUAL (NO BLOQUEANTE)
    # =====================================================

    def send_manual_velocity(self, vx, vy, vz, wz=0.0):

        if self.emergency_stop:
            return

        twist = Twist()
        twist.linear.x = vx
        twist.linear.y = vy
        twist.linear.z = vz
        twist.angular.z = wz

        self.pub_cmd_vel.publish(twist)

    # =====================================================
    # NAVEGACIÓN POR TARGET
    # =====================================================

    def set_target(self, x, y, z):
        self.target_position = (x, y, z)
        self.navigation_active = True
        self.hover_active = False

    def activate_hover(self):
        rx, ry, rz = self.get_relative_position()
        self.target_position = (rx, ry, rz)
        self.hover_active = True
        self.navigation_active = False

    # =====================================================
    # UPDATE (SE LLAMA EN CADA CICLO)
    # =====================================================

    def update(self):

        if self.emergency_stop:
            self.stop()
            return

        if self.navigation_active and self.target_position is not None:

            rx, ry, rz = self.get_relative_position()

            error_x = self.target_position[0] - rx
            error_y = self.target_position[1] - ry
            error_z = self.target_position[2] - rz

            distance = math.sqrt(error_x**2 + error_y**2 + error_z**2)

            if distance < 0.05:
                rospy.loginfo("✔ Target alcanzado")
                self.navigation_active = False
                self.stop()
                return

            kp = 0.8

            vx = max(min(kp * error_x, 0.4), -0.4)
            vy = max(min(kp * error_y, 0.4), -0.4)
            vz = max(min(kp * error_z, 0.3), -0.3)

            self.send_manual_velocity(vx, vy, vz)

        elif self.hover_active:

            rx, ry, rz = self.get_relative_position()

            error_x = self.target_position[0] - rx
            error_y = self.target_position[1] - ry
            error_z = self.target_position[2] - rz

            kp = 0.8

            vx = max(min(kp * error_x, 0.3), -0.3)
            vy = max(min(kp * error_y, 0.3), -0.3)
            vz = max(min(kp * error_z, 0.3), -0.3)

            self.send_manual_velocity(vx, vy, vz)

    # =====================================================
    # CÁMARA
    # =====================================================

    def camera_tilt(self, angle_deg):
        cam = Twist()
        cam.angular.y = math.radians(angle_deg)
        self.pub_camera.publish(cam)

    def camera_pan(self, angle_deg):
        cam = Twist()
        cam.angular.z = math.radians(angle_deg)
        self.pub_camera.publish(cam)