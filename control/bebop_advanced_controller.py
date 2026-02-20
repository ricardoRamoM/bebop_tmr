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

    def __init__(self):

        # Publishers
        self.vel_pub = rospy.Publisher("/bebop/cmd_vel", Twist, queue_size=10)
        self.takeoff_pub = rospy.Publisher("/bebop/takeoff", Empty, queue_size=1)
        self.land_pub = rospy.Publisher("/bebop/land", Empty, queue_size=1)

        # Subscriber
        rospy.Subscriber("/bebop/odom", Odometry, self.odom_callback)

        # Estado actual
        self.current_position = None
        self.current_yaw = 0.0

        # Posición inicial (referencia)
        self.initial_position = None

        # Hover target
        self.hover_target = None

        self.rate = rospy.Rate(30)
        rospy.sleep(1)

    # =====================================
    # ODOM CALLBACK
    # =====================================
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

        # Guardar posición inicial automáticamente
        if self.initial_position is None:
            self.initial_position = self.current_position
            rospy.loginfo("Posición inicial establecida ✔")

    # =====================================
    # FUNCIONES BÁSICAS
    # =====================================

    def takeoff(self):
        self.takeoff_pub.publish(Empty())
        rospy.sleep(3)

    def land(self):
        self.land_pub.publish(Empty())
        rospy.sleep(3)

    def stop(self):
        self.vel_pub.publish(Twist())

    def wait_for_odometry(self):
        while self.current_position is None and not rospy.is_shutdown():
            rospy.sleep(0.1)

    # =====================================
    # COORDENADAS RELATIVAS
    # =====================================

    def get_relative_position(self):

        rx = self.current_position.x - self.initial_position.x
        ry = self.current_position.y - self.initial_position.y
        rz = self.current_position.z - self.initial_position.z

        return rx, ry, rz

    # =====================================
    # IR A UNA COORDENADA (DIAGONAL REAL)
    # =====================================

    def go_to(self, target_x, target_y, target_z,
              kp=0.8, tolerance=0.05, max_speed=0.4):

        rospy.loginfo(f"Going to ({target_x}, {target_y}, {target_z})")

        twist = Twist()

        while not rospy.is_shutdown():

            rx, ry, rz = self.get_relative_position()

            error_x = target_x - rx
            error_y = target_y - ry
            error_z = target_z - rz

            distance = math.sqrt(error_x**2 + error_y**2 + error_z**2)

            if distance < tolerance:
                break

            vx = kp * error_x
            vy = kp * error_y
            vz = kp * error_z

            # Saturación
            vx = max(min(vx, max_speed), -max_speed)
            vy = max(min(vy, max_speed), -max_speed)
            vz = max(min(vz, 0.3), -0.3)

            twist.linear.x = vx
            twist.linear.y = vy
            twist.linear.z = vz

            self.vel_pub.publish(twist)
            self.rate.sleep()

        self.stop()
        rospy.loginfo("Objetivo alcanzado ✔")

    # =====================================
    # HOVER DINÁMICO (PID continuo)
    # =====================================

    def hover_activate(self, kp=0.8):

        rospy.loginfo("Hover activado")

        rx, ry, rz = self.get_relative_position()
        self.hover_target = (rx, ry, rz)

        twist = Twist()

        while not rospy.is_shutdown():

            rx, ry, rz = self.get_relative_position()

            error_x = self.hover_target[0] - rx
            error_y = self.hover_target[1] - ry
            error_z = self.hover_target[2] - rz

            vx = kp * error_x
            vy = kp * error_y
            vz = kp * error_z

            vx = max(min(vx, 0.3), -0.3)
            vy = max(min(vy, 0.3), -0.3)
            vz = max(min(vz, 0.3), -0.3)

            twist.linear.x = vx
            twist.linear.y = vy
            twist.linear.z = vz

            self.vel_pub.publish(twist)
            self.rate.sleep()