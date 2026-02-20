#!/usr/bin/env python3
# bebop_autonomous_controller.py

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


class BebopMissionsController:

    def __init__(self):

        # Publishers
        self.vel_pub = rospy.Publisher("/bebop/cmd_vel", Twist, queue_size=10)
        self.takeoff_pub = rospy.Publisher("/bebop/takeoff", Empty, queue_size=1)
        self.land_pub = rospy.Publisher("/bebop/land", Empty, queue_size=1)

        # Subscriber
        rospy.Subscriber("/bebop/odom", Odometry, self.odom_callback)

        # Variables de estado
        self.current_position = None
        self.current_yaw = 0.0

        self.rate = rospy.Rate(20)
        rospy.sleep(1)

    # =============================
    # ODOM CALLBACK
    # =============================
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


    # =============================
    # FUNCIONES BÁSICAS
    # =============================

    def takeoff(self):
        rospy.loginfo("Taking off...")
        self.takeoff_pub.publish(Empty())
        rospy.sleep(3)


    def land(self):
        rospy.loginfo("Landing...")
        self.land_pub.publish(Empty())
        rospy.sleep(3)

    def stop(self):
        self.vel_pub.publish(Twist())
        rospy.sleep(0.2)

    def get_current_pose(self):
        return self.current_position, self.current_yaw

    # =============================
    # MOVIMIENTO INTERNO GENÉRICO
    # =============================
    def _move_axis(self, axis, distance, kp=0.8, tolerance=0.02, max_speed=0.4):

        if self.current_position is None:
            rospy.logwarn("No odometry received yet!")
            return

        twist = Twist()

        if axis == "x":
            start = self.current_position.x
        elif axis == "y":
            start = self.current_position.y
        elif axis == "z":
            start = self.current_position.z
        else:
            return

        target = start + distance

        while not rospy.is_shutdown():

            if axis == "x":
                current = self.current_position.x
            elif axis == "y":
                current = self.current_position.y
            else:
                current = self.current_position.z

            error = target - current

            if abs(error) < tolerance:
                break

            velocity = kp * error
            velocity = max(min(velocity, max_speed), -max_speed)

            if axis == "x":
                twist.linear.x = velocity
            elif axis == "y":
                twist.linear.y = velocity
            else:
                twist.linear.z = velocity

            self.vel_pub.publish(twist)
            self.rate.sleep()

        self.stop()
    

    # =============================
    # MOVIMIENTOS TIPO HUMANO
    # =============================

    def adelante(self, distancia):
        self._move_axis("x", abs(distancia))

    def atras(self, distancia):
        self._move_axis("x", -abs(distancia))

    def izquierda(self, distancia):
        self._move_axis("y", abs(distancia))

    def derecha(self, distancia):
        self._move_axis("y", -abs(distancia))

    def arriba(self, distancia):
        self._move_axis("z", abs(distancia), max_speed=0.3)

    def abajo(self, distancia):
        self._move_axis("z", -abs(distancia), max_speed=0.3)

    # =============================
    # ROTACIÓN
    # =============================
    def rotar(self, angle_rad, kp=1.2, tolerance=0.02):

        twist = Twist()
        start_yaw = self.current_yaw
        target_yaw = start_yaw + angle_rad

        while not rospy.is_shutdown():

            error = target_yaw - self.current_yaw
            error = math.atan2(math.sin(error), math.cos(error))

            if abs(error) < tolerance:
                break

            vel = kp * error
            vel = max(min(vel, 0.6), -0.6)

            twist.angular.z = vel
            self.vel_pub.publish(twist)
            self.rate.sleep()

        self.stop()


    # =============================
    # HOLD POSITION
    # =============================

    def hold_position(self, duration):

        twist = Twist()
        start_time = rospy.Time.now()

        while not rospy.is_shutdown():

            self.vel_pub.publish(twist)

            if (rospy.Time.now() - start_time).to_sec() > duration:
                break

            self.rate.sleep()
            

    def wait_for_odometry(self):
        rospy.loginfo("Waiting for odometry...")

        while self.current_position is None and not rospy.is_shutdown():
            rospy.sleep(0.1)

        rospy.loginfo("Odometry ready ✔")


# ======================================
# EJEMPLO DE USO
# ======================================

#if __name__ == "__main__":
#    drone = BebopBasicController()
#    rospy.sleep(2)
#    drone.takeoff()
#    drone.move_distance_x(1.0)
#    drone.move_distance_y(1.0)
#    drone.rotate_angle(math.pi/2)
#    drone.move_distance_x(1.0)

#    drone.hold_position(2)

#    drone.land()
