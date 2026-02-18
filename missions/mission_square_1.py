#!/usr/bin/env python3
#mission_square_1.py

import rospy
import math
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from std_msgs.msg import String

pose = {"x": 0.0, "y": 0.0, "theta": 0.0}

# ==============================
# ODOM CALLBACK
# ==============================

def odom_callback(msg):
    global pose
    pose["x"] = msg.pose.pose.position.x
    pose["y"] = msg.pose.pose.position.y

    import tf
    orientation_q = msg.pose.pose.orientation
    (roll, pitch, yaw) = tf.transformations.euler_from_quaternion([
        orientation_q.x, orientation_q.y,
        orientation_q.z, orientation_q.w
    ])
    pose["theta"] = yaw


# ==============================
# MOVIMIENTOS (SIN ROTACIÓN)
# ==============================

def stop_drone(pub):
    pub.publish(Twist())
    rospy.sleep(0.3)


def move(pub, direction, distance, speed=0.2):

    x0 = pose["x"]
    y0 = pose["y"]

    twist = Twist()

    if direction == "adelante":
        twist.linear.x = speed

    elif direction == "atras":
        twist.linear.x = -speed

    elif direction == "derecha":
        twist.linear.y = -speed   # OJO: derecha suele ser negativa en Bebop

    elif direction == "izquierda":
        twist.linear.y = speed

    rate = rospy.Rate(30)

    while not rospy.is_shutdown():

        dx = pose["x"] - x0
        dy = pose["y"] - y0
        d = math.sqrt(dx**2 + dy**2)

        if d >= distance:
            break

        pub.publish(twist)
        rate.sleep()

    stop_drone(pub)


# ==============================
# MAIN MISSION (CUADRADO SIN GIRAR)
# ==============================

if __name__ == "__main__":

    rospy.init_node("mission_square_1")

    rospy.Subscriber("/bebop/odom", Odometry, odom_callback)
    vel_pub = rospy.Publisher("/bebop/cmd_vel", Twist, queue_size=1)
    status_pub = rospy.Publisher("/mission/status", String, queue_size=1)

    rospy.sleep(2)

    rospy.loginfo("MISSION SQUARE SIN GIRO INICIADA")

    try:

        side = 0.40

        # Lado 1
        move(vel_pub, "adelante", side)

        # Lado 2
        move(vel_pub, "derecha", side)

        # Lado 3
        move(vel_pub, "atras", side)

        # Lado 4
        move(vel_pub, "izquierda", side)

        rospy.loginfo("Cuadrado completado sin girar")
        status_pub.publish("done")

    except Exception as e:

        rospy.logerr(f"Mission failed: {e}")
        status_pub.publish("failed")
