#!/usr/bin/env python3

import rospy
import math
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from std_msgs.msg import Empty


class TMRController:

    def __init__(self):

        rospy.init_node("tmr_body_controller")

        # Topics Bebop
        self.cmd_pub = rospy.Publisher("/bebop/cmd_vel", Twist, queue_size=1)
        self.takeoff_pub = rospy.Publisher("/bebop/takeoff", Empty, queue_size=1)
        self.land_pub = rospy.Publisher("/bebop/land", Empty, queue_size=1)

        rospy.Subscriber("/bebop/odom", Odometry, self.odom_callback)

        # Pose actual
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0

        # Parámetros ajustables
        self.kp_yaw = 1.2 # 2.5  primero
        self.v_max = 0.3 # 0.35
        self.w_max = 0.8  # 1.2
        self.tolerance = 0.03

        rospy.sleep(2)  # esperar odometría

    # -------------------------
    # ODOMETRÍA
    # -------------------------

    def odom_callback(self, msg):

        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y

        q = msg.pose.pose.orientation
        self.theta = self.quaternion_to_yaw(q)

        rospy.loginfo(f"x: {self.x:.3f}  y: {self.y:.3f}")

    def quaternion_to_yaw(self, q):
        siny = 2.0 * (q.w * q.z + q.x * q.y)
        cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        return math.atan2(siny, cosy)

    def normalize_angle(self, angle):
        while angle > math.pi:
            angle -= 2*math.pi
        while angle < -math.pi:
            angle += 2*math.pi
        return angle

    def saturate(self, value, limit):
        return max(min(value, limit), -limit)

    # -------------------------
    # CONTROLADOR
    # -------------------------

    def go_to(self, x_target, y_target):

        rate = rospy.Rate(40)

        while not rospy.is_shutdown():

            dx = x_target - self.x
            dy = y_target - self.y
            rho = math.sqrt(dx**2 + dy**2)

            if rho < self.tolerance:
                break

            # Transformación global → cuerpo
            x_body =  math.cos(self.theta)*dx + math.sin(self.theta)*dy
            y_body = -math.sin(self.theta)*dx + math.cos(self.theta)*dy

            desired_theta = math.atan2(dy, dx)
            alpha = self.normalize_angle(desired_theta - self.theta)

            # Desaceleración suave
            v = self.v_max * math.tanh(1.2 * x_body) # tanh factor inicial 2.0
            v_lateral = 0.0
            w = self.kp_yaw * alpha

            if abs(alpha) > 0.4:
                v *= 0.3
                v_lateral *= 0.3

            v = self.saturate(v, self.v_max)
            v_lateral = self.saturate(v_lateral, 0.3)
            w = self.saturate(w, self.w_max)

            cmd = Twist()
            cmd.linear.x = v
            cmd.linear.y = v_lateral
            cmd.angular.z = w

            self.cmd_pub.publish(cmd)
            rate.sleep()

        self.cmd_pub.publish(Twist())

    # -------------------------
    # TEST SIMPLE
    # -------------------------

    def run_test(self):

        rospy.loginfo("🚀 Despegando...")
        self.takeoff_pub.publish(Empty())
        rospy.sleep(5)

        x0 = self.x
        y0 = self.y

        rospy.loginfo("➡ Moviendo 0.3m adelante")
        self.go_to(x0 + 0.3, y0)

        rospy.sleep(3)

        #rospy.loginfo("⬅ Regresando al origen")
        #self.go_to(0.0, 0.0)
        #rospy.sleep(2)

        rospy.loginfo(f"Posición final: x={self.x:.3f}, y={self.y:.3f}")

        rospy.loginfo("🛬 Aterrizando...")
        self.land_pub.publish(Empty())


# -------------------------
# MAIN
# -------------------------

if __name__ == "__main__":
    controller = TMRController()
    try:
        controller.run_test()
        #rospy.loginfo("Hover test iniciado...")
        #rospy.spin()
    except rospy.ROSInterruptException:
        pass

#if __name__ == "__main__":
#    controller = TMRController()
#    rate = rospy.Rate(40)
#    rospy.loginfo("Publicando ceros...")
#    while not rospy.is_shutdown():
#        cmd = Twist()
#        controller.cmd_pub.publish(cmd)
#        rate.sleep()