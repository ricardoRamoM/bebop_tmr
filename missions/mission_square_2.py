#!/usr/bin/env python3
#mission_square_2.py

import rospy
import os
import sys
from std_msgs.msg import String

# =====================================================
# FIX IMPORT PATH
# =====================================================
current_dir = os.path.dirname(os.path.abspath(__file__))
project_root = os.path.abspath(os.path.join(current_dir, '..'))
sys.path.append(project_root)
from control.bebop_autonomous_controller import BebopMissionsController

if __name__ == "__main__":

    rospy.init_node("mission_square_2")

    status_pub = rospy.Publisher("/mission/status", String, queue_size=1)

    drone = BebopMissionsController()

    rospy.sleep(2)

    rospy.loginfo("MISSION SQUARE 2 INICIADA")

    try:

        lado = 0.40

        drone.takeoff()
        rospy.sleep(2)

        # Cuadrado sin girar
        drone.adelante(lado)
        drone.derecha(lado)
        drone.atras(lado)
        drone.izquierda(lado)

        drone.hold_position(2)

        drone.land()

        rospy.loginfo("Cuadrado completado")
        status_pub.publish("done")

    except Exception as e:

        rospy.logerr(f"Mission failed: {e}")
        status_pub.publish("failed")
        drone.land()

