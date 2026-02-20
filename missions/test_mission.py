#!/usr/bin/env python3
# test_mission.py


import rospy
from control.bebop_advanced_controller import BebopAdvancedController

if __name__ == "__main__":

    rospy.init_node("test_mission")

    drone = BebopAdvancedController()
    drone.wait_for_odometry()

    drone.takeoff()

    rospy.sleep(2)

    # Movimiento diagonal
    drone.go_to(1.0, 1.0, 1.0)

    #rospy.sleep(2)

    # Ir a otro punto específico
    #drone.go_to(0.5, -0.5, 1.0)

    #rospy.sleep(2)

    # Activar hover (mantener posición actual)
    drone.hover_activate()

    drone.land()