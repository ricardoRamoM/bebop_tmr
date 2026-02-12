#!/usr/bin/env python3
import rospy
from std_msgs.msg import String

def main():
    rospy.init_node("mission_square")

    status_pub = rospy.Publisher('/bebop/mission_status', String, queue_size=1)

    # Lógica de misión aquí
    rospy.sleep(8.0)

    # Cuando termina condición interna:
    status_pub.publish("completed")

if __name__ == "__main__":
    main()
