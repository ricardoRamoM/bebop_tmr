#!/usr/bin/env python3
# window_tracking_mission.py

import rospy
from std_msgs.msg import String

cameraBindings = {
    'i': (0, 5),
    'k': (0, -5),
    'j': (-5, 0),
    'l': (5, 0),
}

class WindowTrackingMission:

    def __init__(self):

        self.active = False
        self.mode_flag = "automatic"  # fijo para esta misión

        rospy.Subscriber('/bebop/start_mission', String, self.start_callback)
        rospy.Subscriber('/bebop/command_throttled', String, self.command_callback)

        self.status_pub = rospy.Publisher('/bebop/mission_status', String, queue_size=10)

        rospy.loginfo("WindowTrackingMission ready")

    # --------------------------------------------------
    # Activación
    # --------------------------------------------------

    def start_callback(self, msg):
        if msg.data == "WINDOW_TRACK":
            self.active = True
            rospy.loginfo("Window Tracking Mission STARTED")

    # --------------------------------------------------
    # Comandos
    # --------------------------------------------------

    def command_callback(self, msg):

        if not self.active:
            return

        command = msg.data
        rospy.loginfo(f"[MISSION] Command: {command}")

        command_to_method_mapping = {
            'w': 'forward',
            'a': 'left',
            'd': 'right',
            's': 'backwards',
            '+': 'up',
            '-': 'down',
            'q': 'turn_left',
            'e': 'turn_right',
        }

        if command == '2':
            self.movements.landing(self.mode_flag)
            rospy.loginfo("Landing...")

        elif command in command_to_method_mapping:
            method_name = command_to_method_mapping[command]
            movement_method = getattr(self.movements, method_name)
            movement_method(self.mode_flag)

        elif command in cameraBindings:
            pan, tilt = cameraBindings[command]
            if pan != 0:
                self.movements.camera_pan(pan)
            if tilt != 0:
                self.movements.camera_tilt(tilt)

        else:
            rospy.loginfo(f"Unknown command: {command}")

        # --------------------------------------------------
        # Condición interna de fin
        # --------------------------------------------------

        if self.check_completion():
            self.status_pub.publish("completed")
            self.active = False
