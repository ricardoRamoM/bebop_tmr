#!/usr/bin/env python3
# mission_orange_window.py

import rospy
from sensor_msgs.msg import Image
from std_msgs.msg import String
from cv_bridge import CvBridge
import cv2

from control.bebop_movements import BebopMovements
from perception.window_orange_detector import BebopCameraProcessor


class MissionOrangeWindow:

    def __init__(self):
        rospy.init_node('mission_orange_window')

        # Movement interface
        self.movements = BebopMovements()

        # Perception module
        self.detector = BebopCameraProcessor()

        # ROS utilities
        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber(
            "/bebop/image_raw",
            Image,
            self.image_callback
        )

        self.status_pub = rospy.Publisher(
            "/mission/status",
            String,
            queue_size=1
        )

        # Mission state
        self.finished = False
        self.center_tolerance = 25
        self.forward_counter = 0

        rospy.loginfo("Mission Orange Window initialized")

    # =====================================================
    # IMAGE CALLBACK
    # =====================================================

    def image_callback(self, msg):

        if self.finished:
            return

        try:
            frame = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except Exception as e:
            rospy.logerr(f"CvBridge error: {e}")
            return

        processed_image, data = self.detector.process_image(frame)

        # Optional debug window
        cv2.imshow("Orange Detection", processed_image)
        cv2.waitKey(1)

        self.control_logic(data)

    # =====================================================
    # CONTROL LOGIC
    # =====================================================

    def control_logic(self, data):

        if not data["detected"]:
            rospy.loginfo("Searching window...")
            self.movements.turn_left("automatic")
            self.forward_counter = 0
            return

        cx = data["cx"]
        center_x = data["center_x"]

        # Align horizontally
        if cx < center_x - self.center_tolerance:
            rospy.loginfo("Adjusting left")
            self.movements.left("automatic")
            self.forward_counter = 0

        elif cx > center_x + self.center_tolerance:
            rospy.loginfo("Adjusting right")
            self.movements.right("automatic")
            self.forward_counter = 0

        else:
            rospy.loginfo("Aligned - moving forward")
            self.movements.forward("automatic")
            self.forward_counter += 1

        # Condition to consider window passed
        if self.forward_counter > 20:
            rospy.loginfo("Window passed!")
            self.finish_mission()

    # =====================================================
    # FINISH MISSION
    # =====================================================

    def finish_mission(self):
        self.movements.stop("automatic")
        self.status_pub.publish("done")
        self.finished = True
        rospy.loginfo("Mission Orange Window completed")

    # =====================================================
    # SPIN
    # =====================================================

    def run(self):
        rospy.spin()


if __name__ == "__main__":
    mission = MissionOrangeWindow()
    mission.run()
