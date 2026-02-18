#!/usr/bin/env python3
# mission_orange_window.py

import rospy
from sensor_msgs.msg import Image
from std_msgs.msg import String, Empty
from geometry_msgs.msg import Twist
from cv_bridge import CvBridge
import cv2
import os
import sys

# =====================================================
# FIX IMPORT PATH
# =====================================================

current_dir = os.path.dirname(os.path.abspath(__file__))
project_root = os.path.abspath(os.path.join(current_dir, '..'))
sys.path.append(project_root)

from control.bebop_teleop_controller import BebopMovements
from perception.window_orange_detector import BebopCameraProcessor

# =====================================================
# MISSION CLASS
# ====================================================
class MissionOrangeWindow:

    def __init__(self):

        rospy.init_node('mission_orange_window')
        
        # ========================
        # Publishers
        # ========================
        self.pub_cmd = rospy.Publisher('/bebop/cmd_vel', Twist, queue_size=1)
        self.pub_takeoff = rospy.Publisher('/bebop/takeoff', Empty, queue_size=1)
        self.pub_land = rospy.Publisher('/bebop/land', Empty, queue_size=1)
        self.pub_camera = rospy.Publisher('/bebop/camera_control', Twist, queue_size=1)
        self.status_pub = rospy.Publisher('/mission/status', String, queue_size=1)

        # ========================
        # Movement + Vision
        # ========================

        # Ahora sí pasarlos
        self.movements = BebopMovements(
            self.pub_cmd,
            self.pub_takeoff,
            self.pub_land,
            self.pub_camera
        )

        # Perception module
        self.detector = BebopCameraProcessor()
        self.bridge = CvBridge()

        # ROS utilities
        self.image_sub = rospy.Subscriber(
            "/bebop/image_raw",
            Image,
            self.image_callback,
            queue_size=1,
            buff_size=2**24
        )


        # ========================
        # State Variables
        # ========================
        self.latest_image_msg = None   # SOLO guardamos mensaje
        self.latest_data = None
        self.last_detection_time = rospy.Time.now()

        self.finished = False
        self.center_tolerance = 25
        self.forward_counter = 0

        self.debug_image = None


        rospy.loginfo("Mission Orange Window initialized")

    # =====================================================
    # IMAGE CALLBACK (LIGERO - SIN PROCESAMIENTO)
    # =====================================================

    def image_callback(self, msg):
        if self.finished:
            return

        # 🔥 SOLO guardar la imagen más reciente
        self.latest_image_msg = msg


    # =====================================================
    # PROCESS IMAGE (FUERA DEL CALLBACK)
    # =====================================================

    def process_latest_image(self):

        if self.latest_image_msg is None:
            return

        try:
            frame = self.bridge.imgmsg_to_cv2(self.latest_image_msg, "bgr8")
        except Exception as e:
            rospy.logerr(f"CvBridge error: {e}")
            return

        # Reducir resolución
        frame = cv2.resize(frame, (640, 360))

        # =========================
        # MEDIR DELAY DE CÁMARA
        # =========================

        msg_time = self.latest_image_msg.header.stamp.to_sec()
        now_time = rospy.Time.now().to_sec()
        camera_delay = now_time - msg_time

        # =========================
        # MEDIR INFERENCIA
        # =========================

        start_time = rospy.Time.now()

        processed_image, data = self.detector.process_image(frame)

        end_time = rospy.Time.now()

        inference_time = (end_time - start_time).to_sec()

        # =========================
        # DIBUJAR INFO EN IMAGEN
        # =========================

        # Dibujar info en pantalla
        cv2.putText(processed_image,
                    f"Inference: {inference_time*1000:.1f} ms",
                    (20, 30),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.7,
                    (0, 255, 0),
                    2)

        cv2.putText(processed_image,
                    f"ROS time: {rospy.Time.now().to_sec():.2f}",
                    (20, 60),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.6,
                    (255, 255, 0),
                    2)

        self.debug_image = processed_image

        self.latest_data = data
        self.last_detection_time = rospy.Time.now()


    # =====================================================
    # IMAGE CALLBACK
    # =====================================================

#    def image_callback(self, msg):
#        if self.finished:
#            return
#        try:
#            frame = self.bridge.imgmsg_to_cv2(msg, "bgr8")
#        except Exception as e:
#            rospy.logerr(f"CvBridge error: {e}")
#            return
         # Reduce resolution for faster inference
#        frame = cv2.resize(frame, (640, 360)) # (480, 270)
#        processed_image, data = self.detector.process_image(frame)
        # Store latest detection
#        self.latest_data = data
#        self.last_detection_time = rospy.Time.now()

        # Optional debug window
#        cv2.imshow("Orange Detection", processed_image)
#        cv2.waitKey(1)


    # =====================================================
    # CONTROL LOGIC
    # =====================================================
    def control_logic(self):

        if self.latest_data is None:
            return

        # If detection is too old → ignore. Timeout detección
        if (rospy.Time.now() - self.last_detection_time).to_sec() > 0.3:
            rospy.loginfo("Detection timeout → searching")
            self.movements.turn_left("automatic")
            self.forward_counter = 0
            return

        data = self.latest_data

        if not data["detected"]:
            rospy.loginfo("Searching window...")
            self.movements.turn_left("automatic")
            self.forward_counter = 0
            return

        cx = data["cx"]
        center_x = data["center_x"]

        # ========================
        # ALIGNMENT LOGIC
        # ========================
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
    # MAIN LOOP
    # =====================================================

    def run(self):

        rate = rospy.Rate(15)  # 🔥 15 Hz reales

        while not rospy.is_shutdown():

            if not self.finished:

                # 1️⃣ Procesar última imagen disponible
                self.process_latest_image()

                # 2️⃣ Ejecutar control
                self.control_logic()

                # Muestra ka camara
                if self.debug_image is not None:
                    cv2.imshow("Orange Detection", self.debug_image)
                    cv2.waitKey(1)

            rate.sleep()


# =====================================================
# MAIN
# =====================================================

if __name__ == "__main__":
    mission = MissionOrangeWindow()
    mission.run()