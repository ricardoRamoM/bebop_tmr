#!/usr/bin/env python3
import rospy
from std_msgs.msg import String, Bool, Empty
from enum import Enum
import subprocess
import time

class State(Enum):
    IDLE = 0
    TAKING_OFF = 1
    HOVER_STABILIZE = 2
    MISSION_RUNNING = 3
    LANDING = 4
    EMERGENCY = 5


class MissionSupervisor:

    def __init__(self):

        rospy.init_node("mission_supervisor")

        self.state = State.IDLE
        self.current_process = None
        self.current_mission = None
        self.mission_start_time = None
        self.max_mission_time = 20.0  # segundos (ajústalo según TMR)

        # Publishers
        self.takeoff_pub = rospy.Publisher('/bebop/takeoff', Empty, queue_size=1)
        self.land_pub = rospy.Publisher('/bebop/land', Empty, queue_size=1)

        # Subscribers
        rospy.Subscriber('/bebop/mode', String, self.mode_callback)
        rospy.Subscriber('/bebop/emergency', Bool, self.emergency_callback)
        rospy.Subscriber('/bebop/mission_status', String, self.mission_status_callback)

        self.rate = rospy.Rate(10)

        rospy.loginfo("Mission Supervisor FSM started")
        self.run()

    # ===============================
    # MAIN LOOP
    # ===============================
    def run(self):
        while not rospy.is_shutdown():

            if self.state == State.IDLE:
                pass

            elif self.state == State.TAKING_OFF:
                self.handle_takeoff()

            elif self.state == State.HOVER_STABILIZE:
                self.handle_stabilize()

            elif self.state == State.MISSION_RUNNING:
                self.check_mission_timeout()

            elif self.state == State.LANDING:
                self.handle_landing()

            elif self.state == State.EMERGENCY:
                self.handle_emergency()

            self.rate.sleep()

    # ===============================
    # STATE HANDLERS
    # ===============================

    def handle_takeoff(self):
        rospy.loginfo("State: TAKING_OFF")
        self.takeoff_pub.publish()
        rospy.sleep(3.0)  # tiempo para despegar
        self.state = State.HOVER_STABILIZE

    def handle_stabilize(self):
        rospy.loginfo("State: HOVER_STABILIZE")
        rospy.sleep(2.0)  # estabilización
        self.launch_mission()
        self.mission_start_time = rospy.Time.now()
        self.state = State.MISSION_RUNNING

    def handle_landing(self):
        rospy.loginfo("State: LANDING")
        self.land_pub.publish()
        rospy.sleep(3.0)
        self.state = State.IDLE
        rospy.loginfo("Back to IDLE")

    def handle_emergency(self):
        rospy.logwarn("State: EMERGENCY")
        self.stop_mission()
        self.land_pub.publish()
        rospy.sleep(3.0)
        self.state = State.IDLE

    # ===============================
    # CALLBACKS
    # ===============================

    def mode_callback(self, msg):

        if "mission" in msg.data and self.state == State.IDLE:
            self.current_mission = msg.data
            self.state = State.TAKING_OFF

        elif msg.data == "manual":
            rospy.loginfo("Manual mode selected")

    def emergency_callback(self, msg):
        if msg.data:
            self.state = State.EMERGENCY

    def mission_status_callback(self, msg):
        if msg.data == "completed" and self.state == State.MISSION_RUNNING:
            rospy.loginfo("Mission completed internally")
            self.state = State.LANDING

    # ===============================
    # MISSION CONTROL
    # ===============================

    def launch_mission(self):

        if self.current_process:
            self.current_process.terminate()

        rospy.loginfo(f"Launching mission: {self.current_mission}")

        self.current_process = subprocess.Popen(
            ["rosrun", "bebop_control", self.current_mission + ".py"]
        )

    def stop_mission(self):

        if self.current_process:
            self.current_process.terminate()
            self.current_process = None

    def check_mission_timeout(self):
        if self.mission_start_time is None:
            return

        elapsed = (rospy.Time.now() - self.mission_start_time).to_sec()

        if elapsed > self.max_mission_time:
            rospy.logwarn("MISSION TIMEOUT REACHED")
            self.stop_mission()
            self.state = State.LANDING


if __name__ == "__main__":
    MissionSupervisor()
