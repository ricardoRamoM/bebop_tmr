#!/usr/bin/env python3
# mission_supervisor.py

import rospy
import subprocess
from enum import Enum
from std_msgs.msg import String


class GlobalState(Enum):
    IDLE = 0
    TAKING_OFF = 1
    HOVER_STABILIZE = 2
    MISSION_RUNNING = 3
    LANDING = 4
    EMERGENCY = 5


class MissionSupervisor:

    def __init__(self, movements):

        self.movements = movements
        self.state = GlobalState.IDLE

        self.current_process = None
        self.current_mission = None

        self.mission_start_time = None
        self.max_mission_time = 30.0  # Timeout global

        # Suscriptor al estado de misión
        rospy.Subscriber("/mission/status", String, self.mission_status_callback)

        rospy.loginfo("Mission Supervisor Ready")


    # =====================================================
    # INTERFAZ EXTERNA (desde teleop)
    # =====================================================

    def start_mission(self, mission_name):

        if self.state != GlobalState.IDLE:
            rospy.logwarn("Cannot start mission: not in IDLE")
            return

        self.current_mission = mission_name
        self.state = GlobalState.TAKING_OFF


    def abort_mission(self):        
        if self.state not in [
            GlobalState.TAKING_OFF,
            GlobalState.HOVER_STABILIZE,
            GlobalState.MISSION_RUNNING
        ]:
            return

        rospy.logwarn("Mission aborted")
        self.stop_mission()
        self.state = GlobalState.LANDING


    def emergency(self):

        rospy.logerr("EMERGENCY")
        self.stop_mission()
        self.state = GlobalState.EMERGENCY


    def is_running(self):
        return self.state in [
            GlobalState.TAKING_OFF,
            GlobalState.HOVER_STABILIZE,
            GlobalState.MISSION_RUNNING
        ]


    # =====================================================
    # FSM UPDATE
    # =====================================================

    def update(self):

        if self.state == GlobalState.IDLE:
            return

        elif self.state == GlobalState.TAKING_OFF:
            self.handle_takeoff()

        elif self.state == GlobalState.HOVER_STABILIZE:
            self.handle_hover()

        elif self.state == GlobalState.MISSION_RUNNING:
            self.check_timeout()

        elif self.state == GlobalState.LANDING:
            self.handle_landing()

        elif self.state == GlobalState.EMERGENCY:
            self.handle_emergency()


    # =====================================================
    # STATE HANDLERS
    # =====================================================

    def handle_takeoff(self):
        rospy.loginfo("TAKING OFF")
        self.movements.initial_takeoff("automatic")
        self.hover_start_time = rospy.Time.now()
        self.state = GlobalState.HOVER_STABILIZE


    def handle_hover(self):
        if (rospy.Time.now() - self.hover_start_time).to_sec() > 2.0:
            rospy.loginfo("HOVER STABLE → Launching mission")
            self.launch_mission()
            self.mission_start_time = rospy.Time.now()
            self.state = GlobalState.MISSION_RUNNING


    def handle_landing(self):
        rospy.loginfo("LANDING")
        self.movements.landing("automatic")
        self.state = GlobalState.IDLE

        self.current_mission = None
        self.mission_start_time = None


    def handle_emergency(self):
        rospy.logerr("FORCED LANDING (EMERGENCY)")
        self.movements.landing("automatic")
        self.state = GlobalState.IDLE

        self.current_mission = None
        self.mission_start_time = None



    # =====================================================
    # MISSION MANAGEMENT
    # =====================================================

    def launch_mission(self):

        rospy.loginfo(f"Launching mission: {self.current_mission}")

        self.current_process = subprocess.Popen(
            ["rosrun", "bebop_tmr", f"{self.current_mission}.py"]
        )


    def stop_mission(self):

        if self.current_process:
            self.current_process.terminate()
            self.current_process = None


    def mission_status_callback(self, msg):

        # Solo reaccionar si realmente hay misión corriendo
        if self.state != GlobalState.MISSION_RUNNING:
            return

        if msg.data == "done":
            rospy.loginfo("Mission reported DONE")
            self.stop_mission()
            self.state = GlobalState.LANDING

        elif msg.data == "failed":
            rospy.logwarn("Mission reported FAILED → EMERGENCY")
            self.stop_mission()
            self.state = GlobalState.EMERGENCY


    def check_timeout(self):

        if self.mission_start_time is None:
            return

        elapsed = (rospy.Time.now() - self.mission_start_time).to_sec()

        if elapsed > self.max_mission_time:
            rospy.logwarn("MISSION TIMEOUT")
            self.stop_mission()
            self.state = GlobalState.LANDING


# =====================================================
# TEST NODE (solo para validación independiente)
# VERSIÓN CORRECTA PARA VALIDAR SUPERVISOR SOLO
# =====================================================

if __name__ == "__main__":

    rospy.init_node("mission_supervisor_test")

    # ============================
    # Mock de movimientos
    # ============================

    class DummyMovements:
        def initial_takeoff(self, mode):
            rospy.loginfo("[MOCK] Takeoff")

        def landing(self, mode):
            rospy.loginfo("[MOCK] Landing")

    movements = DummyMovements()
    supervisor = MissionSupervisor(movements)

    rate = rospy.Rate(10)  # 10 Hz

    rospy.loginfo("Supervisor test running...")

    # Lanzar misión automáticamente después de 2 segundos
    rospy.sleep(2)
    supervisor.start_mission("mission_square")

    while not rospy.is_shutdown():
        supervisor.update()
        rate.sleep()


