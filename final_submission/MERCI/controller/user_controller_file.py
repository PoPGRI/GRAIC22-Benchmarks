import sys
import traceback

## The below is carried out in bash script
# # install what we use
# import subprocess
# import sys

# def install(package):
#     subprocess.check_call([sys.executable, "-m", "pip", "install", package])


# install("sklearn")
# install("joblib")


from ackermann_msgs.msg import AckermannDrive

from graic_msgs.msg import ObstacleList, ObstacleInfo
from graic_msgs.msg import LocationInfo, WaypointInfo
from carla_msgs.msg import CarlaEgoVehicleControl
from graic_msgs.msg import LaneList
from graic_msgs.msg import LaneInfo
import numpy as np

import graic22dc.src.planner.user_decision as user_decision
import graic22dc.src.control.user_control as user_control
from graic22dc.src.control.baseline import VehicleController
import graic22dc.src.planner.rrt.decision as rrt_decision
from graic22dc.src.planner.recorder import VehicleRecorder
import atexit


class TracePrints(object):
    def __init__(self):
        self.stdout = sys.stdout

    def write(self, string):
        self.stdout.write(f"Writing {string}")
        traceback.print_stack(file=self.stdout)


# Enable this to trace mysterious print statements
# sys.stdout = TracePrints()


class Controller(object):
    """docstring for Controller"""

    def __init__(self):
        super(Controller, self).__init__()

        # self.decisionModule = user_decision.VehicleDecision()
        self.decisionModule = rrt_decision.RRTStarDecision(
            v_max=30.0,
            v_min=20.0,
            path_resolution=0.3,
            expand_dis=0.3,
            connect_circle_dist=5.0,
            max_iter=500,
            goal_sample_rate=65.0,
            predictor_steps=3,
            predictor_delta=10,
            vehicle_radius=1.4,
            person_radius=2.5,
            lanes_radius=2.0,
            lookahead_mult=3.0,
            road_subdivisions=7,
        )
        self.last_path = None
        self.last_goal = None
        self.elevation = 0.0

        # select planner
        # self.decisionModule = user_decision.Baseline(LaneInfo)
        # self.decisionModule = VehicleRecorder(rrt_decision.RRTStarDecision(), "rrt_signals.txt")

        # select low level controller
        self.controller = user_control.Baseline_PID(dbg=False)
        #self.controller = user_control.StanleyPID(dbg=False)
        #self.controller = user_control.Zero(dbg=True)
        self.controlModule = VehicleController(self.controller)
        atexit.register(self.atexit)

    def stop(self):
        return self.controlModule.stop()

    def execute(self, currState, obstacleList, lane_marker, waypoint):
        # Get the target state from decision module
        refState = self.decisionModule.get_ref_state(currState, obstacleList, lane_marker, waypoint)
        self.last_path = self.decisionModule.last_path
        self.elevation = self.decisionModule.elevation
        self.last_goal = self.decisionModule.last_state
        if not refState:
            return None
        return self.controlModule.execute(currState, refState)

    def atexit(self):
        from matplotlib import pyplot as plt

        self.controller.plot_me()
        plt.show()
