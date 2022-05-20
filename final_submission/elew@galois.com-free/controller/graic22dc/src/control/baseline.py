import numpy as np

from ackermann_msgs.msg import AckermannDrive
from graic22dc.src.control.controller import Controller


# class Controller(object):
#     """docstring for Controller"""

#     def __init__(self, decision_module: BaselineVehicleDecision, control_module=None):
#         super(BaselineController, self).__init__()
#         self.decisionModule = decision_module
#         if control_module is None:
#             self.controlModule = BaselineVehicleController()
#         else:
#             self.controlModule = control_module

#     def stop(self):
#         return self.controlModule.stop()

#     def execute(self, currState, obstacleList, lane_marker, waypoint):
#         # Get the target state from decision module
#         refState = self.decisionModule.get_ref_state(currState, obstacleList, lane_marker, waypoint)
#         if not refState:
#             return None
#         return self.controlModule.execute(currState, refState)


class VehicleController:
    def __init__(self, user_controller: Controller) -> None:
        self.controller = user_controller

    def stop(self):
        newAckermannCmd = AckermannDrive()
        newAckermannCmd.acceleration = -20
        newAckermannCmd.speed = 0
        newAckermannCmd.steering_angle = 0
        return newAckermannCmd

    # def execute(self, current_pose, target_pose):
    #     """
    #     This function takes the current state of the vehicle and
    #     the target state to compute low-level control input to the vehicle
    #     Inputs:
    #         currentPose: ModelState, the current state of vehicle
    #         targetPose: The desired state of the vehicle
    #     """

    #     # Checking if the vehicle need to stop
    #     target_v = target_pose[2]
    #     if target_v > 0:
    #         v, delta = self.controller.compute(target_pose, current_pose)
    #         # Send computed control input to vehicle
    #         newAckermannCmd = AckermannDrive()
    #         newAckermannCmd.speed = v
    #         newAckermannCmd.steering_angle = delta
    #         return newAckermannCmd
    #     else:
    #         return self.stop()
    def execute(self, current_pose, target_pose):
        """
        This function takes the current state of the vehicle and
        the target state to compute low-level control input to the vehicle
        Inputs:
            currentPose: ModelState, the current state of vehicle
            targetPose: The desired state of the vehicle
        """

        # Checking if the vehicle need to stop
        target_v = target_pose[2]

        cx, cy, cvx, cvy, psi = self.controller.unpack_current_pose(current_pose)
        actual_v = np.sqrt(cvx**2 + cvy**2)

        v, delta = self.controller.compute(target_pose, current_pose)
        #print(f"speed {v}, actual {actual_v}")
        # Send computed control input to vehicle
        newAckermannCmd = AckermannDrive()
        newAckermannCmd.speed = v
        newAckermannCmd.steering_angle = delta
        if target_v < 1.0:
            newAckermannCmd.acceleration = -20
        return newAckermannCmd
