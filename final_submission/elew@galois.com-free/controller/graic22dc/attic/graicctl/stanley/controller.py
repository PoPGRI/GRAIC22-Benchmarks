import numpy as np
from matplotlib import pyplot as plt
from ackermann_msgs.msg import AckermannDrive

from graicctl.stanley.utils import *

class Controller:
    def __init__(self, dbg=True, dt=None) -> None:
        self.dbg = dbg
        return

    def compute(self, target_pose, current_pose):
        if self.dbg:
            cx, cy, cvx, cvy, psi = self.unpack_current_pose(current_pose)
            tx, ty, target_v = self.unpack_target_pose(target_pose)
            print(f"{cx=}, {cy=}, {cvx=}, {cvy=}", end=", ")
            print_angles(psi=psi)
            print(f"{tx=}, {ty=}, {target_v=}")

    def plot(self):
        pass

    @staticmethod
    def unpack_current_pose(current_pose):
        current_position, current_rotation, current_velocity = current_pose
        psi = current_rotation[2]  # Euler
        x, y = current_position
        vx, vy = current_velocity
        return x, y, vx, vy, psi

    @staticmethod
    def unpack_target_pose(target_pose):
        x, y, v = target_pose
        return x, y, v


class PID(Controller):
    def __init__(self, dt=None) -> None:
        super().__init__()
        return

    def compute(self, target_pose, current_pose):
        super().compute(target_pose, current_pose)
        k_s = 0.1
        k_ds = 1
        k_n = 0.1
        k_theta = 1

        cx, cy, cvx, cvy, psi = self.unpack_current_pose(current_pose)
        tx, ty, target_v = self.unpack_target_pose(target_pose)

        # compute errors
        dx = tx - cx
        dy = ty - cy
        xError = (tx - cx) * np.cos(psi) + (ty - cy) * np.sin(psi)
        yError = -(tx - cx) * np.sin(psi) + (ty - cy) * np.cos(psi)
        curr_v = np.sqrt(cvx**2 + cvy**2)
        vError = target_v - curr_v

        delta = k_n * yError

        v = xError * k_s + vError * k_ds

        return v, delta

"""
class OpenLoop(Controller):
    def __init__(self, T) -> None:
        # super().__init__()
        self.U = np.ones((T, 2)) * 100
        self.U[:, UID.steering_angle] = np.sin(np.linspace(0, 5 * np.pi, num=T))
        self.idx = 0

    def control_input(self, prev_wp, wp, curr_xy, state):
        self.idx += 1
        return self.U[self.idx - 1, :]
"""

class StanleyPID(Controller):
    def __init__(self, SID, dt=None) -> None:
        super().__init__()
        self.prev_wp = None

        self.ACC = 100

        self.esum = 0
        self.e_prev = 0
        self.dt = dt
        self.plots = [[], [], []]
        self.SID = SID
        self.t = 0
        self.dt = 1  # internal time discretization constant used only for plotting

    # def pid(self, vref, v):
    #     Kp, Ki, Kd = 1, 1, 1
    #     e = vref - v
    #     self.esum += e
    #     ei = self.esum*dt
    #     ed = (e-self.e_prev)/dt
    #     ed = 0
    #     self.e_prev = e
    #     u = Kp*e + Ki*ei + Kd*ed
    #     return u

    def stanley(self, wp1, wp2, curr_xy, psi, v_long):
        #print(2)
        k = 0.3
        Kv = 1

        # Is (xc, yc) cog?
        xc, yc = curr_xy
        # equation of line: y = mx + c
        x1, y1 = wp1
        x2, y2 = wp2
        if x2 == x1:
            print(f"x1 == x2 == {x1}")
            raise AssertionError("x2 == x1")
        m = (y2 - y1) / (x2 - x1)
        c = y1 - m * x1

        # Trajectory line: a'x + b'y +c' = 0
        a, b, c = -m, 1, -c
        error_cross_track = (a * xc + b * yc + c) / np.sqrt(a**2 + b**2)
        yaw_path = np.arctan2(y2 - y1, x2 - x1)
        yaw_cross_track = np.arctan2(yc - y1, xc - x1)
        yaw_path_cross_track_diff = yaw_path - yaw_cross_track

        yaw_path_cross_track_diff = normalize_angle(yaw_path_cross_track_diff)
        if yaw_path_cross_track_diff > 0:
            error_cross_track = np.abs(error_cross_track)
        else:
            error_cross_track = -np.abs(error_cross_track)

        # cross_track_steering = np.arctan(k*error_cross_track/(Kv+v_long))
        cross_track_steering = np.arctan2(k * error_cross_track, (Kv + v_long))

        error_heading = yaw_path - psi
        error_heading = normalize_angle(error_heading)

        total_steering_angle_input = error_heading + cross_track_steering
        total_steering_angle_input = normalize_angle(total_steering_angle_input)

        print_angles(
            yaw_path=yaw_path,
            error_heading=error_heading,
            total_steering_angle_input=total_steering_angle_input,
        )

        self.t += self.dt
        self.plots[0].append(self.t)
        self.plots[1].append(np.rad2deg(cross_track_steering))
        self.plots[2].append(np.rad2deg(error_heading))

        return total_steering_angle_input

    def update_prev_and_current_wp(self, curr_xy, wp):
        if self.prev_wp is None:
            self.prev_wp = curr_xy
            self.curr_wp = wp

        if self.curr_wp[0] != wp[0] or self.curr_wp[1] != wp[1]:
            self.prev_wp = self.curr_wp
            self.curr_wp = wp
    """
    def compute_(self, wp, curr_xy, state, t):
        #super().compute(target_pose, current_pose)
        # print(prev_wp, wp)
        # print(self.phi)

        self.update_prev_and_current_wp(curr_xy, wp)

        # print(f"{prev_wp=}, {wp=}")
        steering_input = self.stanley(self.prev_wp, wp, curr_xy, state[SID.psi], state[SID.v_long])
        u = np.array([self.ACC, steering_input])
        # print(np.rad2deg(steering_input))
        return u
    """

    def compute(self, target_pose, current_pose):
        super().compute(target_pose, current_pose)
        x, y, vx, vy, psi = self.unpack_current_pose(current_pose)
        tx, ty, tv = self.unpack_target_pose(target_pose)
        wp = (tx, ty)
        curr_xy = (x, y)
        v_long = vx * np.cos(psi) + vy * np.sin(psi)

        self.update_prev_and_current_wp(curr_xy, wp)
        # print(prev_wp, wp)
        # print(self.phi)
        steering_input = self.stanley(self.prev_wp, wp, curr_xy, psi, v_long)
        u = np.array([self.ACC, steering_input])

        print(np.rad2deg(steering_input))
        if abs(steering_input) >= 2 * np.pi:
            print("steering input >= 2*np.pi")
        steering_input = normalize_angle(steering_input)

        return u

    def plot(self):
        fig1, ax1 = plt.subplots()
        plot_helper(
            ax1,
            np.asarray(self.plots[0]),
            np.asarray(self.plots[1]),
            "time",
            "",
            "",
            label="cross_track_steering",
        )
        plot_helper(
            ax1,
            np.asarray(self.plots[0]),
            np.asarray(self.plots[2]),
            "time",
            "u",
            "",
            label="error_heading",
        )
        ax1.legend()


class StanleyVehicleController():
    def stop(self):
        newAckermannCmd = AckermannDrive()
        newAckermannCmd.acceleration = -20
        newAckermannCmd.speed = 0
        newAckermannCmd.steering_angle = 0
        return newAckermannCmd

    def __init__(self):
        self.spid = StanleyPID(None)

    def execute(self, currentPose, targetPose):
        acc, delta = self.spid.compute(targetPose, currentPose)

        currentEuler = currentPose[1]
        curr_x = currentPose[0][0]
        curr_y = currentPose[0][1]

        target_x = targetPose[0]
        target_y = targetPose[1]
        target_v = targetPose[2]

        k_s = 0.4
        k_ds = 1

        # compute errors
        xError = (target_x - curr_x) * np.cos(
            currentEuler[2]) + (target_y - curr_y) * np.sin(currentEuler[2])
        curr_v = np.sqrt(currentPose[2][0] ** 2 + currentPose[2][1] ** 2)
        vError = target_v - curr_v

        # Checking if the vehicle need to stop
        if target_v > 0:
            v = xError * k_s + vError * k_ds
        else:
            v = 0.0

        newAckermannCmd = AckermannDrive()
        newAckermannCmd.speed = v
        newAckermannCmd.steering_angle = delta
        return newAckermannCmd
