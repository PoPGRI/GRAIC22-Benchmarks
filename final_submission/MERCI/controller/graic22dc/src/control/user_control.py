import numpy as np

from graic22dc.src.common.utils import print_angles, normalize_angle, clip
from graic22dc.src.control.controller import Controller


class Baseline_PID(Controller):
    """
    GRAICC'22 Baseline controller
    """

    def __init__(self, **kwargs) -> None:
        super().__init__(plot_names=["acceleration", "steering_angle_input"], **kwargs)
        return

    @Controller.plotter
    def compute(self, target_pose, current_pose):
        super().compute(target_pose, current_pose)
        k_s = 0.1
        k_ds = 1
        k_n = 0.15
        k_theta = 1.0

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
        if self.dbg:
            print(f"{xError=}, {yError=}, {vError=}, {v=}")
        return v, delta

    @property
    def name(self) -> str:
        return "Baseline controller"


# class OpenLoop(Controller):
#     def __init__(self, T) -> None:
#         # super().__init__()
#         self.U = np.ones((T, 2)) * 100
#         self.U[:, UID.steering_angle] = np.sin(np.linspace(0, 5 * np.pi, num=T))
#         self.idx = 0

#     def control_input(self, prev_wp, wp, curr_xy, state):
#         self.idx += 1
#         return self.U[self.idx - 1, :]


class StanleyPID(Controller):
    MAX_STEER = np.deg2rad(45)

    def __init__(self, **kwargs) -> None:
        super().__init__(plot_names=["steering_angle_input", "heading_steer", "xtrack_steer"], **kwargs)
        self.prev_wp = None
        self.esum = 0
        self.e_prev = 0

    @property
    def name(self) -> str:
        return "StanleyPID"

    def pid_velocity(self, vref, v_long):
        Kp, Ki, Kd = 1, 0.0, 0.0
        e = vref - v_long
        self.esum += e
        ei = self.esum * self.dt
        ed = (e - self.e_prev) / self.dt
        self.e_prev = e
        if self.dbg:
            print(f"{e=}, {ei=}, {ed=}")
        u = Kp * e + Ki * ei + Kd * ed
        return u

    @Controller.plotter
    def stanley(self, wp1, wp2, curr_xy, psi, v_long):
        kx = 0.6
        Ks = 2
        Kd = 0.4

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
        cross_track_steering = np.arctan2(kx * error_cross_track, (Ks + Kd * v_long))

        error_heading = yaw_path - psi
        error_heading = normalize_angle(error_heading)

        total_steering_angle_input = error_heading + cross_track_steering
        total_steering_angle_input = normalize_angle(total_steering_angle_input)
        total_steering_angle_input = clip(total_steering_angle_input, (-StanleyPID.MAX_STEER, StanleyPID.MAX_STEER))

        if self.dbg:
            print_angles(
                yaw_path=yaw_path,
                error_heading=error_heading,
                total_steering_angle_input=total_steering_angle_input,
            )

        return total_steering_angle_input, error_heading, cross_track_steering

    def update_prev_and_current_wp(self, curr_xy, wp):
        if self.prev_wp is None:
            self.prev_wp = curr_xy
            self.curr_wp = wp

        if self.curr_wp[0] != wp[0] or self.curr_wp[1] != wp[1]:
            self.prev_wp = self.curr_wp
            self.curr_wp = wp

        if self.dbg:
            print(f"{self.prev_wp=}, {self.curr_wp}")

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
        steering_input, _, _ = self.stanley(self.prev_wp, wp, curr_xy, psi, v_long)
        # u = np.array([self.pid_velocity(tv, v_long), steering_input])
        u = np.array([tv, steering_input])
        # u = np.array([0, steering_input])
        return u


class PurePursuitPID(Controller):
    def __init__(self, dt) -> None:
        return

    def pure_pursuit(self, wp1, wp2, curr_xy, state):
        raise NotImplementedError
        return total_steering_angle_input

    def control_input(self, prev_wp, wp, curr_xy, state):
        raise NotImplementedError


class MPC(Controller):
    def __init__(self, dt) -> None:
        return

    def mpc(self, wp1, wp2, curr_xy, state):
        raise NotImplementedError
        return total_steering_angle_input

    def control_input(self, prev_wp, wp, curr_xy, state):
        raise NotImplementedError


class Zero(Controller):
    """
    The Zero controller outputs all 0s. It is for testing the system response only.
    """

    def __init__(self, **kwargs) -> None:
        super().__init__(**kwargs)
        return

    def compute(self, target_pose, current_pose):
        super().compute(target_pose, current_pose)
        v, delta = 0.0, 0.0
        return v, delta

    @property
    def name(self) -> str:
        return "Zero controller"
