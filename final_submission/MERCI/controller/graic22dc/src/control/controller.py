from matplotlib import pyplot as plt
import numpy as np

from graic22dc.src.common.utils import print_angles
from graic22dc.src.common.utils import plot_helper


class Controller:
    @staticmethod
    def plotter(fn):
        def wrapper(self, *args, **kwargs):
            ret = fn(self, *args, **kwargs)
            assert ret is not None

            #if self.nplots == 0:
            #    pass
            #elif self.nplots == 1:
            #    self.plots[0].append(ret)
            #else:
            #    assert len(ret) == self.nplots
            #    for idx, ret_val in enumerate(ret):
            #        self.plots[idx].append(ret_val)
            #self.times.append(self.t)
            return ret

        return wrapper

    def __init__(self, plot_names=[], dbg=False, dt=1) -> None:
        self.dbg = dbg
        self.dt = dt
        self.plot_names = plot_names
        self.nplots = len(plot_names)
        self.plots = [[] for i in range(self.nplots + 1)]
        self.times = []
        self.t = 0
        print(f"Running with {self.name}")
        return

    def compute(self, target_pose, current_pose):
        self.t += self.dt
        if self.dbg:
            cx, cy, cvx, cvy, psi = self.unpack_current_pose(current_pose)
            tx, ty, target_v = self.unpack_target_pose(target_pose)
            print(f"{cx=}, {cy=}, {cvx=}, {cvy=}", end=", ")
            print_angles(psi=psi)
            print(f"{tx=}, {ty=}, {target_v=}")

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

    @property
    def name(self) -> str:
        raise NotImplementedError

    def plot_me(self):
        return
        """
        fig1, ax1 = plt.subplots()
        for idx, plot_name in enumerate(self.plot_names):
            plot_helper(
                ax1,
                np.asarray(self.times),
                np.asarray(self.plots[idx]),
                "time",
                "",
                "",
                label=plot_name,
                marker="*",
            )
        # from IPython import embed; embed()
        print(self.plot_names)
        # plot_helper(
        #     ax1,
        #     np.asarray(self.plots[0]),
        #     np.asarray(self.plots[2]),
        #     "time",
        #     "u",
        #     "",
        #     label="error_heading",
        # )
        # plot_helper(
        #     ax1,
        #     np.asarray(self.plots[0]),
        #     np.asarray(self.plots[3]),
        #     "time",
        #     "",
        #     "",
        #     label="total_steering_input",
        # )
        ax1.legend()
        """
