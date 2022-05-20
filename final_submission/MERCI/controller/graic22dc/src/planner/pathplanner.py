import numpy as np
from matplotlib import pyplot as plt

from simulator.tracks.track1 import LaneInfo


class PathPlanner:
    def __init__(self, dbg=False) -> None:
        self.dbg = dbg

    def get_ref_state(self):
        raise NotImplementedError


class FollowFixedTracks(PathPlanner):
    def __init__(self, path, vref=100, **kwargs) -> None:
        super().__init__(**kwargs)
        self.path = path
        self.path_idx = 0
        self.vref = vref
        if self.dbg:
            self.ax = plt.subplot()
            self.ax.set_title("Path")
            self.counter = 0

    def get_ref_state(self, current_pose, obstacleList: list, lane_marker: LaneInfo, checkpoint):
        tol = 10
        loc = lane_marker.lane_markers_center.location[0]
        if self.dbg:
            self.ax.plot(loc.x, loc.y, "*", color="black")
            # self.ax.text(loc.x, loc.y, str(self.counter), color="red", fontsize=12)
            print(f"{self.counter=}, {(loc.x, loc.y)}")
            self.counter += 1

        return np.array((loc.x, loc.y, self.vref))


class FollowFixedTracks_old(PathPlanner):
    def __init__(self, path) -> None:
        super().__init__()
        self.path = path
        self.path_idx = 0

    def dist(self, xy1, xy2):
        return np.linalg.norm(np.array([xy1[0] - xy2[0], xy1[1] - xy2[1]]))

    def get_wp(self, cur_xy):
        tol = 10
        x, y = cur_xy
        curr_wp = self.path[self.path_idx]
        if self.dist(cur_xy, curr_wp) <= tol:
            if self.dbg:
                print(f"wpidx = {self.path_idx}")
            return self.__next_wp()
        else:
            return curr_wp

    def __next_wp(self):
        self.path_idx += 1
        return self.path[self.path_idx]
