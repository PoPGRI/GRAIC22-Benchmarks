from scipy.interpolate import UnivariateSpline
import matplotlib.pyplot as plt
import numpy as np
from typing import Sequence, Callable
import abc
import typing as typ
from sklearn.preprocessing import normalize
import yaml
import scipy.interpolate as interpolate


class RoadSegment:
    """road segment"""
    step = 1 / 20.0

    def __init__(self, left_lane: np.ndarray, center_lane: np.ndarray, right_lane: np.ndarray, lane_state: int):
        self.right_lane = right_lane
        self.center_lane = center_lane
        self.left_lane = left_lane
        self.lane_state = lane_state

    def _fit_lane(self, lane_pts) -> typ.Callable[[float], np.ndarray]:
        def _fit_split(ys):
            ts = np.linspace(0, 1.0, len(ys))
            #print(ts.shape, ys.shape)
            t, c, k = interpolate.splrep(ts, ys, s=2.0, k=2)
            return interpolate.BSpline(t, c, k, extrapolate=True)

        splx = _fit_split(lane_pts[:, 0]) #UnivariateSpline(np.arange(0, 1.0, self.step), lane_pts[:, 0], k=2)
        sply = _fit_split(lane_pts[:, 1]) #UnivariateSpline(np.arange(0, 1.0, self.step), lane_pts[:, 1], k=2)
        splz = _fit_split(lane_pts[:, 2]) #UnivariateSpline(np.arange(0.0, 1.0, self.step), lane_pts[:, 2], k=2)

        def _lane(t):
            return np.hstack((splx(t)[:, np.newaxis], sply(t)[:, np.newaxis], splz(t)[:, np.newaxis]))

        return _lane

    def get_segment(self, start: float, stop: float) -> 'RoadSegment':
        """extrapolate road for a larger horizon"""
        t = np.arange(start, stop, self.step)
        return RoadSegment(self._fit_lane(self.left_lane)(t), self._fit_lane(self.center_lane)(t),
                           self._fit_lane(self.right_lane)(t), self.lane_state)

    def get_bounds_segment(self, start, stop):
        """parallel curve boundary estimator"""
        def _fit_split(ys):
            ts = np.linspace(0, 1.0, len(ys))
            t, c, k = interpolate.splrep(ts, ys, s=1.0, k=3)
            return interpolate.BSpline(t, c, k, extrapolate=True)

        # get our bounds
        l, r = self.get_segment(start, stop).bounds

        # road width id median of present data
        dist = np.median(np.linalg.norm(l - r, axis=1) / 2.0)

        # fit spline
        center = (l[:, :2] + r[:, :2]) / 2.0
        ts = np.linspace(0, 1.0, len(center))
        sx = _fit_split(center[:, 0]).derivative()
        sy = _fit_split(center[:, 1]).derivative()

        # apply offset
        normal = dist * normalize(np.hstack((-sy(ts)[:, np.newaxis], sx(ts)[:, np.newaxis])), axis=1)
        return center + normal, center - normal

    def plot_road(self, ax: plt.Axes, prefix=""):
        ax.plot(*self.right_lane[:, :2].T, 'k', label=f"{prefix} Outer Lane")
        ax.plot(*self.left_lane[:, :2].T, 'k')
        ax.plot(*self.center_lane[:, :2].T, 'r', label=f"{prefix} Center Lane")

    @property
    def locations(self) -> typ.Tuple[np.ndarray, np.ndarray, np.ndarray]:
        return self.left_lane, self.center_lane, self.right_lane

    @property
    def bounds(self) -> typ.Tuple[np.ndarray, np.ndarray]:
        if self.lane_state == 5:
            return self.right_lane, self.left_lane + 2 * (self.left_lane - self.right_lane)
        elif self.lane_state == 4:
            return self.left_lane + 2 * (self.left_lane - self.center_lane), self.right_lane + 2 * (
                        self.right_lane - self.center_lane)
        elif self.lane_state == 3:
            return self.right_lane + 2 * (self.right_lane - self.left_lane), self.left_lane


class Obstacle:
    """person and vehicle obstacle"""

    def __init__(self, verts):
        self.verts = verts

    @property
    def center(self) -> np.ndarray:
        return self.verts.mean(axis=0)


class CurrentState:
    """current state provided from GRAIC"""

    @staticmethod
    def get_xyz(point: typ.Dict[str, float]) -> typ.Tuple[float, float, float]:
        return (point['x'], point['y'], point['z'])

    @staticmethod
    def get_lanes_locations(record) -> typ.Tuple[np.ndarray, np.ndarray, np.ndarray]:
        c = np.array([CurrentState.get_xyz(t) for t in record["lane_marker"]["lane_markers_center"]["location"]])
        l = np.array([CurrentState.get_xyz(t) for t in record["lane_marker"]["lane_markers_left"]["location"]])
        r = np.array([CurrentState.get_xyz(t) for t in record["lane_marker"]["lane_markers_right"]["location"]])
        return l, c, r

    @staticmethod
    def get_obstacles_verts(crecord: typ.Dict[str, typ.Any]) -> np.ndarray:
        obstacles = crecord['obstacles']
        verts = []
        for obstacle in obstacles:
            pt = [(t['vertex_location']['x'], t['vertex_location']['y'], t['vertex_location']['z']) for t in
                  obstacle['vertices_locations']]
            verts.append(np.array(pt))
        return verts

    @classmethod
    def from_dict(cls, crecord: typ.Dict[str, typ.Any]) -> "CurrentState":
        state = crecord["state"][0]
        velocity = crecord["state"][-1]
        # pack the current state into a CurrentState object
        lane_locs = cls.get_lanes_locations(crecord)
        r = RoadSegment(*lane_locs, crecord["lane_marker"]["lane_state"])
        cs = CurrentState(r,
                          state,
                          velocity,
                          cls.get_xyz(crecord['waypoint']['location']),
                          [Obstacle(v) for v in cls.get_obstacles_verts(crecord)])
        return cs

    def __init__(self,
                 road: RoadSegment,
                 cpos: np.ndarray,
                 cvelocity: np.ndarray,
                 waypoint: np.ndarray,
                 obstacles: Sequence[Obstacle]):
        self.road = road
        self.cpos = cpos
        self.cvelocity = cvelocity
        self.waypoint = waypoint
        self.obstacles = obstacles


class VehicleDecisionBase(abc.ABC):
    """decision interface to plug into baseline controller"""

    @abc.abstractmethod
    def get_ref_state(self, curr_state, obstacle_list, lane_marker, waypoint) -> typ.Tuple[float, float, float]:
        raise NotImplementedError
