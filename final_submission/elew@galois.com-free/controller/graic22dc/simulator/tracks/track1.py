from __future__ import annotations
from typing import Sequence

import numpy as np

# from graic_msgs.msg import LaneInfo


class LaneLocation:
    pass


class Vector:
    @classmethod
    def from_rotation(cls, psi: float) -> Vector:
        assert psi >= 0 and psi <= 360
        return cls(x=360, y=psi, z=0)

    @classmethod
    def from_location(cls, x: float, y: float) -> Vector:
        return cls(x=x, y=y, z=0.0)

    def __init__(self, x: float, y: float, z: float) -> None:
        self.x = x
        self.y = y
        self.z = z


class LaneList:
    @classmethod
    def get_lane_list20(cls, psi20, xy20) -> LaneList:
        location = [Vector.from_location(*xy) for xy in xy20]
        rotation = [Vector.from_rotation(psi) for psi in psi20]
        return cls(location, rotation)

    def __init__(self, location: list[Vector], rotation: list[Vector]) -> None:
        assert len(location) == 20
        assert len(rotation) == 20
        self.location: list[Vector] = location
        self.rotation: list[Vector] = rotation


class LaneInfo:
    @classmethod
    def from_path20(cls, psi20, xy20_left, xy20_center, xy20_right) -> LaneInfo:
        lane_markers = LaneList.get_lane_list20(psi20, xy20_center)
        lane_markers_left = LaneList.get_lane_list20(psi20, xy20_left)
        lane_markers_right = LaneList.get_lane_list20(psi20, xy20_right)
        return cls(lane_markers, lane_markers_left, lane_markers_right)

    def __init__(self, lane_markers_center, lane_markers_left, lane_markers_right) -> None:
        LANE_ID = 3
        self.LEFT_LANE = LANE_ID - 1
        self.CENTER_LANE = LANE_ID
        self.RIGHT_LANE = LANE_ID + 1
        self.lane_state = self.CENTER_LANE

        self.lane_markers_center = lane_markers_center
        self.lane_markers_left = lane_markers_left
        self.lane_markers_right = lane_markers_right

        # serialize,deserialize,serialize_numpy,deserialize_numpy


class Track:
    def __init__(self, dbg=False) -> None:
        self.dbg = dbg

    def get_lane_info(xy: Sequence):
        raise NotImplementedError

    @property
    def path_center(self) -> np.ndarray:
        raise NotImplementedError

    @property
    def path_left(self) -> np.ndarray:
        raise NotImplementedError

    @property
    def path_right(self) -> np.ndarray:
        raise NotImplementedError


def circle2(x, radius=500, track_width=10):
    # Path in x-y
    eps = 0.01
    track_width = radius
    path_left, path_center, path_right = (
        np.sqrt((radius - track_width) ** 2 - x**2),
        np.sqrt(radius**2 - x**2),
        np.sqrt((radius + track_width) ** 2 - x**2),
    )
    return path_left, path_center, path_right


# RBD: reimplement in sympy
def circle(radius=500, track_width=10, numPoints=100):
    # Path in x-y
    eps = 0.01
    n = np.linspace(0, 2 * (np.pi - eps), num=numPoints)
    y = np.vstack((np.cos(n), np.sin(n))).T
    path_left, path_center, path_right = (radius - track_width) * y, radius * y, (radius + track_width) * y
    return path_left, path_center, path_right


# RBD: reimplement in sympy
def sinusoid_perturbed_circle(perturbation_amp=50, radius=500, track_width=10, numPoints=100):
    assert numPoints >= 20
    # Path in x-y
    # path = np.array(
    #     ([[radius * (np.cos(phi)), radius * (np.sin(phi))] for phi in np.linspace(0, 2 * np.pi, num=numPoints)])
    # )
    path_left, path_center, path_right = circle(radius, track_width, numPoints)
    path_left[:, 0] += perturbation_amp * np.sin(np.linspace(0, 20 * 2 * np.pi, num=numPoints))
    path_center[:, 0] += perturbation_amp * np.sin(np.linspace(0, 20 * 2 * np.pi, num=numPoints))
    path_right[:, 0] += perturbation_amp * np.sin(np.linspace(0, 20 * 2 * np.pi, num=numPoints))
    return path_left, path_center, path_right


class SimpleTrack2(Track):
    """
    Provides a common class that implements a track using a mathematical function
    """

    pass


class SimpleTrack(Track):
    """
    Provides a common class that implements a track using a series of discrete x-y points as input.

    TBD: Needs to implement checkpoints to avoid jumps caused by track symmetry.
    """

    def __init__(self, path_left: np.ndarray, path_center: np.ndarray, path_right: np.ndarray, **kwargs) -> None:
        super().__init__(**kwargs)
        self.idx = 0
        self.__path_left = path_left
        self.__path_center = path_center
        self.__path_right = path_right

    @property
    def path_center(self) -> np.ndarray:
        return self.__path_center

    @property
    def path_left(self) -> np.ndarray:
        return self.__path_left

    @property
    def path_right(self) -> np.ndarray:
        return self.__path_right

    def closest_point_idx(self, xy) -> int:
        """
        Returns the index of the (tx,ty) \in path that is at the minimun distance from xy
        arg. min. dist(txy, xy)
            s.t. txy \in track
        """
        # return np.argmin(np.linalg.norm(self.path_center - xy, axis=1))
        return np.argmin(np.sqrt((self.__path_center[:, 0] - xy[0]) ** 2 + (self.__path_center[:, 1] - xy[1]) ** 2))

    def lane_info_from_path20(self, path20_left, path20_center, path20_right) -> LaneInfo:
        return LaneInfo.from_path20([0] * 20, path20_left, path20_center, path20_right)

    def get_lane_info(self, xy) -> LaneInfo:
        # default to the next of the closest
        # , as the closest point might not be always in 'front' of the car
        self.idx = max(self.closest_point_idx(xy) + 1, self.idx)
        if self.dbg:
            print(f"{self.idx=}, {self.__path_center[self.idx]=}, {xy=}")

        # path20_center = self.path_center[idx : idx + 20, :]
        # path20_left = self.path_left[idx : idx + 20, :]
        # path20_right = self.path_right[idx : idx + 20, :]

        path20_center = np.roll(self.__path_center, shift=-self.idx, axis=0)[:20]
        path20_left = np.roll(self.__path_left, shift=-self.idx, axis=0)[:20]
        path20_right = np.roll(self.__path_right, shift=-self.idx, axis=0)[:20]

        return self.lane_info_from_path20(path20_left, path20_center, path20_right)


class Baseline(Track):
    def __init__(self) -> None:
        super().__init__()
