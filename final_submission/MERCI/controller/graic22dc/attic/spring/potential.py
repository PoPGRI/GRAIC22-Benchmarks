"""
Simple Potential Field Path Following Implementation
"""
from typing import Sequence, Callable

import matplotlib.pyplot as plt
import numpy as np
import yaml
from ackermann_msgs.msg import AckermannDrive
from scipy.integrate import solve_ivp
from scipy.interpolate import UnivariateSpline


class RoadSegment:
    """road segment"""
    step = 1 / 20.0

    def __init__(self, left_lane, center_lane, right_lane):
        self.right_lane = right_lane
        self.center_lane = center_lane
        self.left_lane = left_lane

    def _fit_lane(self, lane_pts):
        splx = UnivariateSpline(np.arange(0, 1.0, self.step), lane_pts[:, 0], k=2)
        sply = UnivariateSpline(np.arange(0, 1.0, self.step), lane_pts[:, 1], k=2)
        splz = UnivariateSpline(np.arange(0.0, 1.0, self.step), lane_pts[:, 2], k=2)

        def _lane(t):
            return np.hstack((splx(t)[:, np.newaxis], sply(t)[:, np.newaxis], splz(t)[:, np.newaxis]))

        return _lane

    def get_segment(self, start, stop):
        """extrapolate road for a larger horizon"""
        t = np.arange(start, stop, self.step)
        return RoadSegment(self._fit_lane(self.right_lane)(t), self._fit_lane(self.center_lane)(t),
                           self._fit_lane(self.left_lane)(t))

    def plot_road(self, ax: plt.Axes, prefix=""):
        ax.plot(*self.right_lane[:, :2].T, 'k', label=f"{prefix} Outer Lane")
        ax.plot(*self.left_lane[:, :2].T, 'k')
        ax.plot(*self.center_lane[:, :2].T, 'r', label=f"{prefix} Center Lane")

    @property
    def locations(self):
        return self.left_lane, self.center_lane, self.right_lane


class Obstacle:
    """person and vehicle obstacle"""

    def __init__(self, verts):
        self.verts = verts

    @property
    def center(self):
        return self.verts.mean(axis=0)


class CurrentState:
    """current state provided from GRAIC"""

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


class PotentialField:
    def __init__(self, lane_penalty=0.5, obstacle_penalty=1.0, goal_reward=1.0):
        self.lane_penalty = lane_penalty
        self.obstacle_penalty = obstacle_penalty
        self.goal_reward = goal_reward

    @staticmethod
    def compute_forces(pt, obstacles, penalty):
        """forces produced from obstacles"""
        dirs_l = (pt - obstacles)
        dists_l = np.linalg.norm(dirs_l, axis=1)
        dirsn_l = dirs_l / dists_l[:, np.newaxis]
        forces_l = penalty * dirsn_l / (dists_l[:, np.newaxis] ** 2)
        return forces_l

    @staticmethod
    def compute_forces_goal(pt, obstacles, penalty):
        """a force that doesn't attenuate (global goal)"""
        dirs_l = (pt - obstacles)
        dists_l = np.linalg.norm(dirs_l, axis=1)
        dirsn_l = dirs_l / dists_l[:, np.newaxis]
        forces_l = penalty * dirsn_l  # / (dists_l[:, np.newaxis]**2)
        return forces_l

    def get_force_field(self, cstate: CurrentState) -> Callable[[np.ndarray], np.ndarray]:
        """get function that will produce net force for given points"""
        road, waypoint, obstacles = cstate.road, cstate.waypoint, cstate.obstacles

        def force(points):
            points = np.array(points)
            forces_ret = np.zeros(points.shape)

            # extrapolate road backwards and forwards
            left_lane, center_lane, right_lane = road.get_segment(-1.0, 3.0).locations

            left_lane += 2 * (left_lane - center_lane)
            right_lane += 2 * (right_lane - center_lane)

            # get center of obstacles for the repellers
            centers = [obsv.center for obsv in obstacles]
            obstacle_verts = np.vstack(centers) if len(centers) > 0 else []

            for idx, pts in enumerate(points):
                # lane marker forces
                forces_lanes = self.compute_forces(pts, np.vstack((left_lane, right_lane)), self.lane_penalty)

                # waypoint force
                forces_goals = -self.compute_forces_goal(pts, np.atleast_2d(waypoint), self.goal_reward)

                # obstacle forces
                if len(obstacle_verts) > 0:
                    forces_obstacles = self.compute_forces(pts, obstacle_verts, self.obstacle_penalty)
                    forces_all = np.vstack((forces_lanes, forces_obstacles, forces_goals))
                else:
                    forces_all = np.vstack((forces_lanes, forces_goals))

                # get net force
                forces_ret[idx] = np.sum(forces_all, axis=0)
            return forces_ret

        return force

    def get_trajectory(self, cstate, t_eval):
        """integrate forces for trajectories"""
        field = self.get_force_field(cstate)

        def ode_sys(t, y):
            return field([y])[0]

        state = cstate.cpos
        sol = solve_ivp(ode_sys, [np.min(t_eval), np.max(t_eval)], [*state, 0.0], t_eval=t_eval)
        return sol.y.T

    def get_reference(self, cstate: CurrentState, dt=0.1):
        """get a reference point for the decision module"""
        field = self.get_force_field(cstate)
        state = cstate.cpos
        ref_state = self.get_trajectory(cstate, [0.0, dt])[-1]
        ref_speed = np.linalg.norm(field([[*state, 0.0]])[0][:2])
        print(ref_speed)
        return (*ref_state[:2], ref_speed)


class PotentialDecision:
    @staticmethod
    def get_xyz(point):
        return (point['x'], point['y'], point['z'])

    @staticmethod
    def get_lanes_locations(record):
        c = np.array([PotentialDecision.get_xyz(t) for t in record["lane_marker"]["lane_markers_center"]["location"]])
        l = np.array([PotentialDecision.get_xyz(t) for t in record["lane_marker"]["lane_markers_left"]["location"]])
        r = np.array([PotentialDecision.get_xyz(t) for t in record["lane_marker"]["lane_markers_right"]["location"]])
        return l, c, r

    @staticmethod
    def get_obstacles_verts(crecord):
        obstacles = crecord['obstacles']
        verts = []
        for obstacle in obstacles:
            pt = [(t['vertex_location']['x'], t['vertex_location']['y'], t['vertex_location']['z']) for t in
                  obstacle['vertices_locations']]
            verts.append(np.array(pt))
        return verts

    @staticmethod
    def _rosmsg2dict(msg):
        return yaml.safe_load(str(msg))

    def __init__(self):
        self.potential_field = PotentialField(lane_penalty=0.5, obstacle_penalty=3.0, goal_reward=15.0)

    def get_ref_state(self, curr_state, obstacle_list, lane_marker, waypoint):
        record = {
            "state": curr_state,
            "lane_marker": self._rosmsg2dict(lane_marker),
            "waypoint": self._rosmsg2dict(waypoint),
            "obstacles": [self._rosmsg2dict(obi) for obi in obstacle_list],
        }

        lan_locs = self.get_lanes_locations(record)
        r = RoadSegment(*lan_locs[::-1])
        state = record["state"][0]
        velocity = record["state"][-1]
        cs = CurrentState(
            r,
            state,
            velocity,
            np.array(self.get_xyz(record['waypoint']['location'])),
            [Obstacle(v) for v in self.get_obstacles_verts(record)]
        )
        ref = self.potential_field.get_reference(cs)
        print(ref)
        return ref


class VehicleController():
    def stop(self):
        newAckermannCmd = AckermannDrive()
        newAckermannCmd.acceleration = -20
        newAckermannCmd.speed = 0
        newAckermannCmd.steering_angle = 0
        return newAckermannCmd

    def execute(self, currentPose, targetPose):
        """
            This function takes the current state of the vehicle and
            the target state to compute low-level control input to the vehicle
            Inputs:
                currentPose: ModelState, the current state of vehicle
                targetPose: The desired state of the vehicle
        """

        currentEuler = currentPose[1]
        curr_x = currentPose[0][0]
        curr_y = currentPose[0][1]

        target_x = targetPose[0]
        target_y = targetPose[1]
        target_v = targetPose[2]

        k_s = 0.1
        k_ds = 1
        k_n = 0.1
        k_theta = 1

        # compute errors
        dx = target_x - curr_x
        dy = target_y - curr_y
        xError = (target_x - curr_x) * np.cos(
            currentEuler[2]) + (target_y - curr_y) * np.sin(currentEuler[2])
        yError = -(target_x - curr_x) * np.sin(
            currentEuler[2]) + (target_y - curr_y) * np.cos(currentEuler[2])
        curr_v = np.sqrt(currentPose[2][0] ** 2 + currentPose[2][1] ** 2)
        vError = target_v - curr_v

        delta = k_n * yError
        # Checking if the vehicle need to stop
        if target_v > 0:
            v = xError * k_s + vError * k_ds
            # Send computed control input to vehicle
            newAckermannCmd = AckermannDrive()
            newAckermannCmd.speed = v
            newAckermannCmd.steering_angle = delta
            return newAckermannCmd
        else:
            return self.stop()


class Controller(object):
    """docstring for Controller"""

    def __init__(self):
        super(Controller, self).__init__()
        self.decisionModule = PotentialDecision()
        self.controlModule = VehicleController()

    def stop(self):
        return self.controlModule.stop()

    def execute(self, currState, obstacleList, lane_marker, waypoint):
        # Get the target state from decision module
        refState = self.decisionModule.get_ref_state(currState, obstacleList,
                                                     lane_marker, waypoint)
        if not refState:
            return None
        return self.controlModule.execute(currState, refState)
