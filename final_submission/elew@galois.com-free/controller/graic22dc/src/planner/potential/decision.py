import numpy as np
from graicctl.common import CurrentState, RoadSegment, Obstacle
from typing import Sequence, Callable
from scipy.integrate import solve_ivp
import yaml


class PotentialField:
    def __init__(self, lane_penalty=5.0, obstacle_penalty=60.0, goal_reward=30.0):
        self.lane_penalty = lane_penalty
        self.obstacle_penalty = obstacle_penalty
        self.goal_reward = goal_reward

    @staticmethod
    def compute_forces_obstacle(pt, obstacles, penalty):
        """forces produced from obstacles"""
        # dirs_l = (pt - obstacles)
        dirs_l = np.tile(pt, (len(obstacles), 1)) - obstacles
        dists_l = np.linalg.norm(dirs_l, axis=1)
        dirsn_l = dirs_l / dists_l[:, np.newaxis]
        mags_l = np.clip(penalty / (dists_l[:, np.newaxis] ** 2), 0.0, 10.0)
        forces_l = dirsn_l * mags_l
        return forces_l

    @staticmethod
    def compute_forces(pt, obstacles, penalty):
        """forces produced from obstacles"""
        dirs_l = np.tile(pt, (len(obstacles), 1)) - obstacles
        dists_l = np.linalg.norm(dirs_l, axis=1)
        dirsn_l = dirs_l / dists_l[:, np.newaxis]
        mags_l = np.clip(penalty / (dists_l[:, np.newaxis] ** 2), 0.0, 10.0)
        forces_l = dirsn_l * mags_l
        return forces_l

    @staticmethod
    def compute_forces_goal(pt, obstacles, penalty):
        """a force that doesn't attenuate (global goal)"""
        dirs_l = np.tile(pt, (len(obstacles), 1)) - obstacles
        # dirs_l = (pt - obstacles)
        dists_l = np.linalg.norm(dirs_l, axis=1)
        dirsn_l = dirs_l / dists_l[:, np.newaxis]
        forces_l = penalty * dirsn_l  # / (dists_l[:, np.newaxis]**2)
        return forces_l

    def get_force_field(self, cstate: CurrentState) -> Callable[[np.ndarray], np.ndarray]:
        """get function that will produce net force for given points"""
        road, waypoint, obstacles = cstate.road, cstate.waypoint, cstate.obstacles
        r = cstate.road.get_segment(-0.5, 3.0)
        if len(cstate.obstacles) > 0:
            obstacles_verts = np.vstack([oi.verts for oi in cstate.obstacles])
            obstacles_verts[:, 2] = 2.0

        center_dir = np.diff(r.center_lane[-2:][:, :-1], axis=0).flatten()
        wheading = np.arctan2(center_dir[1], center_dir[0])
        nwaypoint = (r.bounds[0][-1][:-1] + r.bounds[1][-1][:-1]) / 2
        goal = [*nwaypoint, wheading]

        lgoals = np.linspace(r.bounds[0][-1][:-1], r.bounds[1][-1][:-1], 7)[1:-1, :]
        if len(cstate.obstacles) > 0:
            mdists = np.array(
                [
                    np.linalg.norm(obstacles_verts[:, :-1] - np.tile(pt, (len(obstacles_verts), 1)), axis=1).min()
                    for pt in lgoals
                ]
            )
            print(mdists.argmax())
            goal = [*lgoals[mdists.argmax()], goal[2]]

        def force(points):
            points = np.array(points)
            forces_ret = np.zeros(points.shape)

            # extrapolate road backwards and forwards
            left_lane, right_lane = r.bounds

            # get center of obstacles for the repellers
            centers = [obsv.verts for obsv in obstacles]
            obstacle_verts = np.vstack(centers) if len(centers) > 0 else []

            waypoint = [*goal[:2], 0.0]

            for idx, pts in enumerate(points):
                # lane marker forces
                forces_lanes = self.compute_forces(pts, np.vstack((left_lane, right_lane)), self.lane_penalty)

                # waypoint force
                forces_goals = -self.compute_forces_goal(pts, np.atleast_2d(waypoint), self.goal_reward)

                # obstacle forces
                if len(obstacle_verts) > 0:
                    forces_obstacles = self.compute_forces_obstacle(pts, obstacle_verts, self.obstacle_penalty)
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
        return (point["x"], point["y"], point["z"])

    @staticmethod
    def get_lanes_locations(record):
        c = np.array([PotentialDecision.get_xyz(t) for t in record["lane_marker"]["lane_markers_center"]["location"]])
        l = np.array([PotentialDecision.get_xyz(t) for t in record["lane_marker"]["lane_markers_left"]["location"]])
        r = np.array([PotentialDecision.get_xyz(t) for t in record["lane_marker"]["lane_markers_right"]["location"]])
        return l, c, r

    @staticmethod
    def get_obstacles_verts(crecord):
        obstacles = crecord["obstacles"]
        verts = []
        for obstacle in obstacles:
            pt = [
                (t["vertex_location"]["x"], t["vertex_location"]["y"], t["vertex_location"]["z"])
                for t in obstacle["vertices_locations"]
            ]
            verts.append(np.array(pt))
        return verts

    @staticmethod
    def _rosmsg2dict(msg):
        return yaml.safe_load(str(msg))

    def __init__(self):
        # self.potential_field = PotentialField(lane_penalty=3.0, obstacle_penalty=20.0, goal_reward=30.0)
        self.potential_field = PotentialField(lane_penalty=30.0, obstacle_penalty=100.0, goal_reward=45.0)

    def get_ref_state(self, curr_state, obstacle_list, lane_marker, waypoint):
        record = {
            "state": curr_state,
            "lane_marker": self._rosmsg2dict(lane_marker),
            "waypoint": self._rosmsg2dict(waypoint),
            "obstacles": [self._rosmsg2dict(obi) for obi in obstacle_list],
        }

        cs = CurrentState.from_dict(record)
        # lan_locs = self.get_lanes_locations(record)
        # r = RoadSegment(*lan_locs, record["lane_marker"]["lane_state"])
        # state = record["state"][0]
        # velocity = record["state"][-1]

        r = cs.road
        state = cs.cpos
        velocity = cs.cvelocity

        cs = CurrentState(
            r,
            state,
            velocity,
            np.array(self.get_xyz(record["waypoint"]["location"])),
            [Obstacle(v) for v in self.get_obstacles_verts(record)],
        )

        ref = self.potential_field.get_reference(cs)
        print(ref)
        return ref
