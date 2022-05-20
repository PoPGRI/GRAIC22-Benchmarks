import yaml
import numpy as np
from graicctl.common import CurrentState
from graicctl.rrt.rrt_star import RRTStar
from graicctl.rrt.rrt import RRTNode
from joblib import Parallel, delayed
from graicctl.curvature import curvature_splines


class RRTStarDecision:
    @staticmethod
    def _rosmsg2dict(msg):
        return yaml.safe_load(str(msg))

    def __init__(self,
                 v_max = 40.0,
            path_resolution = 1.0,
            connect_circle_dist = 20.0,
            max_iter = 300,
            expand_dis = 3.0,
            goal_sample_rate=80.0):
        self.last_state = None
        #self.ref_speed = 30.0
        self.v_max = v_max
        self.v_min = 20.0
        self.path_resolution = path_resolution
        self.connect_circle_dist = connect_circle_dist
        self.max_iter = max_iter
        self.expand_dis = expand_dis
        self.goal_sample_rate = goal_sample_rate

    @staticmethod
    def path_length(pts):
        if pts is None or len(pts) <= 1:
            return 1E8
        lengths = np.sqrt(np.sum(np.diff(pts, axis=0) ** 2, axis=1))  # Length between corners
        total_length = np.sum(lengths)
        return total_length

    def get_path(self, cs: CurrentState):
        def _get_path(cgoal):
            class _Node:
                def __init__(self, x, y):
                    self.x = x
                    self.y = y

            rrt_star = RRTStar(
                start=start[:-1],
                goal=cgoal,
                rand_area=bounds,
                obstacle_list=obstacles_list if obstacles_list is not None else [],
                path_resolution=self.path_resolution,
                connect_circle_dist=self.connect_circle_dist,
                max_iter=self.max_iter,
                expand_dis=self.expand_dis,
                search_until_max_iter=False,
                goal_sample_rate=self.goal_sample_rate,
                # collision_func=r.collision_check_rrt
            )
            node = RRTNode(cgoal[0], cgoal[1])
            node.path_x = [node.x]
            node.path_y = [node.y]
            if rrt_star.check_collision(node, rrt_star.obstacle_list):
                path = rrt_star.planning(animation=False)
            else:
                path = None
            return path

        # get road boundaries
        r = cs.road#.get_segment(-0.5, 3.0)
        leftb, rightb = r.get_bounds_segment(-0.5, 3.0, cs.waypoint)
        centerb = (leftb + rightb) / 2.0

        # update lane list for rrt obstacles
        lanes_list = np.zeros((len(rightb)*2, 3))
        lanes_list[:, :2] = np.vstack((leftb, rightb))
        lanes_list[:, 2] = 1.0

        # update obstacle list with on road obstacles
        if len(cs.obstacles) > 0:
            obstacles_verts = np.vstack([oi.verts for oi in cs.obstacles])
            obstacles_verts[:, 2] = 2.5
            obstacles_list = np.vstack((lanes_list, obstacles_verts))
        else:
            obstacles_list = lanes_list

        # get waypoint
        heading = np.arctan2(cs.cvelocity[1], cs.cvelocity[0])
        start = [*cs.cpos, heading]

        center_dir = np.diff(centerb[-2:][:, :2], axis=0).flatten()
        wheading = np.arctan2(center_dir[1], center_dir[0])
        nwaypoint = (centerb[-1, :2])
        goal = [*nwaypoint, wheading]

        if obstacles_list is not None:
            all_pts = np.vstack((lanes_list[:, :-1], np.atleast_2d(start[:-1]), np.atleast_2d(goal[:-1])))
        else:
            all_pts = np.vstack((np.atleast_2d(start[:-1]), np.atleast_2d(goal[:-1])))

        xbounds = (all_pts[:, 0].min(), all_pts[:, 0].max())
        ybounds = (all_pts[:, 1].min(), all_pts[:, 1].max())
        bounds = xbounds, ybounds

        # look at alternative goals if obstacle is present
        if len(cs.obstacles) > 0:
            lgoals = np.linspace(leftb[-1][:], rightb[-1][:], 7)[1:-1, :]
            paths = Parallel(n_jobs=7)(delayed(_get_path)(cgoal) for cgoal in lgoals)
            paths = [p for p in paths if p is not None]
        else:
            paths = [_get_path(goal)]

        # return the best path
        if len(paths) == 0:
            return None, 0
        else:
            path_lengths = [self.path_length(p) for p in paths]
            return paths[np.argmin(path_lengths)], len(paths) if len(cs.obstacles) > 0 else 7

    def speed_mod(self, path, num_paths, num_obstacles):
        """this modulates the speed of the RRT star decision module"""
        path = np.array(path)

        # 2 or less obstacles warrants a "reckless mode"
        # more than 2 obstacles warrants a "cautious mode" (lower maximum speed)
        if num_obstacles <= 2:
            vmax = self.v_max
            vmin = self.v_min
        else:
            print("[WARNING] Cautious Mode")
            vmax = 30.0
            vmin = self.v_min

        if num_paths <= 2:
            print("[WARNING] Few Paths")
            ref_speed = 10.0
        else:
            try:
                curvature = curvature_splines(*path[:, :2].T)
                mc = np.max(curvature)
                print(mc)
                ref_speed = (vmax - vmin) * (1 - np.arctan(10 * mc)) + vmin
            except:
                ref_speed = vmin
        return ref_speed


    def get_ref_state(self, curr_state, obstacle_list, lane_marker, waypoint):
        record = {
            "state": curr_state,
            "lane_marker": self._rosmsg2dict(lane_marker),
            "waypoint": self._rosmsg2dict(waypoint),
            "obstacles": [self._rosmsg2dict(obi) for obi in obstacle_list],
        }

        cs = CurrentState.from_dict(record)

        path, road_reach = self.get_path(cs)

        if path is not None:
            ref_speed = self.speed_mod(path, road_reach, len(obstacle_list))
            if len(path) >= 3:
                ref = (*path[-3], ref_speed) #(*((path[-2] + path[-3] + path[-4]) / 3.0), self.ref_speed)
            else:
                ref = (*path[-1], ref_speed)
            self.last_state = ref[:2] #(path[-2] + path[-3] + path[-4]) / 3.0
        else:
            print("no path found!")
            if self.last_state is not None:
                # non zero because it crashes the simulation?
                ref = (*self.last_state, 0.0)
            else:
                ref = (*cs.cpos, 0.0)
        return ref


from graicctl.baseline import BaselineVehicleDecision


class MixedDecision(RRTStarDecision):
    def __init__(self):
        self.rrt = RRTStarDecision()
        self.baseline = RRTStarDecision(v_max=30.0)

    def get_ref_state(self, curr_state, obstacle_list, lane_marker, waypoint):
        if len(obstacle_list) > 2:
            print("recovery")
            return self.baseline.get_ref_state(curr_state, obstacle_list, lane_marker, waypoint)
        else:
            return self.rrt.get_ref_state(curr_state, obstacle_list, lane_marker, waypoint)
