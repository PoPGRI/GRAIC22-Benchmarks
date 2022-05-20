import numpy as np
import yaml
from joblib import Parallel, delayed

from graic22dc.src.planner.common import CurrentState
from graic22dc.src.planner.curvature import curvature_splines, smooth_spline_path
from graic22dc.src.planner.predictor import ObstaclePredictor
from graic22dc.src.planner.rrt.rrt import RRTNode
from graic22dc.src.planner.rrt.rrt_star import RRTStar


class RRTStarPath:
    @staticmethod
    def path_length(pts):
        if pts is None or len(pts) <= 1:
            return 1e8
        lengths = np.sqrt(np.sum(np.diff(pts, axis=0) ** 2, axis=1))  # Length between corners
        total_length = np.sum(lengths)
        return total_length

    def __init__(self,
                 path_resolution=1.0,
                 connect_circle_dist=20.0,
                 max_iter=300,
                 expand_dis=3.0,
                 goal_sample_rate=80.0,
                 ):
        self.path_resolution = path_resolution
        self.connect_circle_dist = connect_circle_dist
        self.max_iter = max_iter
        self.expand_dis = expand_dis
        self.goal_sample_rate = goal_sample_rate
        self.smooth = False

    def run_rrt_star(self, start, bounds, obstacles_list, cgoal):
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
        return np.array(path) if path is not None else path

    def is_path_valid(self, start, bounds, obstacles_list, cgoal, path):
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
        # for every point in path, determine if it collides with an obstacle in the list
        # if the path is okay
        for pt in path:
            node = RRTNode(pt[0], pt[1])
            node.path_x = [node.x]
            node.path_y = [node.y]
            if not rrt_star.check_collision(node, rrt_star.obstacle_list):
                return False
        return True

    def get_path(self, start, bounds, leftb, rightb, obstacles_list, subdivisions):
        # look at alternative goals if obstacle is present
        lgoals = np.linspace(leftb[-1][:], rightb[-1][:], subdivisions)[1:-1, :]
        paths = Parallel(n_jobs=subdivisions)(delayed(self.run_rrt_star)(start, bounds, obstacles_list, cgoal) for cgoal in lgoals)
        if self.smooth:
            paths = [smooth_spline_path(p[:, 0], p[:, 1]) for p in paths if p is not None]
        else:
            paths = [p for p in paths if p is not None]

        # return the best path
        if len(paths) == 0:
            fpath = None, 0
        else:
            path_lengths = [self.path_length(p) for p in paths]
            fpath = paths[np.argmin(path_lengths)], len(paths)
        return fpath


class RRTStarDecision:
    @staticmethod
    def _rosmsg2dict(msg):
        return yaml.safe_load(str(msg))

    def __init__(
        self,
        v_max=40.0,
        v_min=20.0,
        path_resolution=1.0,
        connect_circle_dist=20.0,
        max_iter=300,
        expand_dis=3.0,
        goal_sample_rate=80.0,
        predictor_steps=5,
        predictor_delta=10,
        vehicle_radius=2.5,
        person_radius=2.5,
        lanes_radius=1.0,
        lookahead_mult=3.0,
        road_subdivisions=7,
    ):
        """
        :param v_max: maximum speed to shoot for if road is clear
        :param v_min: the minimum speed to shoot for if being cautious
        :param path_resolution: RRT* path resolution
        :param connect_circle_dist: RRT* circle connect dist
        :param max_iter: RRT* max number of iterations
        :param predictor_steps: when predicting ahead, how many delta advances (total steps = steps * delta)
        :param predictor_delta: when predicting ahead, how many time steps to go before providing a prediction
        :param vehicle_radius: amount of bloat around vehicle
        :param person_radius: amount of blot around person
        :param lanes_radius: amount of bloat around lanes boundary
        :param lookahead_mult: how many times ahead to predict road boundaries (1 uses just the data provided)
        :param road_subdivisions: how many progress goals to consider for RRT* planning
        """
        self.last_state = None
        self.v_max = v_max
        self.v_min = v_min
        self.path_resolution = path_resolution
        self.connect_circle_dist = connect_circle_dist
        self.max_iter = max_iter
        self.expand_dis = expand_dis
        self.predictor_steps = predictor_steps
        self.predictor_delta = predictor_delta
        self.vehicle_radius = vehicle_radius
        self.person_radius = person_radius
        self.lanes_radius = lanes_radius
        self.lookahead_mult = lookahead_mult
        self.road_subdivisions = road_subdivisions
        self.goal_sample_rate = goal_sample_rate
        self.predictor = ObstaclePredictor()
        self.last_path = None
        self.elevation = 0.0
        self.reuse = 0
        self.do_reuse = False
        self.reuse_limit = 10
        self.ped_stop_count = 0
        self.ped_stop_limit = 40
        self.ped_stop_state = False
        self.rrts_fast_path = RRTStarPath(path_resolution=0.4, connect_circle_dist=0.4,
                                     max_iter=300, expand_dis=2.0, goal_sample_rate=100.0)
        self.rrts_path = RRTStarPath(path_resolution=path_resolution, connect_circle_dist=connect_circle_dist,
                                          max_iter=max_iter, expand_dis=expand_dis, goal_sample_rate=goal_sample_rate)

    def get_path(self, cs: CurrentState):
        # get road boundaries
        r = cs.road  # .get_segment(-0.5, 3.0)
        leftb, rightb, cstatus = r.get_bounds_segment(0.25, self.lookahead_mult, cs.waypoint)
        if not cstatus:
            rightb, leftb = cs.road.bounds
            rightb = rightb[:, :2]
            leftb = leftb[:, :2]
        centerb = (leftb + rightb) / 2.0

        # include the offroad scenario
        #dist = np.median(np.linalg.norm(leftb - rightb, axis=1) / 2.0)
        #mdist = np.linalg.norm(centerb - cs.cpos, axis=1)
        if True:#np.any(mdist < dist):
            # update lane list for rrt obstacles
            lanes_list = np.zeros((len(rightb) * 2, 3))
            lanes_list[:, :2] = np.vstack((leftb, rightb))
            lanes_list[:, 2] = self.lanes_radius
        else:
            lanes_list = None
            print(f"OFFROAD: my dist is {mdist} and the boundary dist if {dist}")

        n_obstacles = len(cs.obstacles)
        # update obstacle list with on road obstacles
        self.predictor.submit_obstacles(cs.obstacles)
        if n_obstacles > 0:
            obstacles_verts = self.predictor.get_verts(
                cs.cpos, self.predictor_steps, self.predictor_delta, self.vehicle_radius, self.person_radius
            )
            if lanes_list is not None:
                if len(obstacles_verts) > 0:
                    obstacles_list = np.vstack((lanes_list, obstacles_verts))
                else:
                    obstacles_list = lanes_list
            else:
                obstacles_list = obstacles_verts
        else:
            if lanes_list is not None:
                obstacles_list = lanes_list
            else:
                obstacles_list = []

        # get waypoint
        heading = np.arctan2(cs.cvelocity[1], cs.cvelocity[0])
        start = [*cs.cpos, heading]

        # get the goal
        center_dir = np.diff(centerb[-2:][:, :2], axis=0).flatten()
        wheading = np.arctan2(center_dir[1], center_dir[0])
        nwaypoint = centerb[-1, :2]
        goal = [*nwaypoint, wheading]

        # all points to do bounds check
        if obstacles_list is not None and lanes_list is not None:
            all_pts = np.vstack((lanes_list[:, :-1], np.atleast_2d(start[:-1]), np.atleast_2d(goal[:-1])))
        else:
            all_pts = np.vstack((np.atleast_2d(start[:-1]), np.atleast_2d(goal[:-1])))

        xbounds = (all_pts[:, 0].min(), all_pts[:, 0].max())
        ybounds = (all_pts[:, 1].min(), all_pts[:, 1].max())
        bounds = xbounds, ybounds

        #plt.plot(*leftb[:, :2].T)
        #plt.plot(*rightb[:, :2].T)
        #plt.plot(*centerb[:, :2].T)
        #plt.scatter(*all_pts[:, :2].T)
        #plt.show()

        self.elevation = np.max(cs.road.center_lane[:, 2])

        if self.do_reuse and self.last_path is not None and self.last_path[0] is not None and self.rrts_fast_path.is_path_valid(start,
                                                                                                              bounds,
                                                                                                              obstacles_list,
                                                                                                              goal,
                                                                                                              self.last_path[
                                                                                                                  0]) and self.reuse < self.reuse_limit:
            self.reuse += 1
            return self.last_path
        else:
            print("generating new path")
            self.reuse = 0
            path, npaths = self.rrts_fast_path.get_path(start, bounds, leftb, rightb, obstacles_list, self.road_subdivisions)
            if path is None:
                print("backup path...")
                path, npaths = self.rrts_path.get_path(start, bounds, leftb, rightb, obstacles_list, self.road_subdivisions)
                if path is not None:
                    path = smooth_spline_path(path[:, 0], path[:, 1])
            self.last_path = path, npaths, cstatus
            return path, npaths, cstatus

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
            vmax = self.v_min
            vmin = self.v_min

        if num_paths <= 2:
            print("[WARNING] Few Paths")
            ref_speed = self.v_min
        else:
            try:
                curvature = curvature_splines(*path[:, :2].T)
                mc = np.max(curvature)
                # this is a weird parameter...
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
        pos = np.array(cs.cpos)
        path, road_reach, cstatus = self.get_path(cs)

        any_pedestrians = False
        for ob in cs.obstacles:
            if "vehicle" not in ob.name:
                any_pedestrians = True

        if not any_pedestrians:
            self.ped_stop_state = False
            self.ped_stop_count = 0
        else:
            self.ped_stop_state = True
            self.ped_stop_count += 1


        if path is not None:
            path = np.array(path)
            if self.ped_stop_state and self.ped_stop_count < self.ped_stop_limit:
                print(f"stopping for pedestrian {self.ped_stop_count} / {self.ped_stop_limit}")
                ref_speed = 0.0
            else:
                ref_speed = self.speed_mod(path, road_reach, len(obstacle_list))
            #goal_idx = max(int((len(path) // ((1.0 if not cstatus else self.lookahead_mult) * 1.5))), 1 if len(path) == 1 else 2)
            distidx = np.abs(np.linalg.norm(path - pos, axis=1) - 5.0).argmin()
            goal_idx = distidx if distidx != len(path) - 1 else -2
            if len(path) >= goal_idx:
                ref = (*path[goal_idx], ref_speed)
            else:
                ref = (*path[0], ref_speed)
            self.last_state = ref[:2]  # (path[-2] + path[-3] + path[-4]) / 3.0
        else:
            print("no path found!")
            if self.last_state is not None:
                # non zero because it crashes the simulation?
                ref = (*self.last_state, -5.0)
            else:
                ref = (*cs.cpos, -5.0)
        return ref


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
