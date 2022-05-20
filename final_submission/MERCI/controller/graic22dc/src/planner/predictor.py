from sklearn.linear_model import Ridge
from sklearn.preprocessing import PolynomialFeatures
from sklearn.pipeline import make_pipeline
from graic22dc.src.planner.common import Obstacle
import numpy as np


class ObstaclePredictor:
    def __init__(self):
        # current time obstacles with snapshot history
        self.cobstacles = {}

    def submit_obstacles(self, obstacles):
        nobstacles = {}
        for obstacle in obstacles:
            # obstacle already exists
            if obstacle.obstacle_id in self.cobstacles:
                self.cobstacles[obstacle.obstacle_id].append(obstacle)
            else:
                # obstacle is new
                self.cobstacles[obstacle.obstacle_id] = [obstacle]
            nobstacles[obstacle.obstacle_id] = self.cobstacles[obstacle.obstacle_id]

        # delete obstacles that are not present
        #nobstacles = set([ob.obstacle_id for ob in obstacles])
        #bad_keys = set(self.cobstacles.keys()) - nobstacles
        #for bk in bad_keys:
        #    print(f"delete {bk} {self.cobstacles[bk][-1].name}")
        #    del self.cobstacles[bk]
        self.cobstacles = nobstacles

    def get_verts(self, pos, steps=5, delta=10, vehicle_radius=2.5, person_radius=5.0):
        # get vertices of obstacles
        def set_radii(verts, name):
            if "vehicle" in name:
                verts[:, 2] = vehicle_radius
            else:
                verts[:, 2] = person_radius
            return verts

        # create vertices with the obstacle radius info
        if len(self.cobstacles) > 0:
            vobs = []
            for obstacle_name, obstacle_snaps in self.cobstacles.items():
                cvs = self.make_prediction(obstacle_name, steps=steps, delta=delta)
                mdist = np.linalg.norm(np.array([c.location[:2] for c in cvs]) - pos, axis=1)
                radius = vehicle_radius if "vehicle" in cvs[0].name else person_radius
                if np.min(mdist) < radius:
                    print("already collided in obstacle, so pretend it's not there")
                else:
                    vobs += cvs
            if len(vobs) > 0:
                obstacles_verts = np.vstack([set_radii(np.vstack((obs.verts, [obs.location])), obs.name) for obs in vobs])
                return obstacles_verts
            else:
                return []
        else:
            return []

    def make_prediction(self, name, steps=5, delta=10):
        """make prediction for an obstacle with id name"""
        if name in self.cobstacles:
            snaps = self.cobstacles[name]
            # only one snapshot -- not enough to regress
            if len(snaps) == 1:
                ob = snaps[0]
                return [Obstacle(ob.name, ob.obstacle_id, ob.location, ob.verts) for _ in range(1)]
            else:
                # extrapolate the motion
                locs = np.array([ob.location for ob in snaps])[-2:]
                X = np.arange(0, len(locs), 1)[:, np.newaxis]
                model = make_pipeline(PolynomialFeatures(degree=1), Ridge(alpha=1e-3))
                model.fit(X, locs)
                name = snaps[0].name
                if "vehicle" in name:
                    Xn = np.arange(len(locs) - 1, len(locs) - 1 + steps * delta, delta)[:, np.newaxis]
                else:
                    Xn = np.arange(len(locs) - 1, len(locs) - 1 + steps * delta, delta)[:, np.newaxis]
                    #Xn = np.array([*np.arange(-steps * delta, 0.0, delta), *np.arange(len(locs) - 1, len(locs) - 1 + steps * delta, delta)])[:, np.newaxis]
                Yn = model.predict(Xn)
                ob = snaps[0]
                return [Obstacle(ob.name, ob.obstacle_id, loc, ob.verts - ob.location + loc) for loc in Yn]
        else:
            raise ValueError(f"name isn't in obstacle list {self.cobstacles.keys()}")
