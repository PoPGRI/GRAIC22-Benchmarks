import yaml
import json

from graic22dc.src.planner.baseline import BaselineVehicleDecision


class VehicleRecorder:
    """record decision maker's inputs and outputs

    NOTE: we could just have used rosbags instead
    """

    @staticmethod
    def _rosmsg2dict(msg):
        return yaml.safe_load(str(msg))

    def __init__(self, decision_maker: BaselineVehicleDecision, fname: str):
        self._fname = fname
        self._decision_maker = decision_maker
        with open(fname, "w") as fp:
            fp.write("")
        # self._records = []
        self._nmod = 100
        self._idx = 0

    def get_ref_state(self, currState, obstacleList, lane_marker, waypoint):
        ref = self._decision_maker.get_ref_state(currState, obstacleList, lane_marker, waypoint)
        record = {
            "state": currState,
            "lane_marker": self._rosmsg2dict(lane_marker),
            "waypoint": self._rosmsg2dict(waypoint),
            "obstacles": [self._rosmsg2dict(obi) for obi in obstacleList],
            "decision": ref,
        }
        with open(self._fname, "a") as fp:
            s = json.dumps(record)
            fp.write(f"{s},")
        self._idx += 1
        # record input and output
        return ref
