import sys
sys.path.append("..")

from graicctl.rrt.decision import RRTStarDecision
from graicctl.baseline import BaselineController


class Controller(BaselineController):
    def __init__(self):
        super(Controller, self).__init__(RRTStarDecision())
