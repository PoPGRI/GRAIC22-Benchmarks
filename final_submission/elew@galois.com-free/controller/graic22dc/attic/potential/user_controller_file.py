import sys
sys.path.append("..")

from graicctl.potential.decision import PotentialDecision
from graicctl.baseline import BaselineController


class Controller(BaselineController):
    def __init__(self):
        super(Controller, self).__init__(PotentialDecision())
