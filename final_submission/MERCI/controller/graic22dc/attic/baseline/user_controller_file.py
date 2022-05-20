import sys
sys.path.append("..")

from graicctl.baseline import BaselineController, BaselineVehicleDecision


class Controller(BaselineController):
    def __init__(self):
        super(Controller, self).__init__(BaselineVehicleDecision())
