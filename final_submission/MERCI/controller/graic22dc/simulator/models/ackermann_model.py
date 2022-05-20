from email.policy import default
import numpy as np


class AP:
    """
    Ackermann Params
        Fixed vehicle specific parameters
    """

    @classmethod
    def from_default(cls):
        """
        Model parameters from “Automotive Control Systems” by A. Galip Ulsoy et al.
        """
        m = 1800
        f1, f2, f3 = 4.59, 0.01, 74.63  # f1 = 40.59
        L = 2.54
        a = 1.14
        b = L - a
        Caf, Car = 4400 * 2, 4700 * 2
        Iz = 2420
        return cls(m=m, f1=f1, f2=f2, f3=f3, a=a, b=b, Caf=Caf, Car=Car, Iz=Iz)

    @classmethod
    def from_GRAIC22_latest(cls):
        """
        Model Parameters from https://github.com/PoPGRI/Race/blob/main/raceDoc22.md


        m 	    f1 	    f2      f3 	    a 	    b 	    Caf 	Car 	Iz 	    Vehicle ID
        1500	100	    0.1	    0.01	1.14	1.4	    8.8e4	9.4e4	2420	vehicle.model_based.1
        """
        m = 1500
        f1, f2, f3 = 100, 0.1, 0.01
        a, b = 1.14, 1.4
        Caf, Car = 8.8e4, 9.4e4
        Iz = 2420
        return cls(m=m, f1=f1, f2=f2, f3=f3, a=a, b=b, Caf=Caf, Car=Car, Iz=Iz)

    @classmethod
    def from_GRAIC22(cls):
        """
        Model Parameters from https://github.com/PoPGRI/Race/blob/main/raceDoc22.md

        m 	    f1 	    f2      f3 	    a 	    b 	    Caf 	Car 	Iz 	    Vehicle ID
        1800    40.59   0.01    74.63 	1.2 	1.65 	1.4e5 	1.2e5 	3270 	vehicle.model_based.1
        """
        m = 1800
        f1, f2, f3 = 40.59, 0.01, 74.63
        a, b = 1.2, 1.65
        Caf, Car = 1.4e4, 1.2e4
        Iz = 3270
        return cls(m=m, f1=f1, f2=f2, f3=f3, a=a, b=b, Caf=Caf, Car=Car, Iz=Iz)

    @classmethod
    def from_GRAIC22_alt(cls):
        """
        Minimal modifications of GRAIC22 params in order to make it work
        Only f1 is modified.
        """
        m = 1800
        f1, f2, f3 = 4.59, 0.01, 74.63
        a, b = 1.2, 1.65
        Caf, Car = 1.4e4, 1.2e4
        Iz = 3270
        return cls(m=m, f1=f1, f2=f2, f3=f3, a=a, b=b, Caf=Caf, Car=Car, Iz=Iz)

    def __init__(self, *, m, f1, f2, f3, a, b, Caf, Car, Iz) -> None:
        self.m = m
        self.f1 = f1
        self.f2 = f2
        self.f3 = f3
        self.a = a
        self.b = b
        self.L = a + b
        self.Caf = Caf
        self.Car = Car
        self.Iz = Iz


class SID:
    """
    State indices
    """

    v_long = 0
    y = 1
    v_lat = 2
    psi = 3
    r = 4
    len = 5


class UID:
    """
    Control indices
    """

    a_long = 0  # Longitudinal acceleration (m/s^2)
    steering_angle = 1  # Steering angle (radians)


# Global object for convenience
__ap = AP.from_default()
# __ap = AP.from_GRAIC22()
# __ap = AP.from_GRAIC22_alt()
# __ap = AP.from_GRAIC22_latest()


def dynamics(X, U):
    ap = __ap
    # Xlong = <>
    Ua_long = U[UID.a_long]
    v_long = X[SID.v_long]
    v_long_dot = 0  # Ua_long - ap.f1 * v_long - ap.f2 * (v_long**2) - ap.f3

    # Xlat is the vector <y, v_lat, psi, r>
    Xlat = np.array((X[SID.y], X[SID.v_lat], X[SID.psi], X[SID.r]))
    Alat = np.array(
        [
            [0, 1, v_long, 0],
            [
                0,
                -(ap.Caf + ap.Car) / (ap.m * v_long),
                0,
                (ap.b * ap.Car - ap.a * ap.Caf) / (ap.m * v_long) - v_long,
            ],
            [0, 0, 0, 1],
            [
                0,
                (ap.b * ap.Car - ap.a * ap.Caf) / (ap.Iz * v_long),
                0,
                -(ap.a**2 * ap.Caf + ap.b**2 * ap.Car) / (ap.Iz * v_long),
            ],
        ]
    )

    Blat = ap.Caf * np.array([0, 1 / ap.m, 0, ap.a / ap.Iz])
    Xlat_dot = list(Alat @ Xlat + Blat * U[UID.steering_angle])
    xdot = np.asarray([v_long_dot] + Xlat_dot)

    return xdot


def xy_position(v_long, v_lat, theta):
    xdot = v_long * np.cos(theta) - v_lat * np.sin(theta)
    ydot = v_long * np.sin(theta) + v_lat * np.cos(theta)
    # delta_dot = delta_dot
    # theta_dot = v_long * np.tan(delta) / __ap.L
    return np.array([xdot, ydot])  # , theta_dot])
