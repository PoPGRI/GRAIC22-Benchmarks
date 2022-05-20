import numpy as np
from scipy.interpolate import UnivariateSpline


def curvature_splines(x, y, error=0.1):
    """Calculate the signed curvature of a 2D curve at each point
    using interpolating splines.
    Parameters
    """

    t = np.arange(x.shape[0])
    std = error * np.ones_like(x)

    fx = UnivariateSpline(t, x, k=4, w=1 / np.sqrt(std))
    fy = UnivariateSpline(t, y, k=4, w=1 / np.sqrt(std))

    xp = fx.derivative(1)(t)
    xpp = fx.derivative(2)(t)
    yp = fy.derivative(1)(t)
    ypp = fy.derivative(2)(t)
    curvature = np.abs(xp * ypp - yp * xpp) / np.power(xp**2 + yp**2, 1.5)
    return curvature


def smooth_spline_path(x, y, error=0.01):
    #return np.array([x, y]).T
    try:
        t = np.arange(x.shape[0])
        std = error * np.ones_like(x)
        fx = UnivariateSpline(t, x, k=2, w=1 / np.sqrt(std))
        fy = UnivariateSpline(t, y, k=2, w=1 / np.sqrt(std))
        return np.array([fx(t), fy(t)]).T
    except Exception as exc:
        print(f"fit error {exc}")
        return np.array([x, y]).T
