import numpy as np


def plot_helper(ax, x, y, xlabel="", ylabel="", title="", **kwargs):
    ax.plot(x, y, **kwargs)
    ax.set_ylabel(ylabel)
    ax.set_xlabel(xlabel)
    ax.set_title(title)


def check_bounds(x, lb, ub):
    return x <= ub and x >= lb


def print_angles(**kwargs):
    for k, v in kwargs.items():
        print(f"{k}={np.rad2deg(v)}", end=", ")
    print()


def normalize_angle(theta, dbg=False) -> float:

    # project theta to [0, 360]
    theta_ = np.fmod(theta, 2 * np.pi)

    # project theta to [-180, 180]
    if theta_ > np.pi:
        theta_ -= 2 * np.pi
    elif theta_ < -np.pi:
        theta_ += 2 * np.pi

    if dbg:
        print_angles(angle_before=theta)
        print_angles(angle_after=theta_)

    assert check_bounds(theta_, -np.pi, np.pi)
    return theta_


def clip(x: float, min_max: tuple, dbg=False) -> float:
    if dbg:
        print("clipping")
    xmin, xmax = min_max
    assert xmin <= xmax
    return max(min(x, xmax), xmin)
