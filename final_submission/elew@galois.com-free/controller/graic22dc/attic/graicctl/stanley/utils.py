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


def normalize_angle(theta) -> float:
    theta = np.fmod(theta, 2 * np.pi)

    if theta > np.pi:
        theta -= 2 * np.pi
    elif theta < -np.pi:
        theta += 2 * np.pi

    assert check_bounds(theta, -np.pi, np.pi)
    return theta
