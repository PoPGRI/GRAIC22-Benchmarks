#!/usr/bin/env python3

import numpy as np
from matplotlib import pyplot as plt

import simulator.models.ackermann_model as model
from simulator.models.ackermann_model import SID, UID
import graic22dc.src.control.user_control as control  # import PID, StanleyPID, Controller
import graic22dc.src.planner.pathplanner as pp
import graic22dc.src.planner.user_decision as baseline_planner
import simulator.tracks.track1 as track1
from src.common.utils import plot_helper, print_angles, normalize_angle


def simulate(
    T: int, state0: np.ndarray, track: track1.Track, planner: pp.PathPlanner, controller: control.Controller, dt=0.1
):
    """
    Uses the Euler solver with time step dt
    """
    xy0 = np.array([501.0, 0.0])

    t0 = 0.0

    time, traj, xytraj = [t0], [state0], [xy0]
    control_history = [np.array([0.0, 0.0])]

    xy = xy0
    prev_xy = xy
    state = state0
    t = t0

    for i in range(T):
        # current_position, current_rotation, current_velocity = current_pose
        # psi = current_rotation[2] # Euler
        # cx, cy = current_position
        # #cv = current_velocity

        # tx, ty, target_v = target_pose
        vx = state[SID.v_long] * np.cos(state[SID.psi]) - state[SID.v_lat] * np.sin(state[SID.psi])
        vy = state[SID.v_long] * np.sin(state[SID.psi]) + state[SID.v_lat] * np.cos(state[SID.psi])
        current_pose = (xy[0], xy[1]), (0, 0, state[SID.psi]), (vx, vy)

        lane_marker = track.get_lane_info(xy)
        obstacleList = []
        checkpoint = None
        target_pose = planner.get_ref_state(current_pose, obstacleList, lane_marker, checkpoint)
        # target_pose = (planner.get_wp(xy)[0], planner.get_wp(xy)[1], 100)

        u = controller.compute(target_pose, current_pose)

        control_history.append(u)
        state_prev = state
        state = state + model.dynamics(state, u) * dt

        # normalize between [0,360]
        state[SID.psi] = normalize_angle(state[SID.psi], dbg=False)

        # normalize between [0,360]
        state_prev[SID.psi] = normalize_angle(state_prev[SID.psi], dbg=False)
        xy = xy + model.xy_position(state_prev[SID.v_long], state_prev[SID.v_lat], state_prev[SID.psi]) * dt

        traj.append(state)
        xytraj.append(xy)
        time.append(t)
        t += dt

    traj = np.asarray(traj)
    xytraj = np.asarray(xytraj)
    control_history = np.asarray(control_history)
    # import IPython
    # IPython.embed()

    # Plot state trajectories
    fig1, ax1 = plt.subplots(nrows=3, ncols=2)
    plot_helper(ax1[0, 0], time, traj[:, SID.v_long], "time", "v_long", "Longitudinal velocity")
    plot_helper(ax1[0, 1], time, traj[:, SID.v_lat], "time", "v_lat", "Latitudinal velocity")
    plot_helper(
        ax1[1, 0],
        time,
        np.rad2deg(traj[:, SID.psi]),
        "time",
        "psi",
        "Heading angle (deg)",
    )
    plot_helper(ax1[1, 1], time, traj[:, SID.r], "time", "r", "Angular velocity (rad/s)")
    plot_helper(
        ax1[2, 0],
        time,
        control_history[:, UID.a_long],
        "time",
        "longitudinal acceleration",
        "Control",
    )
    plot_helper(
        ax1[2, 1],
        time,
        np.rad2deg(control_history[:, UID.steering_angle]),
        "time",
        "steering input (deg)",
        "Control",
    )

    # Plot kinematic trajectories
    fig2, ax2 = plt.subplots(nrows=1, ncols=2)
    plot_helper(
        ax2[0],
        xytraj[:, 0],
        xytraj[:, 1],
        "X",
        "Y",
        "Kinematic trajectory in X-Y",
        label="Actual",
        color="black",
        linewidth=3,
    )
    # overlay track
    plot_helper(
        ax2[0], track.path_center[:, 0], track.path_center[:, 1], color="green", label="center-lane", marker="*"
    )
    plot_helper(ax2[0], track.path_right[:, 0], track.path_right[:, 1], color="red", label="right-lane")
    plot_helper(ax2[0], track.path_left[:, 0], track.path_left[:, 1], color="red", label="left-lane")
    ax2[0].legend()
    #
    plot_helper(ax2[1], time, xytraj[:, 0], label="X")
    plot_helper(ax2[1], time, xytraj[:, 1], "t", "x-y", "x and y vs t ", label="Y")
    ax2[1].legend()

    controller.plot_me()

    plt.show()


FIXED_VEL = 5e-0


def init_state() -> np.ndarray:
    state0 = np.zeros(SID.len)
    state0[model.SID.v_long] = FIXED_VEL
    # state0[Idx.v_lat]
    # state0[Idx.psi]
    # state0[Idx.r]
    # state0[Idx.y]
    return state0


if __name__ == "__main__":

    dt = 0.1

    ######## Track Selection ########
    # track = track1.SimpleTrack(*track1.circle())
    track = track1.SimpleTrack(
        *track1.sinusoid_perturbed_circle(perturbation_amp=50, radius=500, track_width=100, numPoints=100), dbg=False
    )
    ##################################

    ###### Controller Selection ######
    controller = control.StanleyPID(dbg=True, dt=dt)
    # controller = control.Baseline_PID(dbg=True)
    # controller = control.Zero(dbg=True)
    # controller = control.OpenLoop(T)
    ##################################

    ######## Planner Selection########
    planner = pp.FollowFixedTracks(track, vref=FIXED_VEL, dbg=False)
    # planner = baseline_planner.Baseline(LaneInfo)
    ##################################

    # Add obstacle_gen, and finally club all inside a scenario generator
    T = 4000
    simulate(T, init_state(), track, planner, controller, dt=dt)
