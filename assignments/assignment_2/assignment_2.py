# Run-location bootstrap: this script lives two levels below the workspace root,
# so plain `python assignments/assignment_1/assignment_1.py` would not find the
# top-level `models` / `integrators` packages without PYTHONPATH.
import sys
from pathlib import Path
sys.path.insert(0, str(Path(__file__).resolve().parents[2]))

import matplotlib
matplotlib.use('TkAgg')  # Or try 'QtAgg' if TkAgg doesn't work
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation, PillowWriter

import numpy as np
import yaml
import argparse

import logging
logging.basicConfig(level=logging.INFO, format="[%(asctime)s][%(funcName)s] %(levelname)s %(name)s: %(message)s", datefmt="%H:%M:%S")
logger = logging.getLogger(__name__)
logger.setLevel(logging.INFO)

from models.model_inverted_pendulum_walker import ModelInvertedPendulumWalker
from integrators.integrator_rk4 import IntegratorRK4
from plot_helpers.plot_energy import plot_energy
from plot_helpers.plot_phase import plot_phase
from plot_helpers.plot_grid_map import plot_grid_map

def animate_test():
    # Fixed controls for this visualization example.
    params = {
        "gravity": 9.81,  # m/s^2
        "length": 1.0,  # m
        "mass": 1.0,  # kg
        "incline": 0.06,  # rad
        "angle_of_attack": np.pi / 8,  # rad
        "ankle_torque": 0.0,  # N m
    }

    initial_state = np.array([0.0, 3.0])
    timestep = 1e-3
    sim_time = 3.0
    desired_number_of_steps = 3

    n_timesteps = round(sim_time / timestep) + 1
    time_traj = np.arange(n_timesteps) * timestep
    state_traj = np.zeros((2, n_timesteps))
    state_traj[:, 0] = initial_state
    completed_steps = 0

    model = ModelInvertedPendulumWalker()

    # define jump and terminal callbacks to count completed steps and stop simulation after desired number of steps
    def jump_callback(t1, state1, t2, state2, jump_id, model):
        nonlocal completed_steps
        completed_steps += 1

    def terminal_callback(t1, state1, t2, state2, model):
        if completed_steps >= desired_number_of_steps:
            return True
        else:
            return False

    integrator = IntegratorRK4()
    param_integrator = {"min_step_size": 1e-4, "max_step_size_during_jump": 1e-4}
    state_traj = integrator.integrate(
        param_integrator=param_integrator,
        param_model=params,
        time_trajectory=time_traj,
        initial_state=initial_state,
        model=model,
        checkpoint_callback=None,
        jump_callback=jump_callback,
        terminal_callback=terminal_callback
    )

    _, step = np.shape(state_traj)
    time_traj = time_traj[: step]

    fig, ax = plt.subplots(figsize=(8, 5), layout="constrained")


    def draw_frame(index):
        # The massless swing leg is repositioned instantaneously at each impact.
        model.visualize(state_traj[:, index], params, ax=ax)
        ax.set_title(f"t = {time_traj[index]:.2f} s")


    # Simulate at a small timestep, but render only 25 frames per second.
    fps = 25
    frame_stride = round(1 / (fps * timestep))
    frame_indices = list(range(0, time_traj.size, frame_stride))
    if frame_indices[-1] != time_traj.size - 1:
        frame_indices.append(time_traj.size - 1)

    animation = FuncAnimation(
        fig, draw_frame, frames=frame_indices, interval=1000 / fps, repeat=False
    )
    output = Path("assignments/assignment_2/output")
    output.mkdir(parents=True, exist_ok=True)
    animation.save(output / "walker.gif", writer=PillowWriter(fps=fps))

    # To save an MP4 instead, install FFmpeg and use:
    # animation.save(output / "walker.mp4", writer="ffmpeg", fps=fps)
    print(f"Saved {output / 'walker.gif'} ({completed_steps} footstrikes).")
    plt.show()

# helpers
class LUT2D:
    def __init__(self, x_grids, y_grids, labels, out_default=0):
        self.x_grids = x_grids # np array of shape (M, )
        self.y_grids = y_grids # np array of shape (N, )
        self.labels = labels   # np array of shape (M, N)
        self.out_default = out_default # default output if input is out of bounds


    def query(self, x, y):
        if x < self.x_grids[0] or x > self.x_grids[-1] or y < self.y_grids[0] or y > self.y_grids[-1]:
            return self.out_default

        # find nearest index in x_grids and y_grids
        x_idx = np.argmin(np.abs(self.x_grids - x))
        y_idx = np.argmin(np.abs(self.y_grids - y))
        return self.labels[x_idx, y_idx]
        

def stabilization_criteria(states, state_bounds, last_n_steps = 10):
    """ Check if for all dimension i, avg(states[i, -last_n_steps:-1]) <= state_bounds[i]
    params:
        states: [MxN] np array, M state dimensions, N time steps
        state_bounds: [Mx1] np array, bounds for each state dimension
        last_n_steps: int, number of last steps to consider for averaging
    returns:
        bool, True if all state dimensions are within bounds, False otherwise
    """
    M, N = states.shape
    last_n_steps = min(last_n_steps, N)
    for i in range(M):
        avg_state = np.mean(np.abs(states[i, -last_n_steps:-1]))
        if avg_state > state_bounds[i]:
            return False
    return True

# define controller as callbacks
class TorqueCallback:
    def __init__(self, params_control):
        self.params_control = params_control

    def __call__(self, t1, state1, t2, state2, model, integration_journal):
        # torque = - p * theta - d * theta_dot - mglsin(theta)
        theta = state2[0]
        theta_dot = state2[1]
        torque = (
            - self.params_control["p_gain"] * theta
            - self.params_control["d_gain"] * theta_dot
            - model.param_["mass"] * model.param_["gravity"] * model.param_["length"] * np.sin(theta)
        )
        torque = np.clip(torque, self.params_control["torque_lower_bound"], self.params_control["torque_upper_bound"])
        model.param_["ankle_torque"] = torque

class AngleOfAttackCallback:
    def __init__(self, params_control):
        self.params_control = params_control

    def __call__(self, t1, state1, t2, state2, jump_id, model, integration_journal):
        # angle_of_attack = ...
        jump = 1 if jump_id != 0 else 0
        integration_journal["jump_num"] = integration_journal.get("jump_num", 0) + jump

class TerminalCallback:
    def __init__(self, state_bounds, jump_limit, last_n_steps=10):
        self.state_bounds = state_bounds
        self.jump_limit = jump_limit
        self.last_n_steps = last_n_steps

    def __call__(self, time_trajectory, state_trajectory, step, model, integration_journal):
        # check if the last_n_steps of state_trajectory are within state_bounds
        stabilized = False
        if step + 1 >= self.last_n_steps:
            stabilized = stabilization_criteria(state_trajectory[:, :step + 1], self.state_bounds, self.last_n_steps)
        # or we have enough number of jumps
        jump_num = integration_journal.get("jump_num", 0)
        stopped = False
        if jump_num >= self.jump_limit:
            stopped = True
        if stabilized or stopped:
            logger.debug(f"TerminalCallback: step={step}, stabilized={stabilized}, stopped={stopped}")
        return stabilized or stopped

# default params
config_map_default = {
    "model": {
        "gravity": 9.81,  # m/s^2
        "length": 1.0,  # m
        "mass": 1.0,  # kg
        "incline": 0.06,  # rad
        "angle_of_attack": np.pi / 8,  # rad
        "ankle_torque": 0.0,  # N m
    },
    "control": {
        "p_gain": 2.0,
        "d_gain": 2.0,
    },
    "integrator": {
        "min_step_size": 1e-4,
        "max_step_size_during_jump": 1e-4,
    },
    "torque_control_RoA": {
        "initial_theta_range": [-0.3, 0.3],
        "initial_theta_dot_range": [-1.0, 1.0],
        "theta_grid_size": 30,
        "theta_dot_grid_size": 30,
        "timestep": 1e-2,
        "sim_time": 10.0,
        "stabilization_bounds": [0.05, 0.001],
        "show_plot": True,
        "save_plot_path": "assignments/assignment_2/output/torque_control_RoA.png"
    }
}

# RoA of the torque controller
def compute_RoA(model, config_map: dict):
    params_model = config_map.get("model", config_map_default["model"])
    params_control = config_map.get("control", config_map_default["control"])
    params_integrator = config_map.get("integrator", config_map_default["integrator"])
    params_torque_control_RoA = config_map.get("torque_control_RoA", config_map_default["torque_control_RoA"])

    # overwrite the torque bounds
    params_control["torque_lower_bound"] = -0.1 * params_model["mass"] * params_model["gravity"] * params_model["length"]
    params_control["torque_upper_bound"] = 0.05 * params_model["mass"] * params_model["gravity"] * params_model["length"]

    model.set_params(params_model)
    timestep = params_torque_control_RoA.get("timestep", 1e-2)
    sim_time = params_torque_control_RoA.get("sim_time", 10.0)
    time_trajectory = np.arange(0, sim_time + timestep, timestep)

    state_bounds = np.array(params_torque_control_RoA.get("stabilization_bounds", [0.05, 0.001]))

    # pack the payloads under different initial conditions
    initial_thetas = np.linspace(params_torque_control_RoA["initial_theta_range"][0], \
                                 params_torque_control_RoA["initial_theta_range"][1], \
                                 params_torque_control_RoA["theta_grid_size"])
    initial_theta_dots = np.linspace(params_torque_control_RoA["initial_theta_dot_range"][0], \
                                     params_torque_control_RoA["initial_theta_dot_range"][1], \
                                     params_torque_control_RoA["theta_dot_grid_size"])
    payloads = []
    initial_states = [[np.zeros(2)] * len(initial_theta_dots) for _ in range(len(initial_thetas))]
    indices = []
    for i, theta in enumerate(initial_thetas):
        for j, theta_dot in enumerate(initial_theta_dots):
            initial_state = np.array([theta, theta_dot])
            initial_states[i][j] = initial_state
            indices.append((i, j))
            payloads.append(
                (
                    params_integrator,
                    None,
                    time_trajectory,
                    initial_state,
                    model,
                    TorqueCallback(params_control),
                    AngleOfAttackCallback(params_control),
                    TerminalCallback(state_bounds=state_bounds, jump_limit=1, last_n_steps=100)
                )
            )

    # run the simulations in parallel
    integrator = IntegratorRK4()
    import concurrent.futures
    results = []
    with concurrent.futures.ProcessPoolExecutor() as executor:
        futures = [executor.submit(integrator.integrate, *payload) for payload in payloads]
        results = [f.result() for f in futures]
        # for future in concurrent.futures.as_completed(futures):
        #     results.append(future.result())

    # check if each simulation stabilized
    stability_results = np.zeros((len(initial_thetas), len(initial_theta_dots)))
    for index, state_trajectory in zip(indices, results):
        is_stable = stabilization_criteria(state_trajectory, state_bounds, last_n_steps=100)
        # Find the index of the initial state in the grid
        i, j = index
        stability_results[i, j] = int(is_stable)

    # print the stability results
    for i, j in indices:
        logger.info(f"Simulation {i}, {j}: init: {initial_states[i][j]}, {'Stable' if stability_results[i, j] == 1 else 'Unstable'}")

    # if we want plot or save fig
    if_plot = params_torque_control_RoA.get("show_plot", True)
    save_plot_path = params_torque_control_RoA.get("save_plot_path", None)
    if not if_plot and save_plot_path is None:
        return

    stability_labels = {0: "Unstable", 1: "Stable"}
    fig, ax = plt.subplots(figsize=(8, 5), layout="constrained")
    ax = plot_grid_map(initial_thetas, initial_theta_dots, stability_results, stability_labels, ax=ax)

    if if_plot:
        plt.show()

    if save_plot_path is not None:
        plt.savefig(save_plot_path)

    lut = LUT2D(initial_thetas, initial_theta_dots, stability_results, out_default=0)
    return lut


def stabilize_upright():
    # The upright stabilization controller test

    # params
    model = ModelInvertedPendulumWalker()
    params_model = model.generate_params()
    mass = params_model["mass"]
    gravity = params_model["gravity"]
    length = params_model["length"]
    params_control = \
    {
        "p_gain": 2.0,
        "d_gain": 2.0,
        "torque_lower_bound": -0.1 * mass * gravity * length,
        "torque_upper_bound": 0.05 * mass * gravity * length,
    }
    model.set_params(params_model)

    # run simulation
    params_integrator = {"min_step_size": 1e-4, "max_step_size_during_jump": 1e-4}
    # initial_state = np.array([0.01538462, -0.04871795])
    initial_state = np.array([0.8, 0.0])
    timestep = 1e-2
    sim_time = 10.0
    time_trajectory = np.arange(0, sim_time + timestep, timestep)

    integrator = IntegratorRK4()
    state_bounds = np.array([0.05, 0.001])
    state_trajectory = integrator.integrate(
        param_integrator=params_integrator,
        param_model=None,
        time_trajectory=time_trajectory,
        initial_state=initial_state,
        model=model,
        checkpoint_callback=TorqueCallback(params_control),
        jump_callback=AngleOfAttackCallback(params_control),
        terminal_callback=TerminalCallback(state_bounds=state_bounds, jump_limit=1, last_n_steps=100)
    )
    time_trajectory = time_trajectory[:state_trajectory.shape[1]]

    # check if it stabilized
    is_stable = stabilization_criteria(state_trajectory, state_bounds, last_n_steps=100)

    if is_stable:
        print("The system is stable.")
    else:
        print("The system is not stable.")

    # an 2x1 subplot
    fig, (ax1, ax2) = plt.subplots(2, 1)
    #   4.1. Print Energies
    plot_param = {
        "title": "Rimless Wheel Energy vs Time",
        "xlabel": "Time (s)",
        "ylabel": "Energy (J)"
    }
    ax1 = plot_energy(time_trajectory, state_trajectory, model, ax=ax1, plot_param=plot_param)

    #   4.2. Phase Portrait
    plot_param = {
        "title": "Rimless Wheel Phase Portrait",
        "xlabel": "Theta (rad)",
        "ylabel": "Theta dot (rad/s)",
        "state1_index": 0,
        "state2_index": 1
    }
    ax2 = plot_phase(state_trajectory, ax=ax2, plot_param=plot_param)

    plt.show()

def main():
    # parse yaml config file
    default_path = Path(__file__).with_name("assignment_2.yaml")
    parser = argparse.ArgumentParser(description="Rimless-wheel runner (YAML-driven).")
    parser.add_argument("--config", default=str(default_path))
    config_path = parser.parse_args().config
    if config_path not in [None, ""] and Path(config_path).exists():
        with open(config_path, "r") as f:
            configs = yaml.safe_load(f) or {}
    else:
        configs = config_map_default

    # stabilize_upright()
    # return

    # run RoA test
    model = ModelInvertedPendulumWalker()
    lut = compute_RoA(model, configs)

if __name__ == "__main__":
    main()