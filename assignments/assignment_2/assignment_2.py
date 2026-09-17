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
from plot_helpers.plot_animtation import plot_animation

# =====================
# helpers
# =====================
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

    def update(self, x, y, label):
        if x < self.x_grids[0] or x > self.x_grids[-1] or y < self.y_grids[0] or y > self.y_grids[-1]:
            return False

        # find nearest index in x_grids and y_grids
        x_idx = np.argmin(np.abs(self.x_grids - x))
        y_idx = np.argmin(np.abs(self.y_grids - y))
        self.labels[x_idx, y_idx] = label
        return True

    def refine(self, new_x_grids, new_y_grids):
        # refine the LUT to a new grid
        new_labels = np.zeros((len(new_x_grids), len(new_y_grids)))
        for i, x in enumerate(new_x_grids):
            for j, y in enumerate(new_y_grids):
                new_labels[i, j] = self.query(x, y)
        self.x_grids = new_x_grids
        self.y_grids = new_y_grids
        self.labels = new_labels
        
# =====================
# Feedback Control
# =====================
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
    def __init__(self, params_control, Poincare_section = 3.14, RoA_lut=None, AoA_control_luts=None):
        self.params_control = params_control
        self.Poincare_section = Poincare_section
        self.RoA_lut = RoA_lut
        self.AoA_control_luts = AoA_control_luts

    def __call__(self, t1, state1, t2, state2, model, integration_journal):
        theta = state2[0]
        theta_dot = state2[1]
        # Check if the current state is within the RoA, if not, apply the PD control law
        is_stable = True
        if self.RoA_lut is not None:
            is_stable = self.RoA_lut.query(state2[0], state2[1]) # if the current state is within the RoA
        if is_stable or self.RoA_lut is None:
            # torque = - p * theta - d * theta_dot - mglsin(theta)
            torque = (
                - self.params_control["p_gain"] * theta
                - self.params_control["d_gain"] * theta_dot
                - model.param_["mass"] * model.param_["gravity"] * model.param_["length"] * np.sin(theta)
            )
            torque = np.clip(torque, self.params_control["torque_lower_bound"], self.params_control["torque_upper_bound"])
            model.param_["ankle_torque"] = torque
            return # Current state is not within the RoA, do nothing

        # if penetrating the Poincare section, try applying the AoA control law
        theta_past = state1[0]
        theta_dot_past = state1[1]
        if not (theta_past < self.Poincare_section and theta >= self.Poincare_section and theta_dot_past > 0):
            return # not penetrating the Poincare section, do nothing
        if self.AoA_control_luts is None or len(self.AoA_control_luts) == 0:
            return
        integration_journal["AoA_num"] = integration_journal.get("AoA_num", 0) + 1
        # Retired: Roll as much as we can: find the LUT at the max step
        # Find the min expected step number in the AoA LUTs, and use the corresponding AoA to control
        # AoA_LUT can contain <= 0 values which means unstable or not initialized
        max_step = len(self.AoA_control_luts)
        for i in range(max_step):
            # LUT = self.AoA_control_luts[max_step - 1 - i]
            LUT = self.AoA_control_luts[i]
            # find AoA in LUT s.t. expected step number is the highest
            min_expected_step_num = max_step + 1
            best_AoA = None
            for AoA in LUT.y_grids:
                expected_step_num = LUT.query(theta_dot, AoA)
                if expected_step_num < min_expected_step_num and expected_step_num >= 0:
                    min_expected_step_num = expected_step_num
                    best_AoA = AoA
            if best_AoA is not None:
                model.param_["angle_of_attack"] = best_AoA
                integration_journal["expected_step_num"] = min_expected_step_num
                # logger.info(f"TorqueCallback: t={t2:.4f}, theta={theta:.4f}, theta_dot={theta_dot:.4f}, AoA={best_AoA:.4f}, expected_step_num={max_expected_step_num}")
                break

class AngleOfAttackCallback:
    def __init__(self, params_control):
        self.params_control = params_control

    def __call__(self, t1, state1, t2, state2, jump_id, model, integration_journal):
        # angle_of_attack = ...
        jump = 1 if jump_id != 0 else 0
        integration_journal["jump_num"] = integration_journal.get("jump_num", 0) + jump

class TerminalCallback:
    def __init__(self, state_bounds, AoA_limit, jump_limit, last_n_steps=10):
        self.state_bounds = state_bounds
        self.jump_limit = jump_limit
        self.AoA_limit = AoA_limit
        self.last_n_steps = last_n_steps

    def __call__(self, time_trajectory, state_trajectory, step, model, integration_journal):
        # check if the last_n_steps of state_trajectory are within state_bounds
        stabilized = False
        if step + 1 >= self.last_n_steps:
            stabilized = stabilization_criteria(state_trajectory[:, :step + 1], self.state_bounds, self.last_n_steps)
        # or we have enough number of jumps
        jump_num = integration_journal.get("jump_num", 0)
        AoA_num = integration_journal.get("AoA_num", 0)
        stopped = False
        if jump_num >= self.jump_limit or AoA_num >= self.AoA_limit:
            stopped = True
        if stabilized or stopped:
            integration_journal["stabilized"] = stabilized
            logger.debug(f"TerminalCallback: step={step}, stabilized={stabilized}, stopped={stopped}")
        return stabilized or stopped

# =====================
# Main Assignment Logics
# =====================
# default params
config_map_default = {
    "model": {
        "gravity": 9.81,  # m/s^2
        "length": 1.0,  # m
        "mass": 1.0,  # kg
        "incline": 0.06,  # rad
        # "angle_of_attack": np.pi / 8,  # rad
        "angle_of_attack": 0.445,  # rad
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
        "initial_theta_range": [-0.4, 0.5],
        "initial_theta_dot_range": [-1.0, 1.0],
        "theta_grid_size": 40,
        "theta_dot_grid_size": 80,
        "timestep": 1e-2,
        "sim_time": 10.0,
        "stabilization_bounds": [0.05, 0.001],
        "show_plot": True,
        "save_plot_path": "assignments/assignment_2/output/torque_control_RoA.png"
    },
    "AoA_control": {
        "AoA_range": [0.3925, 0.4486],  # rad
        "Poincare_section_adjust": 0.3,  # rad
        "timestep": 1e-2,
        "sim_time": 10.0,
        "stabilization_bounds": [0.05, 0.001],
        "max_steps": 5,
        "show_plot": True,
        "grid_number": [100, 20],
        "accuracy_criteria": 0.97,
        "test_num": 500,
        "save_plot_path": "assignments/assignment_2/output/AoA_controls"
    },
    "plot_case": {
        "timestep": 1e-4,
        "sim_time": 15.0,
        "initial_theta": -0.0325,
        "initial_theta_dot": 3.90,
        "stabilization_bounds": [0.05, 0.001],
    },
    "control_save_path": "assignments/assignment_2/output/control_dump.pkl",
    "control_load_path": "assignments/assignment_2/output/control_dump.pkl"
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
    gamma = params_model["incline"]
    alpha = params_model["angle_of_attack"]
    initial_theta_min = max(gamma - alpha, params_torque_control_RoA["initial_theta_range"][0])
    initial_theta_max = min(gamma + alpha, params_torque_control_RoA["initial_theta_range"][1])
    initial_thetas = np.linspace(initial_theta_min, \
                                 initial_theta_max, \
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
                    TerminalCallback(state_bounds=state_bounds, AoA_limit=10000, jump_limit=1, last_n_steps=100)
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
    for index, result in zip(indices, results):
        state_trajectory, integration_journal = result
        is_stable = stabilization_criteria(state_trajectory, state_bounds, last_n_steps=100)
        # Find the index of the initial state in the grid
        i, j = index
        stability_results[i, j] = int(is_stable)

    # print the stability results
    for i, j in indices:
        logger.debug(f"Simulation {i}, {j}: init: {initial_states[i][j]}, {'Stable' if stability_results[i, j] == 1 else 'Unstable'}")

    # if we want plot or save fig
    if_plot = params_torque_control_RoA.get("show_plot", True)
    save_plot_path = params_torque_control_RoA.get("save_plot_path", None)
    if not if_plot and save_plot_path is None:
        return

    stability_labels = {0: "Unstable", 1: "Stable"}
    fig, ax = plt.subplots(figsize=(8, 5), layout="constrained")
    ax = plot_grid_map(initial_thetas, initial_theta_dots, stability_results, stability_labels, ax=ax)

    if save_plot_path is not None:
        plt.savefig(save_plot_path)

    if if_plot:
        plt.show()

    lut = LUT2D(initial_thetas, initial_theta_dots, stability_results, out_default=0)
    return lut

def AoA_control(model, RoA_lut: LUT2D, config_map: dict):
    params_model = config_map.get("model", config_map_default["model"])
    params_control = config_map.get("control", config_map_default["control"])
    params_integrator = config_map.get("integrator", config_map_default["integrator"])
    params_AoA_control = config_map.get("AoA_control", config_map_default["AoA_control"])

    # overwrite the torque bounds
    params_control["torque_lower_bound"] = -0.1 * params_model["mass"] * params_model["gravity"] * params_model["length"]
    params_control["torque_upper_bound"] = 0.05 * params_model["mass"] * params_model["gravity"] * params_model["length"]

    model.set_params(params_model)
    timestep = params_AoA_control.get("timestep", 1e-2)
    sim_time = params_AoA_control.get("sim_time", 10.0)
    time_trajectory = np.arange(0, sim_time + timestep, timestep)

    state_bounds = np.array(params_AoA_control.get("stabilization_bounds", [0.05, 0.001]))
    max_steps = params_AoA_control.get("max_steps", 10)

    # Build AoA map from step = 1 to step = max_steps
    AoA_control_luts = []
    theta_dot_max = np.sqrt(2 * params_model["gravity"] / params_model["length"])
    AoA_range = params_AoA_control.get("AoA_range", [0.3925, 0.4486])  # rad
    Poincare_section = params_model["incline"] - AoA_range[0] + params_AoA_control.get("Poincare_section_adjust", 0.3)  # rad, the Poincare section is at the min AoA
    for step in range(0, max_steps):
        # find the coarsest AoA grid that produce correct expected step number
        cur_grid_num = params_AoA_control.get("grid_number", [100, 5])  # [theta_dot_grid_num, AoA_grid_num]
        theta_dot_grid = np.linspace(0, theta_dot_max, cur_grid_num[0])
        AoA_grid = np.linspace(AoA_range[0], AoA_range[1], cur_grid_num[1])
        cur_lut = LUT2D(theta_dot_grid, AoA_grid, -2 * np.ones((len(theta_dot_grid), len(AoA_grid))), out_default=-1)
        while True:
            # test the current LUT accuracy
            # for each theta_dot, AoA, run a simulation and check if the expected step number is correct
            # and compute correctness ratio
            def parallel_sim(theta_dot_AoA_pairs: list[tuple[float, float]]):
                payloads = []
                for theta_dot, AoA in theta_dot_AoA_pairs:
                    initial_state = np.array([Poincare_section, theta_dot])
                    # set model AoA to the current AoA
                    params_model_payload = model.get_params().copy()
                    params_model_payload["angle_of_attack"] = AoA
                    # terminal condition for building AoA map:
                    # - if it stabilized, or tried AoA control once, or jump more than twice
                    payloads.append(
                        (
                            params_integrator,
                            params_model_payload,
                            time_trajectory,
                            initial_state,
                            model,
                            TorqueCallback(params_control, 
                                           Poincare_section=Poincare_section, 
                                           RoA_lut=RoA_lut, 
                                           AoA_control_luts=AoA_control_luts),
                            AngleOfAttackCallback(params_control),
                            TerminalCallback(state_bounds=state_bounds, AoA_limit=1, jump_limit=2, last_n_steps=100)
                        )
                    )

                # run the simulations in parallel
                integrator = IntegratorRK4()
                import concurrent.futures
                results = []
                with concurrent.futures.ProcessPoolExecutor() as executor:
                    futures = [executor.submit(integrator.integrate, *payload) for payload in payloads]
                    results = [f.result() for f in futures]

                return results

            # Build the table with the current grid
            theta_dot_AoA_pairs = [(theta_dot, AoA) for theta_dot in theta_dot_grid for AoA in AoA_grid]
            results = parallel_sim(theta_dot_AoA_pairs)
            for idx, result in enumerate(results):
                state_trajectory, integration_journal = result
                theta_dot = theta_dot_AoA_pairs[idx][0]
                AoA = theta_dot_AoA_pairs[idx][1]
                stabilized = integration_journal.get("stabilized", False)
                actual_step_num = -1
                if stabilized:
                    actual_step_num = 0
                elif "expected_step_num" in integration_journal:
                    actual_step_num = integration_journal["expected_step_num"] + 1

                cur_lut.update(theta_dot, AoA, actual_step_num)

            # sample a bunch of random theta_dot, AoA pairs to test the LUT accuracy
            num_samples = params_AoA_control.get("test_num", 300)
            theta_dot_samples = np.random.uniform(0, theta_dot_max, num_samples)
            AoA_samples = np.random.uniform(AoA_range[0], AoA_range[1], num_samples)
            theta_dot_AoA_pairs_test = [(theta_dot, AoA) for theta_dot, AoA in zip(theta_dot_samples, AoA_samples)]
            results_test = parallel_sim(theta_dot_AoA_pairs_test)

            # check if each simulation actually has the expected step number
            correct_count = 0.0
            total_count = 0.0
            unstable_count = 0.0
            for idx, result in enumerate(results_test):
                state_trajectory, integration_journal = result
                theta_dot = theta_dot_AoA_pairs_test[idx][0]
                AoA = theta_dot_AoA_pairs_test[idx][1]
                expected_step_num = cur_lut.query(theta_dot, AoA)
                stabilized = integration_journal.get("stabilized", False)
                actual_step_num = -1
                if stabilized:
                    actual_step_num = 0
                elif "expected_step_num" in integration_journal:
                    actual_step_num = integration_journal["expected_step_num"] + 1

                # if actual_step_num == expected_step_num:
                if (actual_step_num >= 0 and expected_step_num >= 0) or \
                   (actual_step_num == -1 and expected_step_num == -1): # as long as both are stable, we consider it correct
                    correct_count += 1.0
                total_count += 1.0
                if actual_step_num == -1:
                    unstable_count += 1.0
                
            correctness_ratio = correct_count / total_count
            logger.info(f"AoA_control: step={step}, correctness_ratio={correctness_ratio:.2f}, grid number={cur_grid_num}")
            if correctness_ratio >= params_AoA_control.get("accuracy_criteria", 0.99):
                break

            # Otherwise, we refine the grid
            cur_grid_num[0] *= 2
            cur_grid_num[1] *= 2
            theta_dot_grid = np.linspace(0, theta_dot_max, cur_grid_num[0])
            AoA_grid = np.linspace(AoA_range[0], AoA_range[1], cur_grid_num[1])
            cur_lut.refine(theta_dot_grid, AoA_grid)

        # visualize the current LUT
        fig, ax = plt.subplots(figsize=(8, 5), layout="constrained")
        max_expected_step_num = int(np.max(cur_lut.labels))
        step_labels = {i: f"{i} expected steps" for i in range(max_expected_step_num + 1)}
        step_labels[-1] = "Not stable"
        ax = plot_grid_map(theta_dot_grid, AoA_grid, cur_lut.labels, step_labels, ax=ax)
        plt.title(f"AoA_control: step={step}, correctness_ratio={correctness_ratio:.2f}, grid number={cur_grid_num}")

        if params_AoA_control.get("save_plot_path", None) is not None:
            Path(params_AoA_control.get("save_plot_path")).mkdir(parents=True, exist_ok=True)
            plt.savefig(f"{params_AoA_control.get('save_plot_path')}/AoA_control_step_{step}.png")
        
        if params_AoA_control.get("show_plot", True):
            plt.show()

        AoA_control_luts.append(cur_lut)

    return AoA_control_luts, Poincare_section

def plot_case(RoA_LUT: LUT2D | None, AoA_control_luts: list[LUT2D] | None, Poincare_section: float, config_map: dict):
    # The upright stabilization controller test

    # params
    model = ModelInvertedPendulumWalker()
    params_model = config_map.get("model", config_map_default["model"])
    params_control = config_map.get("control", config_map_default["control"])
    params_integrator = config_map.get("integrator", config_map_default["integrator"])
    plot_case_params = config_map.get("plot_case", config_map_default["plot_case"])

    # overwrite the torque bounds
    params_control["torque_lower_bound"] = -0.1 * params_model["mass"] * params_model["gravity"] * params_model["length"]
    params_control["torque_upper_bound"] = 0.05 * params_model["mass"] * params_model["gravity"] * params_model["length"]

    # run simulation
    initial_state = np.array([plot_case_params.get("initial_theta", 0.4),
                              plot_case_params.get("initial_theta_dot", 0.0)])
    timestep = plot_case_params.get("timestep", 1e-2)
    sim_time = plot_case_params.get("sim_time", 10.0)
    time_trajectory = np.arange(0, sim_time + timestep, timestep)
    state_bounds = np.array(plot_case_params.get("stabilization_bounds", [0.05, 0.001]))
    AoA_limit = 10000
    # if AoA_control_luts is not None:
    #     AoA_limit = len(AoA_control_luts)

    integrator = IntegratorRK4()
    state_trajectory, integration_journal = integrator.integrate(
        param_integrator=params_integrator,
        param_model=params_model,
        time_trajectory=time_trajectory,
        initial_state=initial_state,
        model=model,
        checkpoint_callback=TorqueCallback(params_control, 
                                           Poincare_section=Poincare_section, 
                                           RoA_lut=RoA_LUT, 
                                           AoA_control_luts=AoA_control_luts),
        jump_callback=AngleOfAttackCallback(params_control),
        terminal_callback=TerminalCallback(state_bounds=state_bounds, AoA_limit=AoA_limit, jump_limit=100, last_n_steps=100)
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

    # and plot the animation
    fig, ax = plt.subplots(figsize=(8, 5), layout="constrained")
    ax, animation = plot_animation(time_trajectory, state_trajectory, model, fps=25, ax=ax, plot_param={})
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
    
    control_load_path = configs.get("control_load_path", None)
    if control_load_path is not None and Path(control_load_path).exists():
        import pickle
        with open(control_load_path, "rb") as f:
            lut, AoA_control_luts, Poincare_section = pickle.load(f)
        logger.info(f"Loaded control data from {control_load_path}")
    else:
        # run RoA test
        model = ModelInvertedPendulumWalker()
        lut = compute_RoA(model, configs)
        if lut is None:
            logger.warning("RoA computation failed, skipping AoA control test.")
            return

        # run AoA control test
        AoA_control_luts, Poincare_section = AoA_control(model, lut, configs)

        control_save_path = configs.get("control_save_path", None)
        if control_save_path is not None:
            import pickle
            with open(control_save_path, "wb") as f:
                pickle.dump((lut, AoA_control_luts, Poincare_section), f)
            logger.info(f"Saved control data to {control_save_path}")

    # plot the case
    plot_case(lut, AoA_control_luts, Poincare_section, configs)

if __name__ == "__main__":
    main()