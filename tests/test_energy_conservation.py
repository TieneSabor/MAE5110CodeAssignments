import sys
from pathlib import Path

import numpy as np
import pytest

# Make top-level `models` / `integrators` importable when pytest is run
# from the workspace root (mirrors assignments/assignment_0/assignment_0.py).
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from integrators.integrator_euler import IntegratorEuler
from integrators.integrator_rk4 import IntegratorRK4
from models.model_bouncing_ball import ModelBouncingBall


@pytest.mark.parametrize("integrator_cls", [IntegratorEuler, IntegratorRK4])
def test_energy_conservation(integrator_cls):
    model = ModelBouncingBall()
    params = {
        "gravity": 9.81,
        "mass": 1.0,
        "spring_constant_ground": 100.0,
        "damping_coefficient_ground": 0.0,
        "ground_height": 0.0,
    }
    model.set_params(params)
    initial_state = np.array([1.0, 0.0])

    timestep = 1e-5
    sim_time = 5.0
    n_timesteps = int(sim_time / timestep) + 1
    time_traj = np.arange(n_timesteps) * timestep

    integrator = integrator_cls()
    state_traj = integrator.integrate(
        param_integrator={},
        param_model=params,
        time_trajectory=time_traj,
        initial_state=initial_state,
        model=model,
    )

    kinetic_energy, potential_energy = model.calculate_energy(state_traj, params)
    total_energy = kinetic_energy + potential_energy
    initial_energy = total_energy[0]
    final_energy = total_energy[-1]
    loss_ratio = abs(final_energy - initial_energy) / abs(initial_energy)

    assert loss_ratio < 0.01, (
        f"{integrator_cls.__name__}: energy loss {loss_ratio:.6f} >= 1% "
        f"(initial {initial_energy:.6f}, final {final_energy:.6f})"
    )
