import numpy as np
import matplotlib.pyplot as plt

# Run-location bootstrap: this script lives two levels below the workspace root,
# so plain `python assignments/assignment_0/assignment_0.py` would not find the
# top-level `models` / `integrators` packages without PYTHONPATH.
import sys
from pathlib import Path
sys.path.insert(0, str(Path(__file__).resolve().parents[2]))

# from models.model_pendulum import ModelPendulum  # model instance, see selection below

# parameters
# integrator_type = "euler"  # "euler" or "rk4"
integrator_type = "rk4"    # "euler" or "rk4"
integrator = None
if integrator_type == "euler":
    from integrators import integrator_euler as integrator_module
    integrator = integrator_module.IntegratorEuler()
elif integrator_type == "rk4":
    from integrators import integrator_rk4 as integrator_module
    integrator = integrator_module.IntegratorRK4()
else:
    raise ValueError(f"Unknown integrator type: {integrator_type}")

# dynamic_type = "pendulum"      # "pendulum" or "bouncing_ball"
dynamic_type = "bouncing_ball"  # "pendulum" or "bouncing_ball"
model = None
params = None
initial_state = None
if dynamic_type == "pendulum":
    from models.model_pendulum import ModelPendulum
    model = ModelPendulum()
    params = {
        "gravity": 9.81,  # gravity m/s^2)
        "length": 1,  # rod length (m)
        "mass": 0.2,  # point mass at end of rod (kg)
        "damping_coeff": 0.0,  # damping coefficient (kg*m^2/s)
    }
    model.set_params(params)
    initial_state = np.array([np.pi / 4, 0.0])
elif dynamic_type == "bouncing_ball":
    from models.model_bouncing_ball import ModelBouncingBall
    model = ModelBouncingBall()
    params = {
        "gravity": 9.81,  # gravity m/s^2)
        "mass": 1.0,      # point mass at end of rod (kg)
        "spring_constant_ground": 100.0,    # N/m, positive for pushing up when the ball submerged
        "damping_coefficient_ground": 0.0,  # N/(m/s), possitive for dissipation
        "ground_height": 0.0,  # m
    }
    model.set_params(params)
    initial_state = np.array([1.0, 0.0])
else:
    raise ValueError(f"Unknown dynamic type: {dynamic_type}")


# pendulum
# euler: ok at 100, blow up at 1000
# timestep = 1e-5 * 1000
# rk4: ok at 10000, blow up at 100000
# timestep = 1e-5 * 100
# sim_time = 5.0

# bouncing ball
# euler: ok at 10, blow up at 100
# timestep = 1e-5 * 100
# rk4: ok at 1000, blow up at 10000
timestep = 1e-5 * 1000
sim_time = 5.0

n_timesteps = int(sim_time / timestep) + 1
time_traj = np.arange(n_timesteps) * timestep

# simulation loop
# Deprecated: explicit Euler integration
# state_traj = np.zeros((2, n_timesteps))
# state_traj[:, 0] = initial_state
# for step, t in enumerate(time_traj[:-1]):
#     state_traj[:, step + 1] = state_traj[:, step] + timestep * model.dynamics(
#         t, state_traj[:, step], params
#     )

# New abstractive signiture
# Demo checkpoint callback: integrators forward (t1, s1, t2, s2, model).
# To use a different model inside the callback, close over it with a lambda instead.
# checkpoint_callback = lambda t1, s1, t2, s2, model_cb=None: print(f"Phase change at time {t1}") if (s1[0] * s2[0]) < 0 else None
def checkpoint_callback(t1, s1, t2, s2, model_cb=None):
    if (s1[0] * s2[0]) < 0:
        print(f"Phase change at time {t1}")  # <- breakpoint here
state_traj = integrator.integrate(
    param_integrator={},
    param_model=params,
    time_trajectory=time_traj,
    initial_state=initial_state,
    model=model,
    checkpoint_callback=checkpoint_callback,
)

# sanity check the energies: since there is no actuation, and no damping, total energy should stay
# constant. If we turn on the damping coefficient, it should slowly bleed out energy until it comes to
# a stand-still.

kinetic_energy, potential_energy = model.calculate_energy(state_traj, params) # type: ignore

print(f"Initial total energy: {potential_energy[0] + kinetic_energy[0]}")
print(f"Final total energy: {potential_energy[-1] + kinetic_energy[-1]}")
print(f"Change ratio: {(potential_energy[-1] + kinetic_energy[-1]) / (potential_energy[0] + kinetic_energy[0])}")

if True:
# if False:
    plt.figure()
    # plt.plot(time_traj, state_traj[0,:],  label=f"{dynamic_type} height")
    # plt.plot(time_traj, state_traj[1,:],  label=f"{dynamic_type} velocity")
    plt.plot(time_traj, potential_energy, label=f"{dynamic_type} potential energy")
    plt.plot(time_traj, kinetic_energy, label=f"{dynamic_type} kinetic energy")
    plt.plot(time_traj, potential_energy + kinetic_energy, label=f"{dynamic_type} total energy")
    plt.xlabel("Time (s)")
    plt.ylabel("Energy (J)")
    plt.title(f"{dynamic_type} energy")
    plt.legend()
    plt.tight_layout()
    plt.show()

# TODO: make a phase portrait plot
