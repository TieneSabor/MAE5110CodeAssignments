from .model_base import ModelBase
import numpy as np

import logging
logger = logging.getLogger(__name__)
logger.setLevel(logging.INFO)

class ModelPendulum(ModelBase):
    def __init__(self):
        super().__init__()
        self.M_ = 2 # state dimension [angle, angular_velocity]
        self.dim_map_ = {
            "angle": 0,
            "angular_velocity": 1,
            "": self.M_
        }

    def dynamics(self, t, state, params: dict | None = None):
        if params is None:
            params = self.get_params()
        gravity = params["gravity"]
        length = params["length"]
        mass = params["mass"]
        damping_coeff = params["damping_coeff"]

        angle = state[0]
        angular_velocity = state[1]

        angular_acceleration = (
            mass * gravity * length * np.sin(angle)
            - damping_coeff * angular_velocity  # <-- DAMPING TERM
        ) / (mass * length**2)

        state_derivative = np.array([angular_velocity, angular_acceleration])
        return state_derivative

    def discrete_jump(self, state, params: dict | None = None) -> tuple[int, np.ndarray]:
        return 0, state # no discrete jump for pendulum

    def generate_params(self):
        params = {
            "gravity": 9.81,  # gravity m/s^2)
            "length": 1,  # rod length (m)
            "mass": 1,  # point mass at end of rod (kg)
            "damping_coeff": 0.1,  # damping coefficient (kg*m^2/s)
        }
        return params

    def calculate_energy(self, state, params: dict | None = None):
        """Compute energies for a state ``(2,)`` or trajectory ``(2, N)``."""
        if params is None:
            params = self.get_params()
        gravity = params["gravity"]
        length = params["length"]
        mass = params["mass"]

        angle = state[0]  # indexes entire row "vectorized" if state is (2, N)
        angular_velocity = state[1]

        kinetic_energy = 0.5 * mass * (length * angular_velocity) ** 2
        potential_energy = mass * gravity * length * np.cos(angle)
        return kinetic_energy, potential_energy
