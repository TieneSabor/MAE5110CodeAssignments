from .model_base import ModelBase
import numpy as np

import logging
logger = logging.getLogger(__name__)
logger.setLevel(logging.INFO)

class ModelRimlessWheel(ModelBase):
    def __init__(self):
        # parent's constructor will call generate_params() to initialize self.param_
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

        mass = params["mass"]
        g = params["g"]
        length = params["length"]
        alpha = params["alpha"]
        gamma = params["gamma"]

        theta = state[0]
        theta_dot = state[1]

        theta_ddot = g * np.sin(theta) / length
        return np.array([theta_dot, theta_ddot])

    def discrete_jump(self, state, params: dict | None = None) -> np.ndarray:
        if params is None:
            params = self.get_params()

        alpha = params["alpha"]
        gamma = params["gamma"]

        theta = state[0]
        theta_dot = state[1]

        # contact results in state jump:
        # from alpha + gamma to gamma - alpha
        if (theta >= (alpha + gamma)) and (theta_dot > 0):
            logger.debug(f"ModelRimlessWheel: Forward jump. State: {state}")
            theta = gamma - alpha
            theta_dot = theta_dot * np.cos(2 * alpha)
            return np.array([theta, theta_dot])
        # from gamma - alpha to alpha + gamma
        elif (theta <= (gamma - alpha)) and (theta_dot < 0):
            logger.debug(f"ModelRimlessWheel: Backward jump. State: {state}")
            theta = alpha + gamma
            theta_dot = theta_dot * np.cos(2 * alpha)
            return np.array([theta, theta_dot])
        else:
            # no jump occurs
            logger.debug(f"ModelRimlessWheel: No jump. State: {state}")
            return state

    def generate_params(self):
        params = {
            "mass": 1.0,  # mass of the wheel (kg)
            "g": 9.81,  # acceleration due to gravity (m/s^2)
            "length": 1.0,  # length of the spokes (m).  Positive
            "alpha": 2 * np.pi / 6 / 2, # half the angle between spokes (radians).  Assume positive
            "gamma": np.pi / 8,  # angle of the slope (radians), default 22.5 degrees.  Assume positive and incline downwards
        }
        return params

    def calculate_energy(self, state, params: dict | None = None):
        if params is None:
            params = self.get_params()

        mass = params["mass"]
        g = params["g"]
        length = params["length"]
        # alpha = params["alpha"]
        # gamma = params["gamma"]

        angle = state[0]  # indexes entire row "vectorized" if state is (2, N)
        angular_velocity = state[1]

        kinetic_energy = 0.5 * mass * (length * angular_velocity) ** 2
        potential_energy = mass * g * length * np.cos(angle)
        
        return kinetic_energy, potential_energy
