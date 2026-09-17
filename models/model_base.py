from abc import ABC, abstractmethod
import logging
import numpy as np

logger = logging.getLogger(__name__)
logger.setLevel(logging.INFO)

class ModelBase(ABC):
    @abstractmethod
    def __init__(self, params:dict|None=None):
        self.M_ = None  # state dimension, to be defined in subclasses
        self.dim_map_ = {}  # mapping of dimension names to indices
        self.param_ = params if params is not None else self.generate_params()

    def get_state_dimension(self, dim_name: str=""):
        """
        Get the state dimension of the model.

        dim_name: str, specifies which dimension to return.  If empty, returns |M_|
        
        Returns:
            int, state dimension
        """
        if dim_name in self.dim_map_:
            return self.dim_map_[dim_name]
        logger.warning(f"Dimension '{dim_name}' not found in dim_map_")
        return self.M_

    def get_params(self):
        """
        Get the parameters of the model.

        Returns:
            dict, parameters of the model
        """
        return self.param_

    def set_params(self, params: dict):
        """
        Set the parameters for the model.

        params: dict, parameters to set
        """
        self.param_ = params
    
    @abstractmethod
    def dynamics(self, t, state, params: dict | None = None):
        """
        Compute the dynamics of the system at time t for a given state and parameters.
        
        t: float, current time
        state: [Mx1] np.array, current state of the system
        params: dict, parameters for the model.  If None, uses self.param_
        
        Returns:
            [Mx1] np.array, derivatives of the state
        """
        logger.warning("ModelBase: dynamics() method not implemented")
        return np.zeros_like(state)

    @abstractmethod
    def discrete_jump(self, state, params: dict | None = None) -> tuple[int, np.ndarray]:
        """
        Compute the discrete jump in state due to events (e.g., collisions).
        
        state: [Mx1] np.array, current state of the system
        params: dict, parameters for the model.  If None, uses self.param_
        
        Returns:
            int: jump flag: 0 <-> discrete jump not occurred, otherwise the jump id
            [Mx1] np.array, new state after the discrete jump
        """
        logger.debug("ModelBase: discrete_jump() no-op")
        return 0, state

    @abstractmethod
    def generate_params(self):
        """
        Generate default parameters for the model.
        
        Returns:
            dict, default parameters for the model
        """
        logger.warning("ModelBase: generate_params() method not implemented")
        return {}

    @abstractmethod
    def calculate_energy(self, state, params: dict | None = None):
        """
        Compute energies for a given state and parameters.
        
        state: [Mx1] np.array or [MxN] np.array, current state of the system
        params: dict, parameters for the model.  If None, uses self.param_
        
        Returns:
            tuple of 1D np.arrays, (kinetic_energy [N], potential_energy [N])
        """
        logger.warning("ModelBase: calculate_energy() method not implemented")
        return np.zeros(state.shape[1]), np.zeros(state.shape[1])
        