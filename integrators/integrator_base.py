from abc import ABC, abstractmethod
import logging
import numpy as np

logger = logging.getLogger(__name__)
logger.setLevel(logging.INFO)

class IntegratorBase(ABC):
    @abstractmethod
    def integrate(self, 
                  param_integrator, 
                  param_model, 
                  time_trajectory, 
                  initial_state, 
                  model, 
                  checkpoint_callback=None, 
                  jump_callback=None,
                  terminal_callback=None):
        """
        Integrate the system over time trajectory
        
        param_integrator:  dict, parameters for the integrator
        pram_model:        dict, parameters for the model
        time_trajectory:   [1xN] np array, time trajectory to integrate over
        initial_state:     [Mx1] np array, initial state of the system
        model:             contains model.dynamics defined as (t: float, state: [Mx1] np.array, params: dict) -> [Mx1] np.array
        checkpoint_callback: function, defined as 
                                (t1, state1, t2, state2, 
                                    model=None, 
                                    integration_journal = None) -> None: 
                                called at each time step with the current state
        jump_callback:       function, defined as 
                                (t1, state1, t2, state2, jump_id, 
                                    model=None, 
                                    integration_journal = None) -> None: 
                                called when a discrete jump occurs
        terminal_callback:   function, defined as 
                                (time_traj, state_traj, step, 
                                    model=None, 
                                    integration_journal = None) -> bool:
                                return True if the integration should terminate 
        """
        logger.info("IntegratorBase: integrate() method not implemented")
        N = len(time_trajectory)
        M = len(initial_state)
        state_trajectory = np.zeros((M, N))
        return state_trajectory, {}
