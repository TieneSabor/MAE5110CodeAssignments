from .integrator_base import IntegratorBase
import numpy as np

class IntegratorEuler(IntegratorBase):
    def integrate(self, 
                  param_integrator, 
                  param_model, 
                  time_trajectory, 
                  initial_state, 
                  model, 
                  checkpoint_callback=None,
                  jump_callback=None,
                  terminal_callback=None):
        N = len(time_trajectory)
        M = len(initial_state)
        state_trajectory = np.zeros((M, N))
        state_trajectory[:, 0] = initial_state
        integration_journal = {}

        if param_model is not None:
            model.set_params(param_model)  # to support multi process

        for step, t in enumerate(time_trajectory[:-1]):
            timestep = time_trajectory[step + 1] - time_trajectory[step]
            state_trajectory[:, step + 1] = state_trajectory[:, step] + timestep * model.dynamics(
                t, state_trajectory[:, step]
            )

            jump_id, state_trajectory[:, step + 1] = model.discrete_jump(state_trajectory[:, step + 1])
            if jump_id != 0 and jump_callback:
                jump_callback(t, state_trajectory[:, step], t + timestep, state_trajectory[:, step + 1], jump_id, model, integration_journal)

            if checkpoint_callback:
                checkpoint_callback(t, state_trajectory[:, step], t + timestep, state_trajectory[:, step + 1], model, integration_journal)

            if terminal_callback and terminal_callback(time_trajectory, state_trajectory, step, model, integration_journal):
                # trim the trajectory to the current step + 1
                state_trajectory = state_trajectory[:, : step + 2]
                break

        return state_trajectory, integration_journal