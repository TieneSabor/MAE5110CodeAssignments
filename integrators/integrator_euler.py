from .integrator_base import IntegratorBase
import numpy as np

class IntegratorEuler(IntegratorBase):
    def integrate(self, param_integrator, param_model, time_trajectory, initial_state, model, checkpoint_callback=None):
        N = len(time_trajectory)
        M = len(initial_state)
        state_trajectory = np.zeros((M, N))
        state_trajectory[:, 0] = initial_state
        for step, t in enumerate(time_trajectory[:-1]):
            timestep = time_trajectory[step + 1] - time_trajectory[step]
            state_trajectory[:, step + 1] = state_trajectory[:, step] + timestep * model.dynamics(
                t, state_trajectory[:, step], param_model
            )

            state_trajectory[:, step + 1] = model.discrete_jump(state_trajectory[:, step + 1], param_model)

            if checkpoint_callback:
                checkpoint_callback(t, state_trajectory[:, step], t + timestep, state_trajectory[:, step + 1], model)
        return state_trajectory