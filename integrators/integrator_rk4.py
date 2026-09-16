from .integrator_base import IntegratorBase
import numpy as np

import logging
logger = logging.getLogger(__name__)
logger.setLevel(logging.INFO)

# debug flags
if_step_num = False
step_num: int = 0
fail_step_num: int = 0
MOD_ = 1000000007

class IntegratorRK4(IntegratorBase):
    def integrate(self, 
                  param_integrator, 
                  param_model, 
                  time_trajectory, 
                  initial_state, 
                  model, 
                  checkpoint_callback=None,
                  jump_callback=None,
                  terminal_callback=None):
        # perf
        global step_num, fail_step_num
        step_num = 0
        fail_step_num = 0 

        N = len(time_trajectory)
        M = len(initial_state)
        state_trajectory = np.zeros((M, N))
        state_trajectory[:, 0] = initial_state
        integration_journal = {}

        for step, t in enumerate(time_trajectory[:-1]):
            start_time = time_trajectory[step]
            target_time = time_trajectory[step + 1]
            time_progress = start_time
            state_progress = state_trajectory[:, step]

            last_step_size = target_time - start_time
            while (time_progress < target_time):
                # find max step size that is valid
                step_size = min(last_step_size, target_time - time_progress)
                min_step_size = param_integrator.get("min_step_size", 1e-3)

                no_adapt = True                
                while (True):
                    is_valid, jump_id, new_state = self._try_step(param_integrator, param_model, time_progress, step_size, state_progress, model)
                    if is_valid or (step_size <= min_step_size):
                        # Call the checkpoint callback if it exists
                        if checkpoint_callback:
                            checkpoint_callback(time_progress, state_progress, time_progress + step_size, new_state, model, integration_journal)

                        if jump_id != 0 and jump_callback:
                            jump_callback(time_progress, state_progress, time_progress + step_size, new_state, jump_id, model, integration_journal)

                        state_progress = new_state
                        # double the step size if no adaptation was needed and the step was valid, otherwise keep it the same
                        last_step_size = 2 * step_size if (no_adapt and is_valid) else step_size 
                        time_progress += step_size

                        break
                    else:
                        no_adapt = False
                        step_size /= 2  # reduce step size and try again

                logger.debug(f"IntegratorRK4: t={time_progress:.4f}, step_size={step_size:.4e}, last_step_size={last_step_size:.4e}")

            state_trajectory[:, step + 1] = state_progress
            if terminal_callback and terminal_callback(time_trajectory, state_trajectory, step, model, integration_journal):
                # trim the trajectory to the current step + 1
                state_trajectory = state_trajectory[:, : step + 2]
                break
            
        if if_step_num: 
            logger.info(f"IntegratorRK4: Total steps taken: {step_num}, {fail_step_num} failed steps")

        return state_trajectory

    def _try_step(self, param_integrator, param_model, t: float, step: float, state: np.ndarray, model):
        """
        Attempt to take a single RK4 step. Currently if discrete jumps happens, we check if the step size is less than threshold.

        param_integrator: Dictionary of integrator parameters
        t: Current time
        step: Step size to attempt
        state: Current state vector
        model: The model object that provides the dynamics and discrete jump methods

        returns:
            is_valid: Boolean indicating if the step was successful (always True for fixed step RK4)
            jump_id: Integer indicating the type of discrete jump that occurred (0 if no jump)
            state: The new state after taking the RK4 step
        """
        # perf
        global step_num, fail_step_num

        k1 = model.dynamics(t, state, param_model)
        k2 = model.dynamics(t + step / 2, state + (step / 2) * k1, param_model)
        k3 = model.dynamics(t + step / 2, state + (step / 2) * k2, param_model)
        k4 = model.dynamics(t + step, state + step * k3, param_model)

        new_state = state + (step / 6) * (k1 + 2 * k2 + 2 * k3 + k4)

        jump_id, new_state_plus = model.discrete_jump(new_state, param_model)
        max_step_size_during_jump = param_integrator.get("max_step_size_during_jump", 1e-3)

        if if_step_num: 
            step_num = (step_num + 1) % MOD_

        if jump_id != 0 and step > max_step_size_during_jump:
            if if_step_num: 
                fail_step_num = (fail_step_num + 1) % MOD_
            return False, jump_id, new_state_plus  # Step is invalid due to discrete jump

        return True, jump_id, new_state_plus