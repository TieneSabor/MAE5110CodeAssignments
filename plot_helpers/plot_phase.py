import matplotlib.pyplot as plt

from .utilities import param_get

def plot_phase(state_trajectory, ax=None, plot_param={}):
    """
    Plot the phase portrait of the system

    state_trajectory: [MxN] np.array, state points of the trajectory
    ax: matplotlib axis object, optional
    plot_param: dict, parameters for plotting
    """

    # Get parameters for plotting
    title = param_get(plot_param, "title", "Phase Portrait")
    xlabel = param_get(plot_param, "xlabel", "State 1")
    ylabel = param_get(plot_param, "ylabel", "State 2")
    state1_index = param_get(plot_param, "state1_index", 0)
    state2_index = param_get(plot_param, "state2_index", 1)

    if ax is None:
        _, ax = plt.subplots()

    ax.plot(state_trajectory[state1_index, :], state_trajectory[state2_index, :])
    ax.set_xlabel(xlabel)
    ax.set_ylabel(ylabel)
    ax.set_title(title)
    ax.grid(True)
    return ax
    