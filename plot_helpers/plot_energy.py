import matplotlib.pyplot as plt

from .utilities import param_get

def plot_energy(time_trajectory, state_trajectory, model, ax=None, plot_param={}):
    """
    Plot the energy of the system over time.

    time_trajectory: [1xN] np.array, time points of the trajectory
    state_trajectory: [MxN] np.array, state points of the trajectory
    model: the model used to generate the trajectory
    ax: matplotlib axis object, optional
    plot_param: dict, parameters for plotting
    """

    # Get parameters for plotting
    title = param_get(plot_param, "title", "Energy vs Time")
    xlabel = param_get(plot_param, "xlabel", "Time (s)")
    ylabel = param_get(plot_param, "ylabel", "Energy (J)")

    if ax is None:
        _, ax = plt.subplots()

    kinetic, potential = model.calculate_energy(state_trajectory)

    ax.plot(time_trajectory, potential, label=f"potential")
    ax.plot(time_trajectory, kinetic, label=f"kinetic")
    ax.plot(time_trajectory, potential + kinetic, label=f"total")
    ax.set_xlabel(xlabel)
    ax.set_ylabel(ylabel)
    ax.set_title(title)
    ax.legend()
    return ax
