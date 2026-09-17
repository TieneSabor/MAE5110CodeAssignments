import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation, PillowWriter

def plot_animation(time_traj, state_traj, 
                   model, 
                   fps=30,
                   ax = None,
                   plot_param={}):

    plot_path = plot_param.get("plot_path", None)

    if ax is None:
        fig, ax = plt.subplots(figsize=(8, 6), layout="constrained")
    else:
        fig = ax.get_figure()

    def draw_frame(index):
        # The massless swing leg is repositioned instantaneously at each impact.
        model.visualize(state_traj[:, index], params = None, ax=ax)
        ax.set_title(f"t = {time_traj[index]:.2f} s")

    # Simulate at a small timestep, but render only 25 frames per second.
    frame_indices = [0, ]
    for i in range(len(time_traj)):
        last_frame_time = time_traj[frame_indices[-1]] if frame_indices else 0
        if time_traj[i] - last_frame_time >= 1.0 / fps:
            frame_indices.append(i)

    animation = FuncAnimation(
        fig, draw_frame, frames=frame_indices, interval=1000 / fps, repeat=False
    )

    if plot_path:
        animation.save(plot_path, writer=PillowWriter(fps=fps))
        # animation.save(plot_path, writer="ffmpeg", fps=fps)

    return ax, animation
