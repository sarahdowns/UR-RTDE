import matplotlib.pyplot as plt

def setup_realtime_plot():
    """
    Sets up the matplotlib figure, axes, and initial lines for real-time plotting.
    Returns the figure, axes, and lines objects.
    """
    plt.ion() # Turn on interactive mode
    fig, axs = plt.subplots(2, 2, figsize=(10, 8)) # Adjusted figsize for better viewing
    ax1, ax2, ax3, ax4 = axs.flatten()

    line1, = ax1.plot([], [], label="TCP X")
    line2, = ax2.plot([], [], label="Force X", color='green')
    line3, = ax3.plot([], [], label="Force Y", color='red')
    line4, = ax4.plot([], [], label="Force Z", color='orange')

    ax1.set_xlabel("Time (s)")
    ax1.set_ylabel("TCP X (m)")
    ax1.set_title("TCP X Position")

    ax2.set_xlabel("Time (s)")
    ax2.set_ylabel("Force X (N)")
    ax2.set_title("TCP Force X")

    ax3.set_xlabel("Time (s)")
    ax3.set_ylabel("Force Y (N)")
    ax3.set_title("TCP Force Y")

    ax4.set_xlabel("Time (s)")
    ax4.set_ylabel("Force Z (N)")
    ax4.set_title("TCP Force Z")

    ax1.legend()
    ax2.legend()
    ax3.legend()
    ax4.legend()

    plt.tight_layout() # Adjust subplot params for a tight layout

    # Return all necessary objects to update later
    return fig, axs, [line1, line2, line3, line4]

def update_realtime_plot(lines, timestamps, tcp_x, force_x, force_y, force_z):
    """
    Updates the data for the real-time plot.
    """
    line1, line2, line3, line4 = lines
    ax1, ax2, ax3, ax4 = plt.gcf().get_axes() # Get axes from the current figure

    line1.set_data(timestamps, tcp_x)
    line2.set_data(timestamps, force_x)
    line3.set_data(timestamps, force_y)
    line4.set_data(timestamps, force_z)

    # Autoscale only if there's new data, otherwise it can shrink to zero
    if timestamps:
        for ax in [ax1, ax2, ax3, ax4]:
            ax.relim()
            ax.autoscale_view()

    plt.pause(0.01) # Use a smaller pause for potentially smoother updates

def close_realtime_plot():
    """
    Closes the matplotlib plot.
    """
    plt.ioff() # Turn off interactive mode
    plt.close() # Close the figure