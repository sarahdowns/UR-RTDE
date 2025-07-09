import matplotlib.pyplot as plt

def setup_realtime_plot():
    """
    Sets up the matplotlib figure, axes, and initial lines for real-time plotting.
    Returns the figure, axes, and lines objects.
    """
    plt.ion() # Turn on interactive mode for real-time updates
    fig, axs = plt.subplots(2, 2, figsize=(10, 8))
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

    plt.tight_layout()
    
    return fig, axs, [line1, line2, line3, line4]

def update_realtime_plot(lines, timestamps, tcp_x, force_x, force_y, force_z):
    """
    Updates the data for the real-time plot.
    """
    line1, line2, line3, line4 = lines
    # Using plt.gcf().get_axes() is robust if you have multiple figures
    # but for a single figure, using axs directly (if passed) or global access is fine.
    # For simplicity, let's keep it robust for this shared function.
    current_axes = plt.gcf().get_axes() 
    if len(current_axes) >= 4: # Ensure all axes exist before trying to update
        ax1, ax2, ax3, ax4 = current_axes
    else:
        # Handle error or just skip update if plot isn't correctly initialized
        return


    line1.set_data(timestamps, tcp_x)
    line2.set_data(timestamps, force_x)
    line3.set_data(timestamps, force_y)
    line4.set_data(timestamps, force_z)

    if timestamps: # Only relim/autoscale if there's data
        for ax in [ax1, ax2, ax3, ax4]:
            ax.relim()
            ax.autoscale_view()
    
    plt.pause(0.01)

def close_realtime_plot():
    """
    This function will no longer close the plot automatically.
    The plot will wait for manual closure after plt.show().
    """
    print("Real-time plot has finished updating. Please close the plot window manually to continue the script.")
    # plt.ioff() # We will handle this with plt.show(block=True) in the main script
    # plt.close() # plt.show() will handle closing when user dismisses it