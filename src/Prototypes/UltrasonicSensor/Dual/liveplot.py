import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation

def void():
    # Set up the figure and axis
    fig, ax = plt.subplots()
    ax.set_xlim(0, 2*np.pi)
    ax.set_ylim(-1, 1)

    # Initialize the line objects for sine and cosine
    line_sin, = ax.plot([], [], label='Sin')
    line_cos, = ax.plot([], [], label='Cos')

    # Set up the legend
    ax.legend(loc='upper right')

    # Define the function to update the plot
    def update(frame):
        x = np.linspace(0, 2*np.pi, 1000)
        y_sin = np.sin(x + frame)
        y_cos = np.cos(x + frame)
        line_sin.set_data(x, y_sin)
        line_cos.set_data(x, y_cos)
        return line_sin, line_cos

    # Set up the animation
    ani = FuncAnimation(fig, update, frames=np.linspace(0, 2*np.pi, 200),
                        blit=True, interval=10)

    # Show the plot
    plt.show()

def PlotDistance(distance1, distance2):
    # setup plot figure
    fig, ax = plt.subplots()

    ax.set_xlim(0, 2 * np.pi)
    ax.set_ylim(-2, 2) # sensor range => (4 cm, 4 m)

    # create line objects for both distances
    line_dist1, = ax.plot([], [], label="Sensor 1")
    line_dist2, = ax.plot([], [], label="Sensor 2")

    ax.legend(loc="upper right")

    def UpdatePlot(frame):
        x = np.linspace(0, 2 * np.pi, 1000)

        # plot data
        line_dist1.set_data(x, distance1)
        line_dist2.set_data(x, distance2)

        return line_dist1, line_dist2

    # init animation
    animation = FuncAnimation(fig, UpdatePlot, frames=np.linspace(0, 2 * np.pi, 200), blit=True, interval=10)

    # display
    plt.show()

def main():
    PlotDistance(np.sin(np.linspace(0, 2 * np.pi, 1000)), np.tan(np.linspace(0, 2 * np.pi, 1000)))

if __name__ == "__main__":
    main()


