import matplotlib.pyplot as plt
import numpy as np


class Display:
    def __init__(self):
        self.L = 1

        self.fig = plt.figure()
        self.ax = self.fig.add_subplot(111, autoscale_on=False, xlim=(-4, 4), ylim=(-1, 2))
        self.ax.set_aspect('equal')
        self.ax.grid()

        self.line, = self.ax.plot([], [], 'o-', lw=2)
        self.fig.show()

    def update(self,x,alpha):
        endpoint_y = self.L * np.cos(alpha)
        endpoint_x = self.L * np.sin(alpha) + x
        self.line.set_data([x,0], [endpoint_x,endpoint_y])
        self.fig.canvas.draw()
        
