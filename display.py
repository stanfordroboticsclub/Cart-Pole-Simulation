import matplotlib.pyplot as plt
import numpy as np

import threading


class Display:
    def __init__(self):
        self.L = 1
        plt.ioff()

        self.fig = plt.figure()
        self.ax = self.fig.add_subplot(111, autoscale_on=False, xlim=(-4, 4), ylim=(-1, 2))
        self.ax.set_aspect('equal')
        self.ax.grid()

        self.line, = self.ax.plot([], [], 'o-', lw=2)
        # print('here')
        # self.fig.show(block=True)
        # print('past')

        def onclick(event):
            print('test')
            self.update(self.x, self.alpha)

        cid = self.fig.canvas.mpl_connect('draw_event', onclick)


    def eventloop(self):
        plt.show()
        print('exited')
        # while 1:
        #     plt.pause(0.001)
            # self.update(self.x,self.alpha)
            # self.fig.draw()


    def save(self,x,alpha):
        self.x = x
        self.alpha = alpha

    def update(self,x,alpha):
        endpoint_y = self.L * np.cos(alpha)
        endpoint_x = self.L * np.sin(alpha) + x
        self.line.set_data([x,endpoint_x], [0,endpoint_y])
        self.fig.canvas.draw_idle()
        
