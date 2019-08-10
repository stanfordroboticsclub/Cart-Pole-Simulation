import matplotlib.pyplot as plt
import numpy as np

import threading

import matplotlib.animation as animation


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

        # def onclick(event):
        #     print('test')
        #     self.update(self.x, self.alpha)

        # cid = self.fig.canvas.mpl_connect('draw_event', onclick)
        self.lock = threading.Lock()


    def start(self):
        ani = animation.FuncAnimation(self.fig, self.update, interval = 30)
        plt.show()

    def save(self,x,alpha):
        self.lock.acquire()
        self.x = x
        self.alpha = alpha
        self.lock.release()

    def update(self,frame):
        print(frame)
        if self.lock.acquire(blocking=False):
            endpoint_y = self.L * np.cos(self.alpha)
            endpoint_x = self.L * np.sin(self.alpha) + self.x
            self.lock.release()
            self.line.set_data([self.x,endpoint_x], [0,endpoint_y])
        return self.line,
        
