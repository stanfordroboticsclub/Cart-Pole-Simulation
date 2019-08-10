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
        self.lock = threading.Lock()

    def start(self, to_run):
        thread = threading.Thread(target=to_run, daemon= True)
        thread.start()
        ani = animation.FuncAnimation(self.fig, self.redraw, interval = 30,blit = True)
        plt.show()

    def update(self,x,alpha):
        self.lock.acquire()
        self.x = x
        self.alpha = alpha
        self.lock.release()

    def redraw(self,frame):
        print(frame)
        if self.lock.acquire(blocking=False):
            endpoint_y = self.L * np.cos(self.alpha)
            endpoint_x = self.L * np.sin(self.alpha) + self.x
            self.lock.release()
            self.line.set_data([self.x,endpoint_x], [0,endpoint_y])
            return self.line,
        else:
            return ()
        
