import matplotlib.pyplot as plt
class Display:
    def __init__(self, x=400, y = 100):
        self.fig = plt.figure()
        self.ax = self.fig.add_subplot(111, autoscale_on=False, xlim=(-2, 2), ylim=(-2, 2))
        self.ax.set_aspect('equal')
        self.ax.grid()
        self.line, = self.ax.plot([], [], 'o-', lw=2)
        self.fig.show()

    def update(self,x,alpha):
        self.line.set_data([0,x], [0,alpha])
        self.fig.canvas.draw()
        
