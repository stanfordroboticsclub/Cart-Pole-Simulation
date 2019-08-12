

import numpy as np

# angle in radians from vertical going ccw
# angular velocity

state = [0, 0]


class Pendulum:
    # x_dot = Ac * x
    # x_n+1 = Ad * x

    def __init__(self, init):
        self.state = np.array(init)
        self.dt = 0.1

        self.g = 1
        self.m = 1
        self.l = 1


    def linearize(self, state = None):
        if state == None:
            state = self.state

        theta = state[0]

        # got this from jacojian of ODE
        g = self.g
        m = self.m
        l = self.l
        A = np.array([[0,                    1],
                      [-g*np.cos(theta)/l, -0.1]])

        B = np.array([0,1])

        return np.eye(A.shape[0]) + self.dt*A, self.dt*B

    def next(self, u = None):
        g = self.g
        m = self.m
        l = self.l

        x_d = np.array( [self.state[1], -g*np.sin(self.state[0])/l -0.1 * self.state[1] + u[0] ])
        self.state  =  self.state + x_d * self.dt

        return self.state


        
