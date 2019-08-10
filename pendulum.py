

import numpy as np

# angle in radians from vertical going ccw
# angular velocity

state = [0, 0]


class Pendulum:
    # x_dot = Ac * x
    # x_n+1 = Ad * x

    def __init__(self, init):
        pass

        self.state = np.array(init)


    def A(self, state = None):
        if state == None:
            state = self.state

        theta = state[0]

        # got this from jacojian of ODE
        g = 1
        m = 1
        l = 1
        A = np.array([[0,                    1],
                      [-g*np.cos(theta)/l, 0]])


        # theta_dd = - g*sin(theta)/l

        return A

    def Discrete(self, state = None):
        dt = 0.01
        A = self.A(state)
        return np.eye(A.shape[0]) + dt * A

    def next(self):
        theta = self.state[0]
        theta_d = self.state[1]

        theta_dd = -m*g*sin(theta)/l
        theta_d = -m*g*sin(theta)/l


        
