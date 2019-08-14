

import numpy as np


class System:
    def __init__(self, init):
        raise NotImplemented
    def linearize(self, state, control):
        raise NotImplemented
    def next(self, control=None):
        raise NotImplemented

class Pendulum:
    N = 2
    M = 1
    def __init__(self, init):
        # state = [angle in radians from vertical down going ccw]
        #         [                             angular velocity]
        self.state = np.array(init)
        self.dt = 0.05

        self.g = 1
        self.m = 1
        self.l = 1

    def linearize(self, state, control):
        theta = state[0]

        # got this from jacojian of ODE
        g = self.g
        m = self.m
        l = self.l
        A = np.array([[0,                    1],
                      [-g*np.cos(theta)/l, -0.1]])

        B = np.array([0,1])

        return np.eye(A.shape[0]) + self.dt*A, self.dt*B

    def next(self, control = None):
        g = self.g
        m = self.m
        l = self.l
        u = control

        x_d = np.array( [self.state[1], -g*np.sin(self.state[0])/l -0.1 * self.state[1] + u[0] ])
        self.state  =  self.state + x_d * self.dt

        return self.state

        
class CartPole:
    N = 4
    M = 1
    def __init__(self, init):
        # state = [Cart velocity    ]
        #         [Position on rail ]
        #         [Angular velocity ]
        #         [angle?? (from MIT]
        self.state = np.array(init)
        self.dt = 0.05

    # https://ocw.mit.edu/courses/electrical-engineering-and-computer-science/6-832-underactuated-robotics-spring-2009/readings/MIT6_832s09_read_ch03.pdf
    # 2x ̈ + θ ̈cos θ − θ ̇2 sin θ = f
    # x ̈ cos θ + θ ̈ + sin θ = zero

    # [2,         cos(theta)] @ [    xdd] = [f + thetad^2 sin(theta)]
    # [cos(theta),        1 ] @ [thetadd] = [            -sin(theta)]
    def calc_dynamics(self):
        xd, x, thetad, theta = symbols('xd x thetad theta')
        f                    = symbols('f')

        # accel = Matrix([xdd, thetadd])
        accel = Matrix([[2, cos(theta)],[cos(theta), 1]]).inv() @ Matrix([f + thetad**2 * sin(theta), -sin(theta)]) 
        xdd, thetadd = accel

        dyn_sym =     Matrix([xdd, xd, thetadd, thetad])
        A_sym = dyn.jacobian([xd,  x,  thetad,  theta ])
        B_sym = dyn.jacobian([f])

        self.dyn = lambdify( [[xd,x,thetad, theta], [f]], dyn_sym, 'numpy')
        self.A   = lambdify( [[xd,x,thetad, theta], [f]], A_sym  , 'numpy')
        self.B   = lambdify( [[xd,x,thetad, theta], [f]], B_sym  , 'numpy')

    def linearize(self, state, u):
        if state == None:
            state = self.state

        u = np.zeros((self.M))

        A = self.A(state, u)
        B = self.B(state, u)

        return np.eye(A.shape[0]) + self.dt*A, self.dt*B

    def next(self, u = None):
        if u == None:
            u = np.zeros((self.M))

        self.state  =  self.state + self.dyn(self.state, u) * self.dt
        return self.state
