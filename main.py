
from display import Display
from systems import Pendulum
from time import sleep
import cvxpy as cp
import numpy as np

d = Display()

init = [0,0]
p = Pendulum(init)

def main():
    T = 400
    N = p.N
    M = p.M

    sleep(1)

    U = cp.Variable((T,  M))
    X = cp.Variable((T+1,N))

    control = np.zeros((T,  M))
    states =  np.zeros((T+1,N))
    while 1:
        const = []
        const.append( X[0,:] == np.zeros((N)))

        p.state = np.array(init)
        states[0,:] = p.state
        for i in range(T):
            # print(i)
            A, B = p.linearize(p.state, control[i])

            p.next(control[i])
            states[i+1,:] = p.state
            const.append( X[i+1,:] == A @ X[i,:] + B @ U[i,:] ) 

            sleep(0.001)
            d.update(0,p.state[0])

        X_p = X + states
        U_p = U + control

        const.append( U_p <=  0.5)
        const.append( U_p >= -0.5)
        # const.append( X<=  0.3)
        # const.append( X>= -0.3)

        cost = 1000*cp.sum((X_p[-1:,0] - np.pi)**2) + \
               1000*cp.sum((X_p[-1:,1]   )**2) +\
                10* cp.sum((X_p[:,0]- np.pi)**2) + \
                0
                # 1* cp.sum((U_p)**2)
                # 0.1* cp.sum((U_p[1:] - U_p[:-1])**2)

        prob = cp.Problem( cp.Minimize( cost), const)
        prob.solve()
        print(U.value)

        for a,b in zip(U.value+control, X.value+states):
            print("angle", b[0], "vel", b[1], "act", a[0])
        control += U.value
        
        sleep(0.1)

d.start(main)

