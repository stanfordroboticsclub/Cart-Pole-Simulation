
from display import Display
from pendulum import Pendulum
from time import sleep
import cvxpy as cp
import numpy as np

d = Display()

init = [0,0]
p = Pendulum(init)

def testing():

    N = 1000
    sleep(1)

    U = cp.Variable((N,  1))
    X = cp.Variable((N+1,2))

    const = []

    states =  np.zeros((N+1,2))
    control = np.zeros((N,  1))

    while 1:
        const = []

        # const.append( X <=  0.1)
        # const.append( X >= -0.1)
        const.append( X[0,:] == np.zeros((2)))

        p.state = np.array(init)
        states[0,:] = p.state
        for i in range(N):
            print(i)
            A, B = p.linearize()

            p.next(control[i])
            states[i+1,:] = p.state
            const.append( X[i+1,:] == A @ X[i,:] + B @ U[i,:] ) 

            sleep(0.005)
            d.update(0,p.state[0])

        const.append( U + control <=  0.1)
        const.append( U + control >= -0.1)
        # const.append( X<=  0.3)
        # const.append( X>= -0.3)

        X_p = X + states
        U_p = U + control
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
        
        sleep(2)


d.start(testing)

