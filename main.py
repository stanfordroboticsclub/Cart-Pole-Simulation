
from display import Display
from pendulum import Pendulum
from time import sleep
import cvxpy as cp
import numpy as np

d = Display()

init = [0.5,0]
p = Pendulum(init)

def testing():

    N = 300

    U = cp.Variable((N,  1))
    X = cp.Variable((N+1,2))

    const = []

    states =  np.zeros((N+1,2))
    control = np.zeros((N,  1))

    while 1:
        const = []
        const.append( U <=  2.1)
        const.append( U >= -2.1)
        const.append( X[0,:] == np.zeros((2)))

        p.state = np.array(init)
        states[0,:] = p.state
        for i in range(N):
            print(i)
            A, B = p.linearize()

            p.next(control[i])
            states[i+1,:] = p.state
            const.append( X[i+1,:] == A @ X[i,:] + B @ U[i,:] ) 

            sleep(0.01)
            d.update(0,p.state[0])

        prob = cp.Problem( cp.Minimize( cp.sum( (X[:,:]+ states[:,:])**2)), const)
        prob.solve()
        print(U.value)

        control = U.value
        
        sleep(2)



d.start(testing)

