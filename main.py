
from display import Display
from systems import Pendulum, CartPole
from time import sleep
import cvxpy as cp
import numpy as np

import scipy.linalg
 
def lqr(A,B,Q,R):
    """Solve the continuous time lqr controller.
    dx/dt = A x + B u
    cost = integral x.T*Q*x + u.T*R*u
    """
    X = np.array(scipy.linalg.solve_continuous_are(A, B, Q, R))
    K = np.array(scipy.linalg.inv(R)@(B.T@X))
    eigVals, eigVecs = scipy.linalg.eig(A-B@K)
    return K, X, eigVals
 
def dlqr(A,B,Q,R):
    """Solve the discrete time lqr controller.
    x[k+1] = A x[k] + B u[k]
    cost = sum x[k].T*Q*x[k] + u[k].T*R*u[k]
    """
    X = np.array(scipy.linalg.solve_discrete_are(A, B, Q, R))
    K = np.array(scipy.linalg.inv(B.T@X@B+R)@(B.T@X@A))
    eigVals, eigVecs = scipy.linalg.eig(A-B@K)
    return K, X, eigVals

d = Display()

# init = [0,0]
# p = Pendulum(init)
init = [0,0,0,0]
p = CartPole(init)

def main():
    T = 75
    N = p.N
    M = p.M

    target = np.array([0,0,0,np.pi])
    A, B = p.linearize(target, np.zeros((M)))
    Qf = 1000*np.eye(N)
    Qf[0,0] = 10
    Qf[1,1] = 10

    R  = 1*np.eye(M)
    K,_,_ = dlqr(A,B,Qf,R)
    print("K", K)
    print("A", A)
    print("B", B)

    sleep(1)

    U = cp.Variable((T,  M))
    X = cp.Variable((T+1,N))

    control = np.zeros((T,  M))
    states =  np.zeros((T+1,N))
    atempt = 0
    while 1:
        print(atempt)
        atempt += 1
        const = []
        const.append( X[0,:] == np.zeros((N)))

        p.state = np.array(init)
        states[0,:] = p.state
        for i in range(T):
            # print(i)
            A, B = p.linearize(p.state, control[i])

            p.next(control[i])
            # print(p.state)
            states[i+1,:] = p.state
            const.append( X[i+1,:] == A @ X[i,:] + B @ U[i,:] ) 
            
            sleep(0.010)
            # if atempt > 100:
            #     sleep(0.020)
            d.update(p.state[1],p.state[3])

        if atempt > 100:
            for i in range(300):
                u = np.clip(K @ (p.state - target),-1,1)
                p.next(-u)
                sleep(0.02)
                d.update(p.state[1],p.state[3])

        X_p = X + states
        U_p = U + control

        const.append( U_p <=    1)
        const.append( U_p >= -  1)
        const.append( X_p[:,1] <=  2)
        const.append( X_p[:,1] >= -2)

        const.append( U[:,0] >= -0.1)
        const.append( U[:,0] <=  0.1)

        # cost_pend = 1000*cp.sum((X_p[-1:,0] - np.pi)**2) + \
        #             1000*cp.sum((X_p[-1:,1]   )**2) +\
        #         10* cp.sum((X_p[:,0]- np.pi)**2) + \
        #         0
                # 1* cp.sum((U_p)**2)
                # 0.1* cp.sum((U_p[1:] - U_p[:-1])**2)

        cost =1000* cp.sum((X_p[-1,:]  - target)**2) + \
                1000* cp.sum((X_p[-2,:]  - target)**2) + \
                1000* cp.sum((X_p[-3,:]  - target)**2) + \
                10* cp.sum((X_p[:,3]   -  np.pi)**2) + \
                10* cp.sum((X_p[:,2]           )**2) +\
                 2* cp.sum((X_p[:,1]           )**2) +\
                 2* cp.sum((X_p[:,0]           )**2) +\
                 1* cp.sum((U_p[:,0]           )**2) +\
                0

        prob = cp.Problem( cp.Minimize( cost), const)
        prob.solve()
        # print(U.value)

        # for a,b in zip(U.value+control, X.value+states):
        #     print("angle", b[0], "vel", b[1], "act", a[0])
        control += U.value
        
        sleep(0.1)

d.start(main)

