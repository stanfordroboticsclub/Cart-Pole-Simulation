
from display import Display
from pendulum import Pendulum
from time import sleep

d = Display()
p = Pendulum([2.5,0])

def testing():
    while 1:
        # A, B = p.linearize()
        # p.state = A @ p.state
        p.next()
        print(p.state)
        sleep(0.01)
        d.update(0,p.state[0])

d.start(testing)

