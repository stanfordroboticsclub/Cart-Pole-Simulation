
from display import Display
from pendulum import Pendulum
from time import sleep

d = Display()
p = Pendulum([1,0])

def testing():
    while 1:
        p.state = p.Discrete() @ p.state
        print(p.state)
        sleep(0.01)
        d.update(0,p.state[0])

d.start(testing)

