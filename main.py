
from display import Display
from pendulum import Pendulum
from time import sleep

import threading

d = Display()
p = Pendulum([3,0])

def testing():
    while 1:
        p.state = p.Discrete() @ p.state
        print(p.state)
        sleep(0.01)
        d.save(0,p.state[0])

t = threading.Thread(target=testing, daemon= True)
t.start()
d.start()

