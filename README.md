##Cart Pole Simulation


![Screenshot](https://github.com/stanfordroboticsclub/Cart-Pole-Simulation/blob/master/example.png)

A quick test of a swingup controller for a CartPole robot using an [iterative LQR](https://medium.com/@jonathan_hui/rl-lqr-ilqr-linear-quadratic-regulator-a5de5104c750) controller. Instead of using a backwards pass the optimal control at each step is calculated using convex optimisation. It takes around 100 iterations to converge.

Code should be quite easiliy adaptable for study of other systems. 

To run
---

```python3 main.py```


Requirements
---
- Cvxpy
- Numpy
- Matplotlib
- Sympy


