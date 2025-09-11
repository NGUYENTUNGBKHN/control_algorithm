"""
    ODE is stand for Ordinary Difference Equation
"""
from enum import Enum
import numpy as np

class ODE_TYPE(Enum):
    EULER = 1
    RK4 = 2

class ODE():
    def __init__(self,              # pointer      
                state,              # state : initalize state
                times,              # times : time space
                derivative_func,    # derivative_func : funtion (state, step, t, dt)
                flag                # flag
            )-> None :
        self.state = state
        self.times = times
        self.derivative_func = derivative_func
        self.flag = flag

    """
        Euler Method:

        Iterative formula:
            yn+1 = yn + h*f(tn,yn)
        Return:
            a derivative array
    """
    def integrate_euler(self, state, step, t, dt, derivative_func):
        k1 = derivative_func(state, step, t, dt)
        return [v + dt * k1_ for v, k1_ in zip(state, k1)]
    """
        Runge-kutta 4 : 

        Iterative formula:
            yn+1 = yn + 1/6(k1 + 2k2 + 2k3 + k4)*h
            where is step size (xn+1 = xn + h):
            k1 = f(xn,yn)
            k2 = f(xn + h/2, yn + k1 * h/2)
            k3 = f(xn + h/2, yn + k2 * h/2)
            k4 = f(xn + h, yn + k3 * h)
        Return:
            a derivative array
    """
    def integrate_rk4(self, state, step, t, dt, derivative_func):
        k1 = derivative_func(state, step, t, dt)
        k2 = derivative_func([v + k1_ * dt/2 for v, k1_ in zip(state, k1)], step, t + dt/2, dt)
        k3 = derivative_func([v + k2_ * dt/2 for v, k2_ in zip(state, k2)], step, t + dt/2, dt)
        k4 = derivative_func([v + k3_ * dt for v, k3_ in zip(state, k3)], step, t + dt, dt)
        return [v + ( k1_ + 2*k2_ + 2*k3_ + k4_)*dt/6 for v, k1_, k2_, k3_, k4_ in zip(state, k1, k2, k3, k4)]
    
    def solve(self):
        dt = self.times[0] - self.times[1]
        states = [self.state]
        for step, t in enumerate(self.times):
            if self.flag == ODE_TYPE.EULER:
                states.append(self.integrate_euler(states[-1], step, t, dt, self.derivative_func))
            elif self.flag == ODE_TYPE.RK4:
                states.append(self.integrate_rk4(states[-1], step, t, dt, self.derivative_func))
            else:
                states = []
        return np.array(states)