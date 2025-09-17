import numpy as np
import matplotlib.pyplot as plt
from matplotlib import animation

from robotun_lib.ode.ode_solver import ODE, ODE_TYPE
from robotun_lib.model.free_fall import free_fall
from robotun_lib.ctrl.pid_ctrl import pid_ctrl

th_pid = pid_ctrl(Kp = 40.0, Ki = 0.0, Kd = 20.0, target = 0.0)

velocity_pid = pid_ctrl(Kp = 0.0085, Ki = 0.001, Kd = 0.0, target = 0.0)

position_pid = pid_ctrl(Kp = 0.07, Ki = 0.0, Kd = 0.07, target = 0.0)

arduino_pid = pid_ctrl(Kp = 2000.0, Ki = 0.0, Kd = 0.0, target = 0.0)

object = free_fall()
u_history = []

def limit(v, lim):
    if v > lim:
        return lim
    elif v < -lim:
        return -lim
    else:
        return v
    
def low_pass_filter(alpha, init):
    y = init

    def _inner(x):
        nonlocal y
        y = alpha * x + (1 - alpha) * y
        return y

    return _inner

filtered_measurement = low_pass_filter(0.9934 , 0.0)

def derivate(state, step, t, dt):
    dth, th = state

    u = arduino_pid.get_ctrl(th, dt)

    u = limit(u, 20)
    u_history.append(u)
    
    return [object.get_delta2_v(state, u), dth]


if __name__ == "__main__":
    times = np.linspace(0, 10, 500)
    ode = ODE([0, np.pi/3], times, derivate, ODE_TYPE.RK4)
    solution = []
    solution = ode.solve()
    theta = solution[:,1]

    th_line, = plt.plot(times, solution[:-1, 1], label="theta")
    plt.legend([ th_line,], ['theta'])
    plt.grid(True)
    plt.show()
