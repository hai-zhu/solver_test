import numpy as np
def my_RK2(x, u, f, h, p):

    # perform one step explicit RK2 integrator for nonlinear system \dot{x} = f(x,u,p)
    # inputs:
    #   x: current state
    #   u: current control input
    #   f: nonlinear dynamics function handle
    #   h: step size (sampling time)
    #   p: passing parameters (to f)
    # outputs;
    #   x_next: next step state after integration
    k1 = h * np.array(f(x, u, p))
    k2 = h * np.array(f(x+k1/2.0, u, p))
    x_next = x + k2

    return list(x_next) 

