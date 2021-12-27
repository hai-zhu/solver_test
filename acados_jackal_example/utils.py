import numpy as np 

def my_RK2(x, u, f, h, p):
    # perform one step explicit RK2 integrator for nonlinear system x_dot = f(x, u, p)
    # inputs: 
    #   x: current state, list
    #   u: current control input, list
    #   f: nonlinear system dynamics function 
    #   h: step size (dt)
    #   p: passing parameters (to f)
    # outputs:
    #   x_next: next step state after integration, list 
    k1 = h * np.array(f(x, u, p))
    k2 = h * np.array(f(x+0.5*k1, u, p))
    x_next = x + k2

    return list(x_next)
    