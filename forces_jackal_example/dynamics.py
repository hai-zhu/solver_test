import casadi as cd 

def jackal_dynamics_continuous(x, u, p):
    # state: x = [px, py, theta]
    # control: u = [vel, omega]
    px = x[0]
    py = x[1]
    theta = x[2]
    vel = u[0]
    omega = u[1]
    # dynamics
    px_dot = vel * cd.cos(theta)
    py_dot = vel * cd.sin(theta)
    theta_dot = omega 
    # output
    x_dot = [px_dot, py_dot, theta_dot]

    return x_dot 
    