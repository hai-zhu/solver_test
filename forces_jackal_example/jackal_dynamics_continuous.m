function dx = dynamics(x, u, p)
    
    % state: x = [px, py, theta]
    % control: u = [vel, omega]
    px = x(1);
    py = x(2);
    theta = x(3);
    vel = u(1);
    omega = u(2);
    % dynamics
    px_dot = vel * cos(theta);
    py_dot = vel * sin(theta);
    theta_dot = omega;
    % output
    dx = [px_dot; py_dot; theta_dot];


end
