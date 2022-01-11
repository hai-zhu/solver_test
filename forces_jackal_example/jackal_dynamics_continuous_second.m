function dx = jackal_dynamics_continuous_second(x, u, p)
    
    % state: x = [px, py, theta, vel, omega]
    % control: u = [acc, omega_acc]
    px = x(1);
    py = x(2);
    theta = x(3);
    vel = x(4);
    omega = x(5);
    acc = u(1);
    omega_acc = u(2);
    % dynamics
    px_dot = vel * cos(theta);
    py_dot = vel * sin(theta);
    theta_dot = omega;
    vel_dot = acc;
    omega_dot = omega_acc;
    % output
    dx = [px_dot; py_dot; theta_dot; vel_dot; omega_dot];


end
