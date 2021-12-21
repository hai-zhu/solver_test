function dx = double_integrator_dynamics_continuous(x, u, p)

    % second order dynamics model
    % 
    % state:   x, y, vx, vy
    % control: ax, ay
    % 
    % f:
    %	\dot( x ) == vx
    %	\dot( y ) == vy
    %   \dot( vx) == ax
    %   \dot( vy) == ay
    % 
    % (c) Hai Zhu, TU Delft, 2019, h.zhu@tudelft.nl
    %

    %% control inputs
    ax          =   u(1);
    ay          =   u(2);

    %% dynamics
    vx          =   x(3);
    vy          =   x(4);
    dvx         =   ax;
    dvy         =   ay;

    %% output
    dx = [vx; vy; dvx; dvy];


end
