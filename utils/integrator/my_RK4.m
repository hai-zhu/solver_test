function x_next = my_RK4(x, u, f, h, p)

    % perform one step explicit RK4 integrator for nonlinear system \dot{x} = f(x,u,p)
    % inputs:
    %   x: current state
    %   u: current control input
    %   f: nonlinear dynamics function handle
    %   h: step size (sampling time)
    %   p: passing parameters (to f)
    % outputs;
    %   x_next: next step state after integration
    
    k1 = h*feval(f, x, u, p);
    k2 = h*feval(f, x+k1/2, u+h/2, p);
    k3 = h*feval(f, x+k2/2, u+h/2, p);
    k4 = h*feval(f, x+k3, u+h); 
    x_next = x + (k1 + 2*k2 + 2*k3 + k4)/6;

end
