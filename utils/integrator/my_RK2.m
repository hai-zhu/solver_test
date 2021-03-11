function x_next = my_RK2(x, u, f, h, p)

    % perform one step explicit RK2 integrator for nonlinear system \dot{x} = f(x,u,p)
    % inputs:
    %   x: current state
    %   u: current control input
    %   f: nonlinear dynamics function handle
    %   h: step size (sampling time)
    %   p: passing parameters (to f)
    % outputs;
    %   x_next: next step state after integration
    
    k1 = h * feval(f, x, u, p);
    k2 = h * feval(f, x+k1/2, u, p);
    x_next = x + k2;

end
