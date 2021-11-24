def double_integrator_dynamics_continuous(x, u, p):

    # second order dynamics model
    # 
    #  state:   x, y, vx, vy
    #  control: ax, ay
    #  
    #  f:
    #	\dot( x ) == vx
    #	\dot( y ) == vy
    #   \dot( vx) == ax
    #   \dot( vy) == ay
    # 
    # (c) Hai Zhu, TU Delft, 2021, h.zhu@tudelft.nl
    #

    ## control inputs
    ax          =   u[0]
    ay          =   u[1]

    ## dynamics
    vx          =   x[2]
    vy          =   x[3]
    dvx         =   ax
    dvy         =   ay

    ## output
    dx = [vx, vy, dvx, dvy]

    return dx 

