# solver_test
Examples to test different optimization solvers.

![collision avoidance of a double-integrator robots](./sources/di_ca.mp4)

## Testing environment
* Ubuntu 18.04
* MATLAB R2019b

A valid licence is required if you want to test the Forces Pro solver.

## MATLAB test
* Open a MATLAB instance and navigate the code folder.
* Run the "setPath.m" script to add path to MATLAB.
* Run the following script to test different solvers:
    * yalmip_example.m
    * forces_pro_example.m
    * casadi_opti_example.m
    * casadi_shooting_example.m
    * casadi_collocation_example.m

## ACADOS test
If you want to test the acados solver, first navigate to the directory './matlab/mpc/acados/'. From the folder, open a terminal and run 'source env_set.sh'. Next open MATLAB from the terminal via the command 'matlab'. Then you can run the 'acados_example.m' script to test the acados solver. 
