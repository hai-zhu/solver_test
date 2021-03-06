# solver_test
Examples to test different optimization solvers.

![collision avoidance of a double-integrator robots](./sources/di_ca.gif)

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

If you find this code useful in your research then please cite:
```
@article{Zhu2019RAL,
    title = {{Chance-Constrained Collision Avoidance for MAVs in Dynamic Environments}},
    author = {Zhu, Hai and Alonso-Mora, Javier},
    journal = {IEEE Robotics and Automation Letters},
    number = {2},
    volume = {4},
    pages = {776--783},
    publisher = {IEEE},
    year = {2019}
}
```
```
@inproceedings{Zhu2020ICRA,
    title = {{Robust Vision-based Obstacle Avoidance for Micro Aerial Vehicles in Dynamic Environments}},
    author = {Lin, Jiahao and Zhu, Hai and Alonso-Mora, Javier},
    booktitle = {2020 IEEE International Conference on Robotics and Automation (ICRA)},
    pages = {2682--2688},
    publisher = {IEEE},
    year = {2020}
}
```
