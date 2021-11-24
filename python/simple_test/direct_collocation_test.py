#!/usr/bin/python
import casadi as cd
import numpy as np
from os import system
import time

recompile = True  


T = 1. # Time horizon
N = 40 # number of control intervals

x_des = 0
y_des = 0
z_des = 0

g = 9.81
kD_x = 0.25
kD_y = 0.33
k_vz = 1.227
k_phi = 1.126
k_theta = 1.1075
tau_vz = 0.3367
tau_phi = 0.2368
tau_theta = 0.2318

phi_c_lb = np.radians(-25)
theta_c_lb = np.radians(-25)
psidot_c_lb = np.radians(-25)
vz_c_lb = -2

phi_c_ub = np.radians(25)
theta_c_ub = np.radians(25)
psidot_c_ub = np.radians(25)
vz_c_ub = 2

# Declare model variables
px = cd.MX.sym('px')
py = cd.MX.sym('py')
pz = cd.MX.sym('pz')
vx = cd.MX.sym('vx')
vy = cd.MX.sym('vy')
vz = cd.MX.sym('vz')
phi = cd.MX.sym('phi')
theta = cd.MX.sym('theta')
psi = cd.MX.sym('psi')
x = cd.vertcat(px,py,pz,vx,vy,vz,phi,theta,psi)


phi_c = cd.MX.sym('phi_c')
theta_c = cd.MX.sym('theta_c')
vz_c = cd.MX.sym('vz_c')
psidot_c = cd.MX.sym('psidot_c')
u = cd.vertcat(phi_c, theta_c, vz_c, psidot_c)

# Model Equations
px_dot = vx
py_dot = vy
pz_dot = vz
vx_dot = (cd.tan(phi) * cd.sin(psi) / cd.cos(theta) + cd.tan(theta)*cd.cos(psi)) * g - kD_x * vx
vy_dot = (-cd.tan(phi) * cd.cos(psi) / cd.cos(theta) + cd.tan(theta)*cd.sin(psi)) * g - kD_y * vy
vz_dot = (k_vz * vz_c - vz) / tau_vz
phi_dot = (k_phi * phi_c - phi) / tau_phi
theta_dot = (k_theta * theta_c - theta) / tau_theta
psi_dot = psidot_c

xdot = cd.vertcat(px_dot, py_dot, pz_dot, vx_dot, vy_dot, vz_dot, phi_dot, theta_dot, psi_dot)

# Objective
L = px**2 + py**2 + pz**2 + phi_c**2 + .01*(theta_c**2 + vz_c**2 + psidot_c**2)

DT = T/N

f = cd.Function('f', [x, u], [xdot])
#f = cd.Function('f', [x, u], [xdot]).expand()
l = cd.Function('l', [x, u], [L])
#l = cd.Function('l', [x, u], [L]).expand()
XK = cd.MX.sym('XK', 9)
XK1 = cd.MX.sym('XK1', 9)
XDOTK = cd.MX.sym('XDOTK', 9)
XDOTK1 = cd.MX.sym('XDOTK1', 9)
UK = cd.MX.sym('UK',4)
UK1 = cd.MX.sym('UK1',4)

X_tc = 1./2 * (XK + XK1) + DT/8 * (XDOTK - XDOTK1)
print(X_tc)
f_x_tc = cd.Function('f_x_tc', [XK, XK1, XDOTK, XDOTK1], [X_tc], ['xk', 'xk1', 'xdk', 'xdk1'], ['xtc'])
#f_x_tc = cd.Function('f_x_tc', [XK, XK1, XDOTK, XDOTK1], [X_tc]).expand()

Xdot_tc = -3/(2*DT)*(XK - XK1) - 1./4 * (XDOTK + XDOTK1)
print(Xdot_tc)
f_xdot_tc = cd.Function('f_xdot_tc', [XK, XK1, XDOTK, XDOTK1], [Xdot_tc], ['xk', 'xk1', 'xdk', 'xdk1'], ['xdtc'])
#f_xdot_tc = cd.Function('f_xdot_tc', [XK, XK1, XDOTK, XDOTK1], [Xdot_tc]).expand()

U_tc = 1/2 * (UK + UK1)
f_u_tc = cd.Function('f_u_tc', [UK, UK1], [U_tc], ['uk', 'uk1'], ['utc'])
#f_u_tc = cd.Function('f_u_tc', [UK, UK1], [U_tc]).expand()


# NLP Forumlation
w = [] # decision variables
w0 = [] # initial guess
lbw = [] # lower bound on decision variables
ubw = [] # upper bound on decision variables
J = 0 # cost accumulator
g = [] # inequality constraint equations
lbg = [] # lower bound for g
ubg = [] # upper bound for g

Xk = cd.MX.sym('X0', 9)

# Set initial state
w += [Xk]
lbw += [1, 0, 0, 0, 0, 0, 0, 0, 0]
ubw += [1, 0, 0, 0, 0, 0, 0, 0, 0]
w0 += [1, 0, 0, 0, 0, 0, 0, 0, 0]

Uk = cd.MX.sym('U0', 4)
w += [Uk]
lbw += [phi_c_lb, theta_c_lb, vz_c_lb, psidot_c_lb]
ubw += [phi_c_ub, theta_c_ub, vz_c_ub, psidot_c_ub]
w0 += [0,0,0,0]

for k in range(N):

    J=J+l(Xk, Uk)

    Xk_next = cd.MX.sym('X_' + str(k+1), 9)
    w += [Xk_next]
    lbw += [-cd.inf, -cd.inf, -cd.inf, -cd.inf, -cd.inf, -cd.inf, -cd.inf, -cd.inf, -cd.inf]
    ubw += [cd.inf, cd.inf, cd.inf, cd.inf, cd.inf, cd.inf, cd.inf, cd.inf, cd.inf]
    w0 += [0, 0, 0, 0, 0, 0, 0, 0, 0]

    Uk_next = cd.MX.sym('U_' + str(k+1), 4)
    w += [Uk_next]
    lbw += [phi_c_lb, theta_c_lb, vz_c_lb, psidot_c_lb]
    ubw += [phi_c_ub, theta_c_ub, vz_c_ub, psidot_c_ub]
    w0 += [0,0,0,0]

    Xdot_k = f(Xk, Uk)
    Xdot_k1 = f(Xk_next, Uk_next)
    x_col = f_x_tc(xk=Xk, xk1=Xk_next, xdk=Xdot_k, xdk1=Xdot_k1)['xtc']
    u_col = f_u_tc(uk=Uk, uk1=Uk_next)['utc']
    xdot_col = f_xdot_tc(xk=Xk, xk1=Xk_next, xdk=Xdot_k, xdk1=Xdot_k1)['xdtc']

    g += [xdot_col - f(x_col, u_col)]
    lbg += [0, 0, 0, 0, 0, 0, 0, 0, 0]
    ubg += [0, 0, 0, 0, 0, 0, 0, 0, 0]

    Xk = Xk_next
    Uk = Uk_next

prob = {'f': J, 'x': cd.vertcat(*w), 'g': cd.vertcat(*g)}
solver = cd.nlpsol('solver', 'ipopt', prob)
sol = solver(x0=w0, lbx=lbw, ubx=ubw, lbg=lbg, ubg=ubg)
w_opt = sol['x']
xlist = []
ylist = []
zlist = []
vxlist = []
ulist = []
for k in range(N):
    x = w_opt[k*13]
    y = w_opt[k*13 + 1]
    z = w_opt[k*13 + 2]
    vx = w_opt[k*13 + 3]
    pitch = w_opt[k*13 + 10]

    xlist.append(x)
    ylist.append(y)
    zlist.append(z)
    vxlist.append(vx)
    ulist.append(pitch)

# print('x: ', xlist)
# print('y: ', ylist)
# print('z: ', zlist)
# print('vx: ', vxlist)
# print('pitch: ', ulist)

# Codegen tests
if recompile:
    solver.generate_dependencies('nlp.c')
    print('Compiling...')
    system('gcc -fPIC -shared nlp.c -o nlp.so')
    print('Done Compiling!')

solver_comp = cd.nlpsol('solver', 'ipopt', './nlp.so', {'print_time': 0, 'ipopt.print_level' : 0})
a = time.time()
for ix in range(100):
    solver_comp(x0=w0,lbx=lbw,ubx=ubw,lbg=lbg,ubg=ubg)
print('Total Time: %f' % (time.time() - a))

