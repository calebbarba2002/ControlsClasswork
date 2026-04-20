"""
Rod-Mass System Parameters

Physical system: Point mass on massless rod with nonlinear spring
"""
import numpy as np
import control as cnt

##### Physical Parameters
m = 0.1              # mass of the point mass (kg)
ell = 0.25           # length of the rod (m)
b = 0.1              # damping coefficient (N⋅m⋅s)
g = 9.8              # gravity (m/s²)
k1 = 0.02            # linear spring coefficient
k2 = 0.01            # cubic spring coefficient (nonlinear term)

# TODO: add any additional parameters needed for simulation or control here ...
##### Part 2: Equilibrium + Linearization
##### Initial Conditions
theta0 = 0.0
thetadot0 = 0.0

##### Simulation Parameters
t0 = 0.0
tf = 5.0
ts = 0.01

##### Input Limits
tau_max = 3.0  # maximum torque (N⋅m)

##### Question Part 4: Linearized State-Space Model

# inertia
J = m * ell**2

# linearized stiffness
k_eq = k1 + m * g * ell

A = np.array([
    [0, 1],
    [-k_eq / J, -b / J]
])

B = np.array([
    [0],
    [1 / J]
])

Cm = np.array([[1, 0]])   # measure theta
Cr = Cm
D = np.zeros((1,1))

##### Equilibrium point
x_eq = np.array([0.0, 0.0])   # [theta, thetadot]
u_eq = np.array([0.0])        # tau_e = 0 at theta_e = 0


