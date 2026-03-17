import numpy as np
# BEGIN SOLUTION

##### Chapter 2
# Physical parameters
m1 = 0.35 # mass of block (kg)
m2 = 2.0 # mass of beam (kg)
length = 0.5 # length of beam (m)
g = 9.8 # gravity (m/s^2)

##### Chapter 3
# Initial conditions
z0 = length / 2 # block position (m)
theta0 = np.radians(0.0) # beam angle (rad)
zdot0 = 0.0 # block velocity (m/s)
thetadot0 = 0.0 # angular velocity (rad/s)

# Simulation parameters
t0 = 0.0 # start time
tf = 50.0 # end time
ts = 0.01 # integration time step

##### Chapter 4
# Linearization/equilibrium point
z_eq = length / 2.0
x_eq = np.array([z_eq, 0, 0, 0])
u_eq = np.array([m1 * g * z_eq / length + m2 * g / 2.0])

##### Chapter 5
# Transfer function numerator and denominator
tf_inner_num = [length / (m2 * length**2 / 3.0 + m1 * z_eq**2)]
tf_inner_den = [1, 0, 0]
tf_outer_num = [-g]
tf_outer_den = [1, 0, 0]

##### Chapter 6 / 11 # TODO: decide which chapter to put this in
# State space
inertia_eq = m2 * length**2 / 3.0 + m1 * z_eq**2
A = np.array(
    [
        [0, 0, 1, 0],
        [0, 0, 0, 1],
        [0, -g, 0, 0],
        [-m1 * g / inertia_eq, 0, 0, 0],
    ]
)
B = np.array([[0], [0], [0], [length / inertia_eq]])
Cm = np.eye(2, 4) # measure z and theta
Cr = np.eye(1, 4) # only command z
D = np.zeros((2, 1))

##### Chapter 8
force_max = 15.0 # max force (N)
# END SOLUTION