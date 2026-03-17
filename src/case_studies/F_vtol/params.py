import numpy as np

# BEGIN SOLUTION
##### Chapter 2
# Physical parameters
mc = 1.0 # mass of center body (kg)
mr = 0.25 # mass of single rotor/motor (kg)
Jc = 0.0042 # inertia of center body (kg*m^2)
d = 0.3 # distance from center of body to rotor (m)
mu = 0.1 # friction coefficient (kg/s)
g = 9.81 # gravity (m/s^2)

##### Chapter 3
# Initial conditions
z0 = 0.0 # lateral position (m)
h0 = 0.0 # altitude/height (m)
theta0 = np.radians(0.0) # roll angle (rad)
zdot0 = 0.0 # lateral velocity (m/s)
hdot0 = 0.0 # vertical velocity (m/s)
thetadot0 = 0.0 # angular velocity (rad/s)
target0 = z0

# Simulation parameters
t0 = 0.0 # start time
tf = 50.0 # end time
ts = 0.01 # integration time step

##### Chapter 4
# mixing matrices (see HW F.4 in the book)
# mixing is a UAV term for taking body forces/torques to individual motor forces
unmixer = np.array([[1.0, 1.0], [d, -d]]) # [F, tau] = unmixer @ [fr, fl]
mixer = np.linalg.inv(unmixer) # [fr, fl] = mixer @ [F, tau]

# Linearization/equilibrium point
mass = mc + 2 * mr
F_eq = mass * g
tau_eq = 0.0
x_lon_eq = np.zeros(2)
x_lat_eq = np.zeros(4)
x_eq = np.zeros(6)
# u_lon_eq = np.array([F_eq])
# u_lat_eq = np.zeros(1)
u_FT_eq = np.array([F_eq, tau_eq]) # [force, torque]
u_eq = mixer @ u_FT_eq # [fr, fl]
# u_eq = np.array([F_eq / 2, F_eq / 2]) # [fr, fl]
# u_eq = mixer @ np.array([F_eq, 0.0]) # [fr, fl] # TODO: decide which u_eq to use

##### Chapter 5
# Transfer function numerator and denominator
inertia = Jc + 2 * mr * d**2
tf_lon_num = [1 / mass]
tf_lon_den = [1, 0, 0]
tf_lat_inner_num = [1 / inertia]
tf_lat_inner_den = [1, 0, 0]
tf_lat_outer_num = [-F_eq / mass]
tf_lat_outer_den = [1, mu / mass, 0]

##### Chapter 6 / 11 # TODO: decide which chapter to put this in
# State space
A_lon = np.eye(2, k=1)
B_lon = np.array([[0, 1 / mass]]).T
Cm_lon = np.eye(1, 2) # measure h
Cr_lon = Cm_lon
D_lon = np.zeros((1, 1))

A_lat = np.array(
    [
        [0, 0, 1, 0],
        [0, 0, 0, 1],
        [0, -g, -mu / mass, 0],
        [0, 0, 0, 0],
    ]
)
B_lat = np.array([[0, 0, 0, 1 / inertia]]).T
Cm_lat = np.eye(2, 4) # measure z and theta
Cr_lat = np.eye(1, 4) # only command z
D_lat = np.zeros((2, 1))

# combined
# TODO: choose a version

# A = np.eye(6, k=3)
# A[3, 2:4] = [-g, -mu / mass]

A = np.block(
    [
        [0, 0, 0, 1, 0, 0],
        [0, 0, 0, 0, 1, 0],
        [0, 0, 0, 0, 0, 1],
        # [np.zeros((3, 3)), np.eye(3)],
        # [np.eye(3, 6, k=3)], # top 3 rows are [zeros, I]
        [0, 0, -g, -mu / mass, 0, 0], # zddot row
        # [np.zeros((2, 6))], # last 2 rows are zeros
        [0, 0, 0, 0, 0, 0],
        [0, 0, 0, 0, 0, 0],
    ]
)

# B if we use [F, tau] as input
B_FT = np.block(
    [
        [0, 0],
        [0, 0],
        [0, 0],
        [0, 0],
        # [np.zeros((4, 2))],
        [1 / mass, 0],
        [0, 1 / inertia],
    ]
)
# since: u = [fr, fl] and u_FT = [F, tau]
# and: u_FT = unmixer @ u
# we get: B_FT @ u_FT = B_FT @ unmixer @ u
# therefore: B = B_FT @ unmixer
B = B_FT @ unmixer
Cm = np.eye(3, 6) # measure z, h, and theta
Cr = np.eye(2, 6) # only command z and h
D = np.zeros((3, 2))

##### Chapter 8
fr_max = 10.0 # max force of single motor (N)
force_max = 2 * fr_max # max force of both motors (N)
# END SOLUTION