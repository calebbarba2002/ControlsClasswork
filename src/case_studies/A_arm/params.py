import numpy as np

##### Chapter 2
# Physical parameters
m = 0.5  # mass of the arm (kg)
ell = 0.3  # length of the arm (m)
g = 9.8  # gravity (m/s**2)
b = 0.01  # damping coefficient (Nms)

##### Chapter 3
# Initial Conditions
theta0 = np.radians(0.0)  # (rad)
thetadot0 = 0.0  # (rad/s)

# Simulation parameters
t0 = 0.0  # start time
tf = 50.0  # end time
ts = 0.01  # integration time step

##### Chapter 4
# Linearization/equilibrium point
tau_eq = m * g * ell / 2
x_eq = np.zeros(2)
u_eq = np.array([tau_eq])

##### Chapter 5
# Transfer function numerator and denominator
inertia = m * ell**2 / 3
tf_num = [1 / inertia]
tf_den = [1, b / inertia, 0]

##### Chapter 6 / 11-14
# State space
A = np.array([[0, 1], [0, -b / inertia]])
B = np.array([[0, 1 / inertia]]).T
Cm = np.eye(1, 2)  # measure theta
Cr = Cm
D = np.zeros((1, 1))

##### Chapter 8
tau_max = 1.0  # Max torque, N-m
