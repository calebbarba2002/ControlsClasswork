import numpy as np

# physical parameters of the system
m = 5.0     # mass (kg)
k = 3.0     # spring constant (N/m)
b = 0.5     # damping coefficient (N s/m)

# simulation parameters
ts = 0.01   # simulation timestep
Ts = ts 

# initial conditions
z0 = 0.0
zdot0 = 0.0

# actuator limits
force_max = 5.0 #N

# transfer function of the plant
# G(s) = 1 / (m s^2 + b s + k)
tf_num = np.array([1.0 / m])
tf_den = np.array([1.0, b / m, k / m])
