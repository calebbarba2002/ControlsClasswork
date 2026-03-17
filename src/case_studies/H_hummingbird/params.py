import numpy as np

g = 9.81
ell1 = 0.247
ell2 = -0.039
ell3x =  -0.007
ell3y = -0.007
ell3z = 0.018
ellT = 0.355
d = 0.12
m1 = 0.108862
J1x = 0.000189
J1y = 0.001953
J1z = 0.001894
m2 = 0.4717
J2x = 0.00231
J2y = 0.003274
J2z = 0.003416
m3 = 0.1905
J3x = 0.0002222
J3y = 0.0001956
J3z = 0.000027


phi0 = 0.0
theta0 = 0.0
psi0 = 0.0
phidot0 = 0.0
thetadot0 = 0.0
psidot0 = 0.0
ts = 0.01
beta = 0.001


##### Chapter 4
# mixing matrices (see end of Chapter 4 in lab manual)
# mixing is a UAV term for taking body forces/torques to individual motor forces
unmixer = np.array([[1.0, 1.0], [d, -d]])  # [F, tau] = unmixer @ [fl, fr]
mixer = np.linalg.inv(unmixer)  # [fl, fr] = mixer @ [F, tau]


# === PD DESIGN PARAMETERS ===
tr_theta = 1.0
zeta_theta = 0.707

# natural frequency
wn_theta = 2.2 / tr_theta

# equivalent inertia
J_eq = J2y + m1 * ell1**2 + m2 * ell2**2

# system gain
b_theta = ellT / J_eq

# PD gains
kp_theta = wn_theta**2 / b_theta
kd_theta = 2 * zeta_theta * wn_theta / b_theta

print(f"{kp_theta = :.3f}, {kd_theta = :.3f}")


