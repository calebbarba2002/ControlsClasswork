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

# === LATERAL CONTROL DESIGN === Lab H.8

# Roll loop (inner)
tr_phi = 0.25
zeta_phi = 0.707
wn_phi = 2.2 / tr_phi

kp_phi = wn_phi**2 * J1x
kd_phi = 2 * zeta_phi * wn_phi * J1x

# Yaw loop (outer)
M = 8
tr_psi = M * tr_phi
zeta_psi = 0.8
wn_psi = 2.2 / tr_psi

# yaw dynamics coefficient
b_psi = ellT / (m1 * ell1 + m2 * ell2)

kp_psi = wn_psi**2 / b_psi
kd_psi = 2 * zeta_psi * wn_psi / b_psi

print(f"{kp_phi = :.5f}, {kd_phi = :.5f}")
print(f"{kp_psi = :.5f}, {kd_psi = :.5f}")


Fe = (m1 * ell1 + m2 * ell2) * g / ellT
sigma = 0.05

