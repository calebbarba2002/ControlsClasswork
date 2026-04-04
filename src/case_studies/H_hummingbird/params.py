import numpy as np

g = 9.81
ell1 = 0.247
ell2 = -0.039
ell3x = -0.007
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
unmixer = np.array([[1.0, 1.0], [d, -d]])
mixer = np.linalg.inv(unmixer)

# === LONGITUDINAL PD GAINS ===
tr_theta = 1.0
zeta_theta = 0.707
wn_theta = 2.2 / tr_theta
J_eq = J2y + m1 * ell1**2 + m2 * ell2**2
b_theta = ellT / J_eq

kp_theta = 0.35
kd_theta = 0.20

print(f"{kp_theta = :.3f}, {kd_theta = :.3f}")

# === LATERAL CONTROL GAINS ===
# Roll loop (inner)
tr_phi = 0.25
zeta_phi = 0.707
wn_phi = 2.2 / tr_phi

kp_phi = 0.02      # reduced from 0.03
kd_phi = 0.008

# Yaw loop (outer)
M = 8
tr_psi = M * tr_phi
zeta_psi = 0.8
wn_psi = 2.2 / tr_psi

b_psi = ellT / (m1 * ell1 + m2 * ell2)

kp_psi = 0.17      # reduced from 0.16
kd_psi = 0.28      # reduced from 0.17

print(f"{kp_phi = :.5f}, {kd_phi = :.5f}")
print(f"{kp_psi = :.5f}, {kd_psi = :.5f}")

Fe = (m1 * ell1 + m2 * ell2) * g / ellT
sigma = 0.05



# import numpy as np

# g = 9.81
# ell1 = 0.247
# ell2 = -0.039
# ell3x =  -0.007
# ell3y = -0.007
# ell3z = 0.018
# ellT = 0.355
# d = 0.12
# m1 = 0.108862
# J1x = 0.000189
# J1y = 0.001953
# J1z = 0.001894
# m2 = 0.4717
# J2x = 0.00231
# J2y = 0.003274
# J2z = 0.003416
# m3 = 0.1905
# J3x = 0.0002222
# J3y = 0.0001956
# J3z = 0.000027


# phi0 = 0.0
# theta0 = 0.0
# psi0 = 0.0
# phidot0 = 0.0
# thetadot0 = 0.0
# psidot0 = 0.0
# ts = 0.01
# beta = 0.001


# ##### Chapter 4
# # mixing matrices (see end of Chapter 4 in lab manual)
# # mixing is a UAV term for taking body forces/torques to individual motor forces
# unmixer = np.array([[1.0, 1.0], [d, -d]])  # [F, tau] = unmixer @ [fl, fr]
# mixer = np.linalg.inv(unmixer)  # [fl, fr] = mixer @ [F, tau]


# # === PD DESIGN PARAMETERS ===
# tr_theta = 1.0
# zeta_theta = 0.707

# # natural frequency
# wn_theta = 2.2 / tr_theta

# # equivalent inertia
# J_eq = J2y + m1 * ell1**2 + m2 * ell2**2

# # system gain
# b_theta = ellT / J_eq

# # PD gains
# kp_theta = 0.22    # was ~0.145
# kd_theta = 0.15    # was ~0.093


# print(f"{kp_theta = :.3f}, {kd_theta = :.3f}")

# # === LATERAL CONTROL DESIGN === Lab H.8

# # Roll loop (inner)
# tr_phi = 0.25
# zeta_phi = 0.707
# wn_phi = 2.2 / tr_phi

# kp_phi = 0.030      # was ~0.01464
# kd_phi = 0.008     # was ~0.00235


# # Yaw loop (outer)
# M = 8
# tr_psi = M * tr_phi
# zeta_psi = 0.8
# wn_psi = 2.2 / tr_psi

# # yaw dynamics coefficient
# b_psi = ellT / (m1 * ell1 + m2 * ell2)

# kp_psi = 0.16      # was ~0.02895
# kd_psi = 0.17      # was ~0.04210



# print(f"{kp_phi = :.5f}, {kd_phi = :.5f}")
# print(f"{kp_psi = :.5f}, {kd_psi = :.5f}")


# Fe = (m1 * ell1 + m2 * ell2) * g / ellT
# sigma = 0.05

