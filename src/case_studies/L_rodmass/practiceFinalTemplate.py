"""
FINAL EXAM TEMPLATE - NONLINEAR SYSTEM → LINEAR MODEL

HOW TO USE THIS:
1. Replace dynamics equation (f)
2. Replace equilibrium equation
3. Replace derivatives
DONE.
"""

import numpy as np
import control as cnt
import matplotlib.pyplot as plt

# ==========================================
# PARAMETERS (EDIT THESE)
# ==========================================
m = 0.1
l = 0.25
b = 0.1
g = 9.8

# add/remove depending on system
k1 = 0.02
k2 = 0.01

# ==========================================
# DYNAMICS (EDIT THIS PART ONLY)
# ==========================================
def dynamics(x, u):
    """
    x = [q, qdot]
    u = input
    """

    q = x[0]
    qdot = x[1]

    # ======================================
    # EDIT THIS EQUATION FOR NEW SYSTEM
    # ======================================
    qddot = (1/(m*l**2)) * (
        u
        - b*qdot
        - k1*q
        - k2*q**3
        - m*g*l*np.sin(q)
    )

    return np.array([qdot, qddot])


# ==========================================
# STEP 1: EQUILIBRIUM
# ==========================================
q_e = 0.0   # CHANGE THIS ON EXAM

# ======================================
# EDIT THIS EQUATION FOR NEW SYSTEM
# ======================================
u_e = (
    k1*q_e
    + k2*q_e**3
    + m*g*l*np.sin(q_e)
)

print("Equilibrium input u_e =", u_e)


# ==========================================
# STEP 2: VERIFY EQUILIBRIUM (SIMULATION)
# ==========================================
dt = 0.01
T = 5
time = np.arange(0, T, dt)

x = np.array([q_e, 0.0])  # start at equilibrium

x_hist = []

for t in time:
    xdot = dynamics(x, u_e)
    x = x + dt * xdot
    x_hist.append(x)

x_hist = np.array(x_hist)

# plot to verify flat line
plt.plot(time, x_hist[:,0])
plt.title("Equilibrium Check (should be flat)")
plt.xlabel("Time")
plt.ylabel("q")
plt.grid()
plt.show()


# ==========================================
# STEP 3: LINEARIZATION
# ==========================================

# ======================================
# COMPUTE THESE DERIVATIVES MANUALLY
# ======================================

df_dq = (1/(m*l**2)) * (
    -k1
    - 3*k2*q_e**2
    - m*g*l*np.cos(q_e)
)

df_dqdot = -b/(m*l**2)
df_du = 1/(m*l**2)

A = np.array([
    [0, 1],
    [df_dq, df_dqdot]
])

B = np.array([
    [0],
    [df_du]
])

C = np.array([[1, 0]])
D = np.array([[0]])

print("\nA =", A)
print("B =", B)


# ==========================================
# STEP 4: TRANSFER FUNCTION
# ==========================================
sys_ss = cnt.ss(A, B, C, D)
sys_tf = cnt.ss2tf(sys_ss)

print("\nTransfer Function:")
print(sys_tf)


# ==========================================
# STEP 5: STATE SPACE MODEL
# ==========================================
print("\nState Space Model:")
print("x_dot = A x + B u")
print("y = C x")
