# """
# Practice Final Exam - Part 2: Equilibrium Verification
# Rod-Mass System

# This script verifies that the equilibrium torque holds the system at rest.
# Students implement dynamics in dynamics.py, then run this to verify.
# """
# # 3rd-party
# import numpy as np
# import matplotlib.pyplot as plt

# # local (controlbook)
# from case_studies import common, L_rodmass



# # Instantiate system and controller
# system = L_rodmass.Dynamics(alpha=0.0)  # no parameter uncertainty
# reference = common.SignalGenerator(amplitude=0.0)  # zero reference

# # TODO: define the controller to use here (if you defined an equilibrium 
# # controller, use it here to verify equilibrium): 

# # controller = 

# # Run simulation
# time, x_hist, u_hist, r_hist, xhat_hist, *_ = common.run_simulation(
#     system,
#     [reference],
#     controller,
#     controller_input="measurement",
#     t_final=5.0,
#     dt=L_rodmass.params.ts
# )

# # Print verification results
# # you can print values here that you need for the final exam. 

# # Visualize results
# viz = L_rodmass.Visualizer(time, x_hist, u_hist, r_hist, xhat_hist)
# viz.plot()
# viz.animate()

"""
Practice Final Exam - Part 2: Equilibrium + Linearization
Rod-Mass System

This script:
1) Computes equilibrium torque (tau_e)
2) Verifies equilibrium via simulation
3) Linearizes the system (A, B)
4) Computes transfer function
5) Reports state-space model
"""

# 3rd-party
import numpy as np
import matplotlib.pyplot as plt
import control as cnt

# local (controlbook)
from case_studies import common, L_rodmass


# ==========================================
# SYSTEM SETUP
# ==========================================
system = L_rodmass.Dynamics(alpha=0.0)  # no uncertainty
reference = common.SignalGenerator(amplitude=0.0)  # zero reference

P = L_rodmass.params


# ==========================================
# PART 2.1: EQUILIBRIUM TORQUE
# ==========================================
theta_e = 0.0  # equilibrium angle (CHANGE THIS ON EXAM)

# equilibrium torque from EOM (set theta_dot = 0, theta_ddot = 0)
tau_e = (
    P.k1 * theta_e
    + P.k2 * theta_e**3
    + P.m * P.g * P.ell * np.sin(theta_e)
)

print("====================================")
print("Equilibrium torque tau_e =", tau_e)
print("====================================")


# ==========================================
# PART 2.2: EQUILIBRIUM CONTROLLER
# ==========================================
class EquilibriumController:
    """
    Applies constant torque tau_e
    Used to verify equilibrium is correct
    """
    def update_with_measurement(self, r, y):
        u = np.array([tau_e])

        xhat = np.zeros(2)  # dummy estimate (same size as state)

        return u, xhat


controller = EquilibriumController()


# ==========================================
# RUN SIMULATION
# ==========================================
time, x_hist, u_hist, r_hist, xhat_hist, *_ = common.run_simulation(
    system,
    [reference],
    controller,
    controller_input="measurement",
    t_final=5.0,
    dt=P.ts
)


# ==========================================
# PART 2.3: LINEARIZATION
# ==========================================
A = np.array([
    [0, 1],
    [
        (1/(P.m * P.ell**2)) * (
            -P.k1
            - 3 * P.k2 * theta_e**2
            - P.m * P.g * P.ell * np.cos(theta_e)
        ),
        -P.b / (P.m * P.ell**2)
    ]
])

B = np.array([
    [0],
    [1/(P.m * P.ell**2)]
])

C = np.array([[1, 0]])
D = np.array([[0]])

print("\n====================================")
print("Linearized State-Space Model")
print("A =\n", A)
print("B =\n", B)
print("====================================")


# ==========================================
# PART 2.4: TRANSFER FUNCTION
# ==========================================
sys_ss = cnt.ss(A, B, C, D)
sys_tf = cnt.ss2tf(sys_ss)

print("\n====================================")
print("Transfer Function:")
print(sys_tf)
print("====================================")


# ==========================================
# PART 2.5: STATE-SPACE MODEL
# ==========================================
print("\n====================================")
print("State-Space Form:")
print("x_dot = A x + B u")
print("y = C x")
print("====================================")


# ==========================================
# VISUALIZATION
# ==========================================
viz = L_rodmass.Visualizer(time, x_hist, u_hist, r_hist, xhat_hist)
viz.plot()
viz.animate()


# ==========================================
# EXTRA CHECK (optional for exam confidence)
# ==========================================
plt.figure()
plt.plot(time, x_hist[:, 0])
plt.title("Theta vs Time (Should stay constant if equilibrium is correct)")
plt.xlabel("Time (s)")
plt.ylabel("Theta (rad)")
plt.grid()
plt.show()
