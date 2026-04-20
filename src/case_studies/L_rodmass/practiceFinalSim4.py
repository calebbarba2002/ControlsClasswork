# 3rd-party
import numpy as np
import matplotlib.pyplot as plt

# local (controlbook)
from case_studies import common, L_rodmass

print("\n" + "="*60)
print("Part 4: State-Space Control with Observers")
print("="*60)

# Reference signal - 20 deg square wave
reference = common.SignalGenerator(amplitude=20*np.pi/180, frequency=0.1)

# =========================================================
# Part 4.2: FULL STATE FEEDBACK (NO OBSERVER)
# =========================================================
print("\n--- Part 4.2: State-Space Control (Full State) ---")

# Instantiate system (NO uncertainty here)
system_ssi = L_rodmass.Dynamics(alpha=0.0)

# Use your SSI controller (from ssi_controller.py)
controller_ssi = L_rodmass.ControllerSSI()

# Run simulation (uses TRUE STATE)
time_ssi, x_ssi, u_ssi, r_ssi, xhat_ssi, d_ssi, dhat_ssi = common.run_simulation(
    system_ssi,
    [reference],
    controller_ssi,
    controller_input="state",   # 🔥 IMPORTANT: full state
    t_final=20.0,
    dt=L_rodmass.params.ts
)

viz_ssi = L_rodmass.Visualizer(time_ssi, x_ssi, u_ssi, r_ssi, xhat_ssi)
viz_ssi.plot()


# =========================================================
# Part 4.5–4.6: OBSERVER + DISTURBANCE
# =========================================================
print("\n--- Parts 4.5 and 4.6: Observer with Input Disturbance ---")

# 10% uncertainty (as required)
system_dist = L_rodmass.Dynamics(alpha=0.1)

# disturbance (constant torque)
input_disturbance = np.array([0.5])

# controller with observer + disturbance observer
controller_dist = L_rodmass.ControllerSSIDO()

# Run simulation (uses MEASUREMENTS now)
time_dist, x_dist, u_dist, r_dist, xhat_dist, d_dist, dhat_dist = common.run_simulation(
    system_dist,
    [reference],
    controller_dist,
    controller_input="measurement",   # 🔥 IMPORTANT: observer mode
    input_disturbance=input_disturbance,
    t_final=20.0,
    dt=L_rodmass.params.ts
)

viz_dist = L_rodmass.Visualizer(
    time_dist, x_dist, u_dist, r_dist, xhat_dist, d_dist, dhat_dist
)
viz_dist.plot()


# =========================================================
# Part 4.7: LQR CONTROLLER
# =========================================================
print("\n--- Part 4.7: LQR Controller ---")

# same uncertain system
system_lqr = L_rodmass.Dynamics(alpha=0.1)

# LQR controller (your lqr_controller.py)
controller_lqr = L_rodmass.ControllerLQR()

time_lqr, x_lqr, u_lqr, r_lqr, xhat_lqr, d_lqr, dhat_lqr = common.run_simulation(
    system_lqr,
    [reference],
    controller_lqr,
    controller_input="measurement",
    input_disturbance=input_disturbance,
    t_final=20.0,
    dt=L_rodmass.params.ts
)

viz_lqr = L_rodmass.Visualizer(
    time_lqr, x_lqr, u_lqr, r_lqr, xhat_lqr, d_lqr, dhat_lqr
)
viz_lqr.plot()
viz_lqr.animate()

print("\n" + "="*60)
print("State-Space Control Complete")
print("="*60)



# """
# Practice Final Exam - Part 4: State-Space Control with Observers
# Rod-Mass System

# This script demonstrates:
# - Part 4.2: State-space control with integrator (observer-based)
# - Part 4.5: Observer-based control with input disturbance
# - Part 4.6: Observer estimation error analysis
# - Part 4.7: LQR controller for faster response
# """
# # 3rd-party
# import numpy as np
# import matplotlib.pyplot as plt

# # local (controlbook)
# from case_studies import common, L_rodmass


# print("\n" + "="*60)
# print("Part 4: State-Space Control with Observers")
# print("="*60)

# # Reference signal - 20 degree square wave at 0.1 Hz
# reference = common.SignalGenerator(amplitude=20*np.pi/180, frequency=0.1)

# #=========================================================================
# # Part 4.2: State-Space Control with Integrator (Full State Feedback)
# #=========================================================================
# print("\n--- Part 4.2: State-Space Control (Full State) ---")

# # TODO: Implement the state space integral controller
# # TODO: # Instantiate the system and state-space controller

# # Run simulation (no input disturbance in this section)
# time_ssi, x_ssi, u_ssi, r_ssi, xhat_ssi, d_ssi, dhat_ssi = common.run_simulation(
#     system_ssi,
#     [reference],
#     controller_ssi,
#     controller_input="state",
#     t_final=20.0,
#     dt=L_rodmass.params.ts
# )


# # Visualize (plots only - animation in final section)
# viz_ssi = L_rodmass.Visualizer(time_ssi, x_ssi, u_ssi, r_ssi, xhat_ssi)
# viz_ssi.plot()


# #=========================================================================
# # Parts 4.5 and 4.6: Observer-Based Control WITH Input Disturbance
# #=========================================================================
# print("\n--- Parts 4.5 and 4.6: Observer with Input Disturbance ---")

# # TODO: Implement the state space integral controller with disturbance observer
# # TODO: # Instantiate the system and controller with disturbance observer

# # Run simulation with disturbance
# time_dist, x_dist, u_dist, r_dist, xhat_dist, d_dist, dhat_dist = common.run_simulation(
#     system_dist,
#     [reference],
#     controller_dist,
#     controller_input="measurement",
#     input_disturbance=input_disturbance,
#     t_final=20.0,
#     dt=L_rodmass.params.ts
# )


# # Visualize with disturbance plots (plots only - animation in final section)
# viz_dist = L_rodmass.Visualizer(time_dist, x_dist, u_dist, r_dist, xhat_dist, d_dist, dhat_dist)
# viz_dist.plot()

# #=========================================================================
# # Part 4.7: LQR Controller for Faster Response
# #=========================================================================
# print("\n--- Part 4.7: LQR Controller ---")

# # TODO: Implement an LQR controller with integral control, and a disturbance observer. 
# # TODO: # Instantiate the system and lqr controller

# # Run simulation with disturbance
# time_lqr, x_lqr, u_lqr, r_lqr, xhat_lqr, d_lqr, dhat_lqr = common.run_simulation(
#     system_lqr,
#     [reference],
#     controller_lqr,
#     controller_input="measurement",
#     input_disturbance=input_disturbance,
#     t_final=20.0,
#     dt=L_rodmass.params.ts
# )


# # Visualize LQR results
# viz_lqr = L_rodmass.Visualizer(time_lqr, x_lqr, u_lqr, r_lqr, xhat_lqr, d_lqr, dhat_lqr)
# viz_lqr.plot()
# viz_lqr.animate()

# print("\n" + "="*60)
# print("State-Space Control Complete")
# print("="*60)
