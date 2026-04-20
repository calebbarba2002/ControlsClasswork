# 3rd-party
import numpy as np
import matplotlib.pyplot as plt

# local
from case_studies import common, L_rodmass
from case_studies.L_rodmass.pid_controller import RodMassControllerPID


print("\n" + "="*60)
print("Part 3: PID Control")
print("="*60)

# ============================================
# Reference: ±20 deg square wave @ 0.1 Hz
# ============================================
reference = common.SignalGenerator(
    amplitude=20*np.pi/180,
    frequency=0.1
)

# ============================================
# PART 3.5: PD CONTROL (NO UNCERTAINTY)
# ============================================

print("\n--- PD Control (Nominal System) ---")

# System (no uncertainty)
system_pd = L_rodmass.Dynamics(alpha=0.0)

# ===== YOU WILL COMPUTE THESE ON EXAM =====
kp = 1.0   # <-- from part 3.3
kd = 0.1   # <-- from part 3.4
# =========================================

controller_pd = RodMassControllerPID(kp=kp, kd=kd, ki=0.0)

time_pd, x_pd, u_pd, r_pd, xhat_pd, *_ = common.run_simulation(
    system_pd,
    [reference],
    controller_pd,
    controller_input="measurement",
    t_final=20.0,
    dt=L_rodmass.params.ts
)

viz_pd = L_rodmass.Visualizer(time_pd, x_pd, u_pd, r_pd, xhat_pd)
viz_pd.plot()


# ============================================
# PART 3.6: PID CONTROL (WITH UNCERTAINTY)
# ============================================

print("\n--- PID Control with 10% Uncertainty ---")

# System with uncertainty
system_pid = L_rodmass.Dynamics(alpha=0.1)

# ===== YOU WILL TUNE THESE =====
kp = 1.0
kd = 0.1
ki = 0.5   # add integrator to remove steady-state error
# =========================================

controller_pid = RodMassControllerPID(kp=kp, kd=kd, ki=ki)

time_pid, x_pid, u_pid, r_pid, xhat_pid, *_ = common.run_simulation(
    system_pid,
    [reference],
    controller_pid,
    controller_input="measurement",
    t_final=20.0,
    dt=L_rodmass.params.ts
)

viz_pid = L_rodmass.Visualizer(time_pid, x_pid, u_pid, r_pid, xhat_pid)
viz_pid.plot()
viz_pid.animate()

print("\n" + "="*60)
print("PID Control Complete")
print("="*60)



# """
# Practice Final Exam - Part 3: PID Control
# Rod-Mass System

# This script demonstrates PID control with and without parameter uncertainty.
# - First run: PD control (no integrator needed) with nominal parameters
# - Second run: PID control with 10% parameter uncertainty
# """

# # 3rd-party
# import numpy as np
# import matplotlib.pyplot as plt

# # local (controlbook)
# from case_studies import common, L_rodmass


# print("\n" + "="*60)
# print("Part 3: PID Control")
# print("="*60)

# # Reference signal - 20 degree square wave at 0.1 Hz
# reference = common.SignalGenerator(amplitude=20*np.pi/180, frequency=0.1)

# # TODO: Implement the PID controller (or PD and then PID controller)
# # TODO: Instantiate system with nominal parameters and define the controller to use here (PD control for nominal case):


# # Run simulation
# time_pd, x_pd, u_pd, r_pd, xhat_pd, *_ = common.run_simulation(
#     system_pd,
#     [reference],
#     controller_pd,
#     controller_input="measurement",
#     t_final=20.0,
#     dt=L_rodmass.params.ts
# )


# # Visualize (plots only - animation in final section)
# viz_pd = L_rodmass.Visualizer(time_pd, x_pd, u_pd, r_pd, xhat_pd)
# viz_pd.plot()


# #=========================================================================
# # Part 3.6: PID Control with 10% Parameter Uncertainty
# #=========================================================================
# print("\n--- Part 3.6: PID Control with 10% Uncertainty ---")

# # TODO: Instantiate system with 10% parameter uncertainty and define the controller to use here (PID control for uncertain case):

# # Visualize
# viz_pid = L_rodmass.Visualizer(time_pid, x_pid, u_pid, r_pid, xhat_pid)
# viz_pid.plot()
# viz_pid.animate()

# print("\n" + "="*60)
# print("PID Control Complete")
# print("="*60)
