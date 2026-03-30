# # 3rd-party
# import numpy as np
# import matplotlib.pyplot as plt
# # local (controlbook)
# from case_studies import common, D_mass

# # Part (a): alpha=0 for exact parameters (no uncertainty)
# mass = D_mass.Dynamics(alpha=0.0)

# # Part (c): use observer-based SSI controller
# controller = D_mass.ControllerSSIO()

# z_ref = common.SignalGenerator(amplitude=0.5, frequency=0.04)

# # Part (e): input disturbance of 0.25
# d_force = np.array([0.25])

# time, x_hist, u_hist, r_hist, xhat_hist, d_hist, *_ = common.run_simulation(
#     mass,
#     [z_ref],
#     controller,
#     controller_input="measurement",  # changed from "state" — now uses observer
#     input_disturbance=d_force,
#     t_final=50,
#     dt=D_mass.params.ts,
# )

# viz = D_mass.Visualizer(time, x_hist, u_hist, r_hist, xhat_hist, d_hist)
# viz.plot()

# # Part (d): plot true state vs estimated state on same graph
# fig, axes = plt.subplots(2, 1, figsize=(10, 6))

# axes[0].plot(time, x_hist[:, 0], label='z (true)', linewidth=2)
# axes[0].plot(time, xhat_hist[:, 0], '--', label='ẑ (estimated)', linewidth=2)
# axes[0].set_ylabel('Position z (m)')
# axes[0].legend()
# axes[0].grid(True)

# axes[1].plot(time, x_hist[:, 1], label='ż (true)', linewidth=2)
# axes[1].plot(time, xhat_hist[:, 1], '--', label='ż̂ (estimated)', linewidth=2)
# axes[1].set_ylabel('Velocity ż (m/s)')
# axes[1].set_xlabel('Time (s)')
# axes[1].legend()
# axes[1].grid(True)

# plt.suptitle('True State vs Observer Estimated State')
# plt.tight_layout()
# plt.show()

# 3rd-party
import numpy as np

# local (controlbook)
from case_studies import common, D_mass

mass = D_mass.Dynamics()
controller = D_mass.ControllerSSIO()
z_ref = common.SignalGenerator(amplitude=0.5, frequency=0.04)
d_force = np.array([0.25]) # TODO: use disturbance or not?

# time, x_hist, u_hist, r_hist, xhat_hist, *_ = common.run_simulation(
time, x_hist, u_hist, r_hist, xhat_hist, d_hist, *_ = common.run_simulation(
    mass,
    [z_ref],
    controller,
    controller_input="measurement",
    input_disturbance=d_force,
    t_final=50,
    dt=D_mass.params.ts,
)

# viz = D_mass.Visualizer(time, x_hist, u_hist, r_hist, xhat_hist)
viz = D_mass.Visualizer(time, x_hist, u_hist, r_hist, xhat_hist, d_hist)
viz.plot()
# viz.animate()