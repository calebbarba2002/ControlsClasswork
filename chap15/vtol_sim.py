# 3rd-party
import numpy as np
import matplotlib.pyplot as plt
# local (controlbook)
from case_studies import common, F_vtol

P = F_vtol.params

# Part (a): alpha=0, no uncertainty
vtol = F_vtol.Dynamics(alpha=0.0, F_wind=0.0)

# Part (c): observer-based controller
controller = F_vtol.ControllerLQRIDO()

z_ref = common.SignalGenerator(amplitude=3.0, frequency=0.04, y_offset=5.0)
h_ref = common.SignalGenerator(amplitude=2.0, frequency=0.03, y_offset=5.0)
refs = [z_ref, h_ref]

# Part (e): input force disturbance=1.0, torque disturbance=0.1
d = P.mixer @ np.array([1.0 / P.mass, 0.1])

time, x_hist, u_hist, r_hist, xhat_hist, d_hist, *_ = common.run_simulation(
    vtol,
    refs,
    controller,
    controller_input="measurement",
    input_disturbance=d,
    t_final=60,
    dt=P.ts,
)

viz = F_vtol.Visualizer(
    time, x_hist, u_hist, r_hist, xhat_hist, d_hist, convert_to_F_tau=False
)
viz.plot()

# Part (d): true state vs estimated state
state_labels  = ['z (m)', 'h (m)', 'θ (rad)', 'ż (m/s)', 'ḣ (m/s)', 'θ̇ (rad/s)']
true_labels   = ['z', 'h', 'θ', 'ż', 'ḣ', 'θ̇']
est_labels    = ['ẑ', 'ĥ', 'θ̂', 'ẑ̇', 'ĥ̇', 'θ̂̇']

fig, axes = plt.subplots(6, 1, figsize=(10, 14))
for i in range(6):
    axes[i].plot(time, x_hist[:, i], label=true_labels[i], linewidth=2)
    axes[i].plot(time, xhat_hist[:, i], '--', label=est_labels[i], linewidth=2)
    axes[i].set_ylabel(state_labels[i])
    axes[i].legend()
    axes[i].grid(True)

axes[-1].set_xlabel('Time (s)')
plt.suptitle('True State vs Observer Estimated State (d_F=1.0, d_tau=0.1)')
plt.tight_layout()
plt.show()