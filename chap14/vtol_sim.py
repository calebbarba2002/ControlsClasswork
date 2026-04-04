# 3rd-party
import numpy as np
import matplotlib.pyplot as plt
# local (controlbook)
from case_studies import common, F_vtol

P = F_vtol.params

# Part (a): alpha=0.2 for 20% parameter uncertainty
vtol = F_vtol.Dynamics(alpha=0.2, F_wind=1.0)

# Part (b): disturbance observer controller
controller = F_vtol.ControllerSSIDO()

z_ref = common.SignalGenerator(amplitude=3.0, frequency=0.04, y_offset=5.0)
h_ref = common.SignalGenerator(amplitude=2.0, frequency=0.03, y_offset=5.0)
refs = [z_ref, h_ref]

# Part (a): altitude disturbance of 1.0
d = P.mixer @ np.array([1.0, 0.0])

# Part (b): output noise std=0.01 on z, h, and theta (3 output channels)
output_noise = [
    common.SignalGenerator(amplitude=0.01),  # noise on z
    common.SignalGenerator(amplitude=0.01),  # noise on h
    common.SignalGenerator(amplitude=0.01),  # noise on theta
]

time, x_hist, u_hist, r_hist, xhat_hist, d_hist, *_ = common.run_simulation(
    vtol,
    refs,
    controller,
    controller_input="measurement",
    input_disturbance=d,
    output_noise=output_noise,
    t_final=60,
    dt=P.ts,
)

viz = F_vtol.Visualizer(
    time, x_hist, u_hist, r_hist, xhat_hist, d_hist, convert_to_F_tau=False
)
viz.plot()

# Part (b): plot true state vs estimated state
state_labels = ['z (m)', 'h (m)', 'θ (rad)', 'ż (m/s)', 'ḣ (m/s)', 'θ̇ (rad/s)']
true_labels  = ['z', 'h', 'θ', 'ż', 'ḣ', 'θ̇']
est_labels   = ['ẑ', 'ĥ', 'θ̂', 'ẑ̇', 'ĥ̇', 'θ̂̇']

fig, axes = plt.subplots(6, 1, figsize=(10, 14))
for i in range(6):
    axes[i].plot(time, x_hist[:, i], label=true_labels[i], linewidth=2)
    axes[i].plot(time, xhat_hist[:, i], '--', label=est_labels[i], linewidth=2)
    axes[i].set_ylabel(state_labels[i])
    axes[i].legend()
    axes[i].grid(True)

axes[-1].set_xlabel('Time (s)')
plt.suptitle('F.14: True State vs Observer Estimated State\n(alpha=0.2, wind=1.0, altitude_dist=1.0, noise=0.01)')
plt.tight_layout()
plt.show()