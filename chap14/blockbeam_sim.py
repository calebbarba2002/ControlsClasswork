# 3rd-party
import numpy as np
import matplotlib.pyplot as plt
# local (controlbook)
from case_studies import common, E_blockbeam

# Part (a): alpha=0.2 for 20% parameter uncertainty
blockbeam = E_blockbeam.Dynamics(alpha=0.2)

# Part (b): disturbance observer controller
controller = E_blockbeam.ControllerSSIDO()

z_ref = common.SignalGenerator(amplitude=0.15, frequency=0.03, y_offset=0.25)

# Part (a): input disturbance of 0.5
d_force = np.array([0.5])

# Part (a): output noise std=0.001 on z and theta (2 output channels)
output_noise = [
    common.SignalGenerator(amplitude=0.001),  # noise on z
    common.SignalGenerator(amplitude=0.001),  # noise on theta
]

time, x_hist, u_hist, r_hist, xhat_hist, d_hist, *_ = common.run_simulation(
    blockbeam,
    [z_ref],
    controller,
    controller_input="measurement",
    input_disturbance=d_force,
    output_noise=output_noise,
    t_final=60,
    dt=E_blockbeam.params.ts,
)

viz = E_blockbeam.Visualizer(time, x_hist, u_hist, r_hist, xhat_hist, d_hist)
viz.plot()

# Part (b): plot true state vs estimated state
fig, axes = plt.subplots(4, 1, figsize=(10, 10))
labels_true = ['z (true)', 'θ (true)', 'ż (true)', 'θ̇ (true)']
labels_est  = ['ẑ (est)',  'θ̂ (est)',  'ẑ̇ (est)',  'θ̂̇ (est)']
ylabels     = ['z (m)', 'θ (rad)', 'ż (m/s)', 'θ̇ (rad/s)']

for i in range(4):
    axes[i].plot(time, x_hist[:, i], label=labels_true[i], linewidth=2)
    axes[i].plot(time, xhat_hist[:, i], '--', label=labels_est[i], linewidth=2)
    axes[i].set_ylabel(ylabels[i])
    axes[i].legend()
    axes[i].grid(True)

axes[-1].set_xlabel('Time (s)')
plt.suptitle('E.14: True State vs Observer Estimated State\n(alpha=0.2, d=0.5, noise=0.001)')
plt.tight_layout()
plt.show()