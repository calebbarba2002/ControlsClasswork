# 3rd-party
import numpy as np
import matplotlib.pyplot as plt
# local (controlbook)
from case_studies import common, E_blockbeam

# Part (a): alpha=0 for exact parameters
blockbeam = E_blockbeam.Dynamics(alpha=0.0)

# Part (c): observer-based controller
controller = E_blockbeam.ControllerSSIO()

z_ref = common.SignalGenerator(amplitude=0.15, frequency=0.03, y_offset=0.25)

# Part (e): input disturbance of 0.5
d_force = np.array([0.5])

time, x_hist, u_hist, r_hist, xhat_hist, d_hist, *_ = common.run_simulation(
    blockbeam,
    [z_ref],
    controller,
    controller_input="measurement",  # uses observer, not full state
    input_disturbance=d_force,
    t_final=60,
    dt=E_blockbeam.params.ts,
)

viz = E_blockbeam.Visualizer(time, x_hist, u_hist, r_hist, xhat_hist, d_hist)
viz.plot()

# Part (d): plot true state vs estimated state
fig, axes = plt.subplots(4, 1, figsize=(10, 10))
labels_true  = ['z (true)', 'θ (true)', 'ż (true)', 'θ̇ (true)']
labels_est   = ['ẑ (est)', 'θ̂ (est)', 'ẑ̇ (est)', 'θ̂̇ (est)']
ylabels      = ['z (m)', 'θ (rad)', 'ż (m/s)', 'θ̇ (rad/s)']

for i in range(4):
    axes[i].plot(time, x_hist[:, i], label=labels_true[i], linewidth=2)
    axes[i].plot(time, xhat_hist[:, i], '--', label=labels_est[i], linewidth=2)
    axes[i].set_ylabel(ylabels[i])
    axes[i].legend()
    axes[i].grid(True)

axes[-1].set_xlabel('Time (s)')
plt.suptitle('True State vs Observer Estimated State (d=0.5 disturbance)')
plt.tight_layout()
plt.show()