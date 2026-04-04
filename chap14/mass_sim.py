# 3rd-party
import numpy as np

# local (controlbook)
from case_studies import common, D_mass

# Part (a): alpha=0.2 for 20% parameter uncertainty
mass = D_mass.Dynamics(alpha=0.2)

# Part (b): disturbance observer controller
controller = D_mass.ControllerSSIDO()

z_ref = common.SignalGenerator(amplitude=0.5, frequency=0.04)

# Part (a): input disturbance of 0.25
d_force = np.array([0.25])

# Part (a): output noise with standard deviation 0.001
output_noise = [common.SignalGenerator(amplitude=0.001)]

time, x_hist, u_hist, r_hist, xhat_hist, d_hist, *_ = common.run_simulation(
    mass,
    [z_ref],
    controller,
    controller_input="measurement",
    input_disturbance=d_force,
    output_noise=output_noise,
    t_final=50,
    dt=D_mass.params.ts,
)

viz = D_mass.Visualizer(time, x_hist, u_hist, r_hist, xhat_hist, d_hist)
viz.plot()