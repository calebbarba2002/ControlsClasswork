# 3rd-party
import numpy as np

# local (controlbook)
from case_studies import common, D_mass

mass = D_mass.Dynamics()
controller = D_mass.ControllerLQRIDO()
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