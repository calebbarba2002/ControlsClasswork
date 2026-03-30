# 3rd-party
import numpy as np
# local (controlbook)
from case_studies import common, D_mass
mass = D_mass.Dynamics(alpha=0.2)
controller = D_mass.ControllerSSI()
z_ref = common.SignalGenerator(amplitude=0.5, frequency=0.04)
d_force = np.array([0.25])

time, x_hist, u_hist, r_hist, xhat_hist, d_hist, *_ = common.run_simulation(
    mass,
    [z_ref],
    controller,
    controller_input="state",
    input_disturbance=d_force,
    t_final=50,
    dt=D_mass.params.ts,
)
viz = D_mass.Visualizer(time, x_hist, u_hist, r_hist, xhat_hist, d_hist)
viz.plot()
# viz.animate()
