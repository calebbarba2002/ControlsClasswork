# 3rd-party
import numpy as np

# local (controlbook)
from case_studies import common, B_pendulum


pendulum = B_pendulum.Dynamics()
controller = B_pendulum.ControllerSSIO(separate_integrator=False)

z_ref = common.SignalGenerator(amplitude=0.5, frequency=0.04)
refs = [z_ref]

d_force = np.array([0.05])

time, x_hist, u_hist, r_hist, xhat_hist, d_hist, *_ = common.run_simulation(
    pendulum,
    refs,
    controller,
    controller_input="measurement",
    input_disturbance=d_force,
    t_final=60,
    dt=B_pendulum.params.ts,
)

viz = B_pendulum.Visualizer(time, x_hist, u_hist, r_hist, xhat_hist, d_hist)
viz.plot()
# viz.animate()
