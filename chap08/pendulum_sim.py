# 3rd-party pip installed packages
import numpy as np

# local (controlbook)
from case_studies import common, B_pendulum


pendulum = B_pendulum.Dynamics()
controller = B_pendulum.ControllerPD()

# this z_ref amplitude could be non-zero, but book asks for stabilization only
z_ref = common.SignalGenerator(amplitude=0.0, frequency=0.04)
theta_ref = None  #  allows controller outer loop to fill in this value (not necessary)
refs = [z_ref, theta_ref]

time, x_hist, u_hist, r_hist, *_ = common.run_simulation(
    pendulum,
    refs,
    controller,
    controller_input="state",
    t_final=60,
    dt=B_pendulum.params.ts,
)

viz = B_pendulum.Visualizer(time, x_hist, u_hist, r_hist)
viz.plot()
# viz.animate()
