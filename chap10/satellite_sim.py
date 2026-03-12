# 3rd-party
import numpy as np

# local (controlbook)
from case_studies import common, C_satellite


satellite = C_satellite.Dynamics(alpha=0.2)
controller = C_satellite.ControllerPID()

theta_ref = None  # allows controller outer loop to fill in this value (not necessary)
phi_ref = common.SignalGenerator(amplitude=np.radians(15), frequency=0.015)
refs = [theta_ref, phi_ref]

time, x_hist, u_hist, r_hist, xhat_hist, *_ = common.run_simulation(
    satellite,
    refs,
    controller,
    controller_input="measurement",
    t_final=100,
    dt=C_satellite.params.ts,
)

viz = C_satellite.Visualizer(time, x_hist, u_hist, r_hist, xhat_hist)
viz.plot()
# viz.animate()
