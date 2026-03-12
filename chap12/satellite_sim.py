# 3rd-party
import numpy as np

# local (controlbook)
from case_studies import common, C_satellite


satellite = C_satellite.Dynamics(alpha=0.2)
controller = C_satellite.ControllerSSI(separate_integrator=False)

phi_ref = common.SignalGenerator(amplitude=np.radians(15), frequency=0.015)
refs = [phi_ref]

d_torque = np.array([1.0])

time, x_hist, u_hist, r_hist, xhat_hist, d_hist, *_ = common.run_simulation(
    satellite,
    refs,
    controller,
    controller_input="state",
    input_disturbance=d_torque,
    t_final=100,
    dt=C_satellite.params.ts,
)

viz = C_satellite.Visualizer(time, x_hist, u_hist, r_hist, xhat_hist, d_hist)
viz.plot()
# viz.animate()
