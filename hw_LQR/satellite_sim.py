# 3rd-party
import numpy as np

# local (controlbook)
from case_studies import common, C_satellite


satellite = C_satellite.Dynamics(alpha=0.2)
controller = C_satellite.ControllerLQRIDO(separate_integrator=False)

phi_ref = common.SignalGenerator(amplitude=np.radians(15), frequency=0.015)
refs = [phi_ref]

d_torque = np.array([1.0])

theta_noise = common.SignalGenerator(amplitude=0.001)
phi_noise = common.SignalGenerator(amplitude=0.001)
y_noise = [theta_noise, phi_noise]

time, x_hist, u_hist, r_hist, xhat_hist, d_hist, dhat_hist = common.run_simulation(
    satellite,
    refs,
    controller,
    controller_input="measurement",
    input_disturbance=d_torque,
    output_noise=y_noise,
    t_final=100,
    dt=C_satellite.params.ts,
)

viz = C_satellite.Visualizer(time, x_hist, u_hist, r_hist, xhat_hist, d_hist, dhat_hist)
viz.plot()
# viz.animate()
