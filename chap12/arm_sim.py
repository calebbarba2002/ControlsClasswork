# 3rd-party
import numpy as np

# local (controlbook)
from case_studies import common, A_arm


arm = A_arm.Dynamics(alpha=0.2)
controller = A_arm.ControllerSSI(separate_integrator=False)
theta_ref = common.SignalGenerator(amplitude=np.radians(50), frequency=0.05)
d_force = np.array([0.25])

time, x_hist, u_hist, r_hist, xhat_hist, d_hist, *_ = common.run_simulation(
    arm,
    [theta_ref],
    controller,
    controller_input="state",
    input_disturbance=d_force,
    t_final=20,
    dt=A_arm.params.ts,
)

viz = A_arm.Visualizer(time, x_hist, u_hist, r_hist, xhat_hist, d_hist)
viz.plot()
# viz.animate()
