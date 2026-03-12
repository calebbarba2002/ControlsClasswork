# 3rd-party
import numpy as np

# local (controlbook)
from case_studies import common, A_arm


arm = A_arm.Dynamics()
controller = A_arm.ControllerPD(use_feedback_linearization=True)
theta_ref = common.SignalGenerator(amplitude=np.radians(30), frequency=0.01)

time, x_hist, u_hist, r_hist, *_ = common.run_simulation(
    arm,
    [theta_ref],
    controller,
    controller_input="state",
    t_final=5,
    dt=A_arm.params.ts,
)

viz = A_arm.Visualizer(time, x_hist, u_hist, r_hist)
viz.animate()
