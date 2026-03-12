# local (controlbook)
from case_studies import common, B_pendulum


pendulum = B_pendulum.Dynamics(alpha=0.2)
controller = B_pendulum.ControllerPID()

z_ref = common.SignalGenerator(amplitude=0.5, frequency=0.04)
theta_ref = None  #  allows controller outer loop to fill in this value (not necessary)
refs = [z_ref, theta_ref]

time, x_hist, u_hist, r_hist, xhat_hist, *_ = common.run_simulation(
    pendulum,
    refs,
    controller,
    controller_input="measurement",
    t_final=60,
    dt=B_pendulum.params.ts,
)

viz = B_pendulum.Visualizer(time, x_hist, u_hist, r_hist, xhat_hist)
viz.plot()
# viz.animate()
