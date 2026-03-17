# local (controlbook)
from case_studies import common, F_vtol

vtol = F_vtol.Dynamics(alpha=0.2)
controller = F_vtol.ControllerPID()

z_ref = common.SignalGenerator(amplitude=3.0, frequency=0.04, y_offset=5.0)
h_ref = common.SignalGenerator(amplitude=2.0, frequency=0.03, y_offset=5.0)
theta_ref = None # allows controller outer loop to fill in this value (not necessary)
refs = [z_ref, h_ref, theta_ref]

time, x_hist, u_hist, r_hist, xhat_hist, *_ = common.run_simulation(
    vtol,
    refs,
    controller,
    controller_input="measurement",
    t_final=60,
    dt=F_vtol.params.ts,
)

viz = F_vtol.Visualizer(time, x_hist, u_hist, r_hist, xhat_hist, convert_to_F_tau=False)
viz.plot()
viz.animate(speed_scale=3)