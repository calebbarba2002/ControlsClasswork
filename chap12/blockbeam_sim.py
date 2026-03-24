# local (controlbook)
from case_studies import common, E_blockbeam

# add parameter uncertainty
blockbeam = E_blockbeam.Dynamics(alpha=0.2)

controller = E_blockbeam.ControllerSS()

z_ref = common.SignalGenerator(amplitude=0.15, frequency=0.03, y_offset=0.25)

time, x_hist, u_hist, r_hist, xhat_hist, *_ = common.run_simulation(
    blockbeam,
    [z_ref],
    controller,
    controller_input="state",
    t_final=60,
    dt=E_blockbeam.params.ts,
    input_disturbance=1.0,   # add disturbance
)

viz = E_blockbeam.Visualizer(time, x_hist, u_hist, r_hist, xhat_hist)
viz.plot()
