# local (controlbook)
from case_studies import common, E_blockbeam

blockbeam = E_blockbeam.Dynamics(alpha=0.2)
controller = E_blockbeam.ControllerPID()
z_ref = common.SignalGenerator(amplitude=0.15, frequency=0.03, y_offset=0.25)

time, x_hist, u_hist, r_hist, xhat_hist, *_ = common.run_simulation(
    blockbeam,
    [z_ref, None],
    controller,
    controller_input="measurement",
    t_final=60,
    dt=E_blockbeam.params.ts,
)

viz = E_blockbeam.Visualizer(time, x_hist, u_hist, r_hist, xhat_hist)
viz.plot()
# viz.animate()