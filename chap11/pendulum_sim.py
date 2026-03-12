# local (controlbook)
from case_studies import common, B_pendulum


pendulum = B_pendulum.Dynamics()
controller = B_pendulum.ControllerSS()

z_ref = common.SignalGenerator(amplitude=0.5, frequency=0.04)
refs = [z_ref]

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
