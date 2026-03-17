# local (controlbook)
from case_studies import common, D_mass

mass = D_mass.Dynamics()
controller = D_mass.ControllerSS()
z_ref = common.SignalGenerator(amplitude=0.5, frequency=0.04)

time, x_hist, u_hist, r_hist, xhat_hist, *_ = common.run_simulation(
    mass,
    [z_ref],
    controller,
    controller_input="state",
    t_final=50,
    dt=D_mass.params.ts,
)

viz = D_mass.Visualizer(time, x_hist, u_hist, r_hist, xhat_hist)
viz.plot()
# viz.animate()