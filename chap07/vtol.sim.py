# local (controlbook)
from case_studies import common, F_vtol

vtol = F_vtol.Dynamics()
controller = F_vtol.AltitudeControllerPD()
h_ref = common.SignalGenerator(amplitude=3.0, frequency=0.01, y_offset=5.0)
z_ref = None # lateral position not controlled in this example

time, x_hist, u_hist, r_hist, *_ = common.run_simulation(
    vtol,
    [z_ref, h_ref],
    controller,
    controller_input="state",
    t_final=35,
    dt=F_vtol.params.ts,
)
viz = F_vtol.Visualizer(time, x_hist, u_hist, r_hist)
viz.animate()