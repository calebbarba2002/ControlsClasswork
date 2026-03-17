# local (controlbook)
from case_studies import common, F_vtol

vtol = F_vtol.Dynamics()
# controller = VTOLControllerSS(use_single_system=True)
controller = F_vtol.ControllerSS(use_single_system=False)

z_ref = common.SignalGenerator(amplitude=3.0, frequency=0.04, y_offset=5.0)
h_ref = common.SignalGenerator(amplitude=2.0, frequency=0.03, y_offset=5.0)
refs = [z_ref, h_ref]

time, x_hist, u_hist, r_hist, *_ = common.run_simulation(
    vtol, refs, controller, controller_input="state", t_final=60, dt=F_vtol.params.ts
)

viz = F_vtol.Visualizer(time, x_hist, u_hist, r_hist, convert_to_F_tau=True)
viz.plot()
# viz.animate()