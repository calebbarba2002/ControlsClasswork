# 3rd-party
import numpy as np

# local (controlbook)
from case_studies import common, C_satellite


satellite = C_satellite.Dynamics()
controller = C_satellite.ControllerSS()

phi_ref = common.SignalGenerator(amplitude=np.radians(15), frequency=0.015)
refs = [phi_ref]

time, x_hist, u_hist, r_hist, *_ = common.run_simulation(
    satellite,
    refs,
    controller,
    controller_input="state",
    t_final=100,
    dt=C_satellite.params.ts,
)

viz = C_satellite.Visualizer(time, x_hist, u_hist, r_hist)
viz.plot()
# viz.animate()
