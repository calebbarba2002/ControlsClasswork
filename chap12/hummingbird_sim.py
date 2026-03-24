# local (controlbook)
from case_studies import common, H_hummingbird
from case_studies.H_hummingbird.full_pd_controller import FullPDController
import numpy as np

# create system
hummingbird = H_hummingbird.Dynamics()

# controller
controller = FullPDController()

# =========================
# REFERENCE SIGNALS
# =========================

# YAW reference
psi_ref = common.SignalGenerator(
    amplitude=np.radians(30),   # 30 deg yaw motion
    frequency=0.05,
    y_offset=0.0
)

# keep pitch stable
theta_ref = common.SignalGenerator(
    amplitude=0.0,
    frequency=0.01,
    y_offset=0.0
)

# roll is not directly commanded
phi_ref = common.SignalGenerator(amplitude=0.0)

# references: [phi, theta, psi]
refs = [
    phi_ref,
    theta_ref,
    psi_ref
]

# =========================
# RUN SIM
# =========================

time, x_hist, u_hist, r_hist, *_ = common.run_simulation(
    hummingbird,
    refs,
    controller,
    controller_input="measurement",   # ✅ using measurement version
    t_final=30,
    dt=H_hummingbird.params.ts
)

# =========================
# VISUALIZATION
# =========================

viz = H_hummingbird.Visualizer(time, x_hist, u_hist)
viz.animate()
