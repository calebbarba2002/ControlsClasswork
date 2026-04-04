# # 3rd-party
# import numpy as np

# # local (controlbook)
# from case_studies import H_hummingbird, common


# hummingbird = H_hummingbird.Dynamics()
# controller = H_hummingbird.ControllerLonPD()
# theta_ref = common.SignalGenerator(amplitude=np.radians(30), frequency=0.1)
# psi_ref = None  # yaw not controlled in this example

# time, x_hist, u_hist, r_hist, *_ = common.run_simulation(
#     hummingbird,
#     [None, theta_ref, psi_ref],  # roll needs a placeholder for visualization
#     controller,
#     controller_input="measurement",
#     t_final=20,
#     dt=H_hummingbird.params.ts,  # TODO: decide if ts should be a param or always use 0.01
# )

# viz = H_hummingbird.Visualizer(time, x_hist, u_hist, r_hist)
# viz.animate()



# local (controlbook)
from case_studies import common, H_hummingbird
from case_studies.H_hummingbird.longitudinal_pd_controller import LongitudinalPDController
import numpy as np

#create system
hummingbird = H_hummingbird.Dynamics()

#controller
controller = LongitudinalPDController()

#reference signal(ONLY theta matters)
theta_ref = common.SignalGenerator(
    amplitude=np.radians(30),   # 30 degrees → radians
    frequency=0.1,              # from lab
    y_offset=0.0
)

#references: [phi, theta, psi], we only care about theta, others = 0
refs = [
    common.SignalGenerator(amplitude=0.0),  #phi
    theta_ref,                              #theta
    common.SignalGenerator(amplitude=0.0)   #psi
]

#run simulation
time, x_hist, u_hist, r_hist, *_ = common.run_simulation(
    hummingbird,
    refs,
    controller,
    controller_input="state",
    t_final=30,
    dt=H_hummingbird.params.ts
)

#Animate
viz = H_hummingbird.Visualizer(time, x_hist, u_hist)
viz.animate()