import numpy as np

from case_studies import common, H_hummingbird
from case_studies.H_hummingbird import params as P
from case_studies.H_hummingbird.dynamics import HummingbirdDynamics
from case_studies.H_hummingbird.full_pd_controller import FullPDController


hummingbird = HummingbirdDynamics()
controller = FullPDController()

x_hist = [hummingbird.state]
u_hist = []
r_hist = []

time = np.arange(start=0.0, stop=40.0, step=P.ts, dtype=np.float64)
for t in time[1:]:

    theta_ref = 0.0
    psi_ref = np.deg2rad(30.0) * np.sign(np.sin(2.0 * np.pi * 0.05 * t))
    # psi_ref = np.deg2rad(30.0) * (t*0+1)
    r = np.array([theta_ref, psi_ref])

    u = controller.update_with_measurement(r, hummingbird.state[0:3])

    F = u[0]
    tau = u[1]

    motor_forces = P.mixer @ np.array([F, tau])
    pwm = motor_forces / hummingbird.km

    y = hummingbird.update(pwm)

    x_hist.append(hummingbird.state)
    u_hist.append(pwm)
    r_hist.append(r)

x_hist = np.array(x_hist)
u_hist = np.array(u_hist)
r_hist = np.array(r_hist)

viz = H_hummingbird.Visualizer(time, x_hist, u_hist)
viz.animate()
# viz.plot()


#preferred way to run this is with the sim script template below
# 3rd-party
# import numpy as np

# # local (controlbook)
# from case_studies import H_hummingbird, common


# hummingbird = H_hummingbird.Dynamics()
# controller = H_hummingbird.ControllerPD()
# theta_ref = common.SignalGenerator(amplitude=np.radians(30), frequency=0.1)
# psi_ref = common.SignalGenerator(amplitude=np.radians(30), frequency=0.1) # yaw not controlled in this example

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