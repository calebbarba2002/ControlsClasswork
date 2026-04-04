import numpy as np
from case_studies import H_hummingbird
from case_studies.H_hummingbird import params as P
from case_studies.H_hummingbird.pid_controller import HummingbirdControllerPID

# Step 1: 20% model uncertainty
hummingbird = H_hummingbird.Dynamics(alpha=0.2)
controller = HummingbirdControllerPID()

# initialize histories — x_hist and r_hist start with one entry, u_hist starts empty
x_hist = [hummingbird.state]
u_hist = []
r_hist = [np.array([0.0, 0.0, np.deg2rad(30.0) * np.sign(np.sin(0.0))])]

time = np.arange(start=0.0, stop=40.0, step=P.ts, dtype=np.float64)

# Step 2: disturbance of 0.05 N per motor
d_force = np.array([0.05, 0.05])

for t in time[1:]:
    theta_ref = 0.0
    psi_ref = np.deg2rad(30.0) * np.sign(np.sin(2.0 * np.pi * 0.05 * t))
    r = np.array([theta_ref, psi_ref])

    u, xhat = controller.update_with_measurement(r, hummingbird.state[0:3])

    F = u[0]
    tau = u[1]

    motor_forces = P.mixer @ np.array([F, tau])
    pwm = motor_forces / hummingbird.km
    pwm_disturbed = pwm + d_force

    hummingbird.update(pwm_disturbed)

    x_hist.append(hummingbird.state)
    u_hist.append(pwm_disturbed)
    # store [phi_ref, theta_ref, psi_ref] so visualizer maps correctly
    r_hist.append(np.array([0.0, theta_ref, psi_ref]))

x_hist = np.array(x_hist)
u_hist = np.array(u_hist)
r_hist = np.array(r_hist)

viz = H_hummingbird.Visualizer(time, x_hist, u_hist, r_hist)

viz.animate()
# viz.plot()



# import numpy as np
# from case_studies import H_hummingbird
# from case_studies.H_hummingbird import params as P
# from case_studies.H_hummingbird.dynamics import HummingbirdDynamics

# # Step 1: 20% model uncertainty
# hummingbird = H_hummingbird.Dynamics(alpha=0.2)

# # Step 3: start with PD to see steady-state error, then switch to PID
# controller = H_hummingbird.ControllerPID()

# x_hist = [hummingbird.state]
# u_hist = []
# r_hist = []

# time = np.arange(start=0.0, stop=40.0, step=P.ts, dtype=np.float64)

# # Step 2: disturbance of 0.1 N total (0.05 N per motor)
# d_force = np.array([0.05, 0.05])

# for t in time[1:]:
#     theta_ref = 0.0
#     psi_ref = np.deg2rad(30.0) * np.sign(np.sin(2.0 * np.pi * 0.05 * t))
#     r = np.array([theta_ref, psi_ref])

#     u = controller.update_with_measurement(r, hummingbird.state[0:3])

#     F = u[0]
#     tau = u[1]

#     motor_forces = P.mixer @ np.array([F, tau])
#     pwm = motor_forces / hummingbird.km

#     # Step 2: add disturbance
#     pwm_disturbed = pwm + d_force

#     y = hummingbird.update(pwm_disturbed)

#     x_hist.append(hummingbird.state)
#     u_hist.append(pwm)
#     r_hist.append(r)

# x_hist = np.array(x_hist)
# u_hist = np.array(u_hist)
# r_hist = np.array(r_hist)

# viz = H_hummingbird.Visualizer(time, x_hist, u_hist)
# viz.animate()
# # viz.plot()