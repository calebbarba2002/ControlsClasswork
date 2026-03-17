import numpy as np
from case_studies import H_hummingbird

system = H_hummingbird.dynamics.HummingbirdDynamics()

ts = system.dt
time = np.arange(0.0, 20.0, ts)

x_hist = [system.state.copy()]
u_hist = []

#grav balancing PWM
pwm_hover = np.array([0.2, 0.0])  #equal so zero roll torque, [pwm_left, pwm_right]

for t in time[1:]:
    system.update(pwm_hover)
    x_hist.append(system.state.copy())
    u_hist.append(pwm_hover.copy())

x_hist = np.array(x_hist)
u_hist = np.array(u_hist)

viz = H_hummingbird.visualizer.HummingbirdVisualizer(
    time, x_hist, u_hist
)
viz.animate()
