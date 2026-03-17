import numpy as np
from case_studies import H_hummingbird

#1 create system

system = H_hummingbird.dynamics.HummingbirdDynamics()

#2 sets equilibrium state
#theta_e = 0
#all velocities = 0
system.state = np.array([
    0.0,  # phi
    0.0,  # theta
    0.0,  # psi
    0.0,  # phidot
    0.0,  # thetadot
    0.0   # psidot
])

#3 equilibrium input
#
#from dynamics.py:
#   self.km = (m1*ell1 + m2*ell2)g / ellT
#
#and update() does:
#   force = pwm * km
#
#therefore equilibrium pwm = 0.5 on each motor to produce equal forces that balance gravity and produce zero net torque.
pwm_hover = np.array([0.5, 0.5])


#simulation setup

ts = system.dt
time = np.arange(0.0, 10.0, ts)

x_hist = [system.state.copy()]
u_hist = []

#simulate

for t in time[1:]:
    system.update(pwm_hover)
    x_hist.append(system.state.copy())
    u_hist.append(pwm_hover.copy())

x_hist = np.array(x_hist)
u_hist = np.array(u_hist)

#visualizer

viz = H_hummingbird.visualizer.HummingbirdVisualizer(
    time, x_hist, u_hist
)
viz.animate()
