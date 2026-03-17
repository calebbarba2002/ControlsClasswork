# 3rd-party
import numpy as np

# local (controlbook)
from case_studies import common, H_hummingbird

# Signal generators (angles in radians)
phi_gen   = common.SignalGenerator(amplitude=np.radians(50), frequency=0.5)
theta_gen = common.SignalGenerator(amplitude=np.radians(20), frequency=0.1)
psi_gen   = common.SignalGenerator(amplitude=np.radians(30), frequency=0.2)
fr_gen = common.SignalGenerator(amplitude=5.0, frequency=0.3)
fl_gen = common.SignalGenerator(amplitude=5.0, frequency=0.3)

x0 = np.zeros(6)
x_hist = [x0]
u_hist = []

time = np.arange(start = 0, stop=20, step=0.01)

for t in time[1:]:
    x = np.empty(6)

    x[0] = phi_gen.sin(t)
    x[1] = theta_gen.sin(t)
    x[2] = psi_gen.sin(t)
    x[3] = 0
    x[4] = 0
    x[5] = 0


    u = np.array([fr_gen.square(t), fl_gen.square(t)])

    # Store state and inputs
    x_hist.append(x)
    u_hist.append(u)

# Convert lists to arrays for Visualizer
x_hist = np.array(x_hist)
u_hist = np.array(u_hist)

# Animate
viz = H_hummingbird.Visualizer(time, x_hist, u_hist)
viz.animate()
