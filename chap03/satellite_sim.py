# 3rd-party
import numpy as np

# local (controlbook)
from case_studies import common, C_satellite


# initialize system and input generator
satellite = C_satellite.Dynamics()
torque_gen = common.SignalGenerator(amplitude=0.1, frequency=0.1)

# initialize data storage
x_hist = [satellite.state]
u_hist = []

# loop over time
time = np.arange(start=0.0, stop=25.0, step=C_satellite.params.ts, dtype=np.float64)
for t in time[1:]:
    # generate input signal
    u = np.array([torque_gen.sin(t)])

    # simulate system
    y = satellite.update(u)

    # store data for visualization
    u_hist.append(u)
    x_hist.append(satellite.state)

# convert data to numpy arrays
x_hist = np.array(x_hist)
u_hist = np.array(u_hist)

# visualize
viz = C_satellite.Visualizer(time, x_hist, u_hist)
viz.animate()  # could also just call viz.plot()
