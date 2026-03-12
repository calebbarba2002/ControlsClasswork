# 3rd-party
import numpy as np

# local (controlbook)
from case_studies import common, B_pendulum


# initialize system and input generator
pendulum = B_pendulum.Dynamics()
force_gen = common.SignalGenerator(amplitude=1.0, frequency=1.0)

# initialize data storage
x_hist = [pendulum.state]
u_hist = []

# loop over time
time = np.arange(start=0.0, stop=15.0, step=B_pendulum.params.ts, dtype=np.float64)
for t in time[1:]:
    # generate input signal
    u = np.array([force_gen.sin(t)])

    # simulate system response
    y = pendulum.update(u)

    # store data for visualization
    u_hist.append(u)
    x_hist.append(pendulum.state)

# convert data to numpy arrays
x_hist = np.array(x_hist)
u_hist = np.array(u_hist)

# visualize
viz = B_pendulum.Visualizer(time, x_hist, u_hist)
viz.animate()  # could also just call viz.plot()
