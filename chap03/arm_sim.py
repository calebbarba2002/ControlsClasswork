# 3rd-party
import numpy as np

# local (controlbook)
from case_studies import common, A_arm


# initialize system and input generator
arm = A_arm.Dynamics()
torque_gen = common.SignalGenerator(amplitude=0.2, frequency=0.05)

# initialize data storage
x_hist = [arm.state]
u_hist = []

# loop over time
time = np.arange(start=0.0, stop=50.0, step=A_arm.params.ts, dtype=np.float64)
for t in time[1:]:
    # generate input signal
    u = np.array([torque_gen.square(t)])

    # simulate system response
    y = arm.update(u)

    # store data for visualization
    u_hist.append(u)
    x_hist.append(arm.state)

# convert data to numpy arrays
x_hist = np.array(x_hist)
u_hist = np.array(u_hist)

# visualize
viz = A_arm.Visualizer(time, x_hist, u_hist)
viz.animate()  # could also just call viz.plot()
