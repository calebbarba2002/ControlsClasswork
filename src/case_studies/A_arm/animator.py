# 3rd-party
import numpy as np

# local (controlbook)
from ..common.animator import MatplotlibAxisAnimator


class ArmAnimator(MatplotlibAxisAnimator):
    """
    By inheriting from MatplotlibAxisAnimator, this class has access to the
    matplotlib axis object via `self.ax`. This class just needs to implement
    the `setup_background()`, `setup_changing_objects()`, and
    `update_changing_objects()` methods.
    """

    def setup_background(self):
        self.length = 1.0
        lim = self.length * 1.25
        self.ax.axis((-lim, lim, -lim, lim))
        self.ax.set_aspect("equal", "box")

        (baseline,) = self.ax.plot([0, self.length], [0, 0], "k--")

    def setup_changing_objects(self, x0, r0):
        theta0 = x0[0]
        x, y = self.get_arm_endpoints(theta0)
        (self.arm_line,) = self.ax.plot(x, y, linewidth=5, color="blue")

    def update_changing_objects(self, x, r):
        theta = x[0]
        x, y = self.get_arm_endpoints(theta)
        self.arm_line.set_xdata(x)
        self.arm_line.set_ydata(y)
        return (self.arm_line,)

    def get_arm_endpoints(self, theta):
        x = np.array([0, self.length * np.cos(theta)])
        y = np.array([0, self.length * np.sin(theta)])
        return x, y
