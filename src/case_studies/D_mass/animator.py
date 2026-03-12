# 3rd-party
import numpy as np
from matplotlib.patches import Rectangle

# local (controlbook)
from ..common.animator import MatplotlibAxisAnimator


class MassAnimator(MatplotlibAxisAnimator):
    def setup_background(self):
        self.width = 0.25
        self.length = self.width * 8
        x_low = -self.length * 1.1
        x_high = self.length * 1.1
        y_low = -self.length * 0.8
        y_high = self.length * 1.2
        self.ax.axis((x_low, x_high, y_low, y_high))
        self.ax.set_aspect("equal", "box")

        (track,) = self.ax.plot([x_low, x_high], [0, 0], "k--")
        (wall,) = self.ax.plot([-self.length, -self.length], [0, 2 * self.width], "k")

    def setup_changing_objects(self, x0, r0):
        z0 = x0[0]
        xy = self.get_block_xy(z0)
        self.block = Rectangle(xy, self.width, self.width, fc="blue", ec="black")
        self.ax.add_patch(self.block)

        x, y = self.get_spring_data(z0)
        (self.spring,) = self.ax.plot(x, y, lw=1, c="black")

    def update_changing_objects(self, x, r):
        z = x[0]
        self.block.set_xy(self.get_block_xy(z))
        self.spring.set_data(self.get_spring_data(z))
        return self.block, self.spring

    def get_block_xy(self, z):
        pos_x = z - self.width * 0.5
        pos_y = 0.0
        lower_left_corner = (pos_x, pos_y)
        return lower_left_corner

    def get_spring_data(self, z):
        x = [-self.length, z - self.width * 0.5]
        y = [self.width * 0.5, self.width * 0.5]
        return x, y
