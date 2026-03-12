# 3rd-party
import numpy as np
from matplotlib.patches import Rectangle

# local (controlbook)
from ..common.animator import MatplotlibAxisAnimator


class BlockbeamAnimator(MatplotlibAxisAnimator):
    def setup_background(self):
        self.z_hist = self.x_hist[0]
        self.theta_hist = self.x_hist[1]

        self.width = 0.05
        self.length = 0.5
        self.ax.set_xlim(-self.length * 0.25, self.length * 1.25)
        self.ax.set_ylim(-self.length, self.length)
        self.ax.set_xlabel("z")
        self.ax.set_aspect("equal", "box")

        (baseline,) = self.ax.plot([0.0, self.length], [0.0, 0.0], "k--")

    def setup_changing_objects(self, x0, r0):
        z0, theta0 = x0[:2]

        xy = self.get_block_xy(z0, theta0)
        self.block = Rectangle(
            xy,
            width=self.width,
            height=self.width * 0.25,
            angle=np.rad2deg(theta0),
            facecolor="limegreen",
            edgecolor="black",
        )
        self.ax.add_patch(self.block)

        x, y = self.get_beam_data(theta0)
        (self.beam,) = self.ax.plot(x, y, lw=3, c="black")

    def update_changing_objects(self, x, r):
        z, theta = x[:2]

        self.block.set_xy(self.get_block_xy(z, theta))
        self.block.set_angle(np.rad2deg(theta))

        self.beam.set_data(self.get_beam_data(theta))

        return self.block, self.beam

    def get_block_xy(self, z, theta):
        gap = 0.005
        lower_left_corner_unrotated = np.array([z - 0.5 * self.width, gap])
        R = np.array([[np.cos(theta), -np.sin(theta)], [np.sin(theta), np.cos(theta)]])
        lower_left_corner = R @ lower_left_corner_unrotated
        return tuple(lower_left_corner)

    def get_beam_data(self, theta):
        x = [0, self.length * np.cos(theta)]
        y = [0, self.length * np.sin(theta)]
        return x, y
