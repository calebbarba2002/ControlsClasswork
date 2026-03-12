# 3rd-party
import numpy as np
import matplotlib.patches as mpatches

# local (controlbook)
from ..common.animator import MatplotlibAxisAnimator


class CartPendulumAnimator(MatplotlibAxisAnimator):
    def setup_background(self):
        self.color = "blue"

        self.length = 1.0
        self.cart_width = self.length / 2
        self.cart_height = self.length * 0.15
        self.gap = 0.01
        self.buffer = self.length * 3

        span = 10
        (groundline,) = self.ax.plot([-span, span], [0, 0], "k--")

        z0 = self.x_hist[0, 0]
        self.ax.grid(True)
        self.ax.set_aspect("equal", "box")
        self.ax.set_xlim(z0 - self.buffer, z0 + self.buffer)
        self.ax.set_ylim(-self.buffer / 2, self.buffer)

    def setup_changing_objects(self, x0, r0):
        z0, theta0 = x0[:2]
        bottom_left_corner = self.get_cart_corner(z0)
        self.cart = mpatches.Rectangle(
            xy=bottom_left_corner,
            width=self.cart_width,
            height=self.cart_height,
            rotation_point="xy",
            facecolor=self.color,
            edgecolor="black",
        )
        self.ax.add_patch(self.cart)

        x, y = self.get_pendulum_data(z0, theta0)
        (self.pendulum,) = self.ax.plot(x, y, lw=1, c=self.color)
        (self.bob,) = self.ax.plot(x[-1:], y[-1:], "o", c=self.color)

    def update_changing_objects(self, x, r):
        z, theta = x[:2]

        bottom_left_corner = self.get_cart_corner(z)
        self.cart.set_xy(bottom_left_corner)

        x, y = self.get_pendulum_data(z, theta)
        self.pendulum.set_data(x, y)
        self.bob.set_data(x[-1:], y[-1:])
        return (self.cart, self.pendulum, self.bob)

    def get_cart_corner(self, z):
        bottom_left = (z - self.cart_width / 2, self.gap)
        return bottom_left

    def get_pendulum_data(self, z, theta):
        bottom = self.gap + self.cart_height
        x = [z, z + self.length * np.sin(theta)]
        y = [bottom, bottom + self.length * np.cos(theta)]
        return x, y
