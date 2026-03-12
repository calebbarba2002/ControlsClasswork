# 3rd-party
import numpy as np
import matplotlib.patches as mpatches

# local (controlbook)
from ..common.animator import MatplotlibAxisAnimator


class SatelliteAnimator(MatplotlibAxisAnimator):
    def setup_background(self):
        self.length = 1.0
        self.width = 0.3

        span = self.length * 2
        (horizontal_line,) = self.ax.plot([-span, span], [0, 0], "k--")

        self.ax.set_aspect("equal", "box")
        self.ax.set_xlim(-span, span)
        self.ax.set_ylim(-span, span)

        # define points for custom polygon shape for the satellite base
        self.base_pts = np.array(
            [
                [self.width / 2.0, -self.width / 2.0],
                [self.width / 2.0, -self.width / 6.0],
                [self.width / 2.0 + self.width / 6.0, -self.width / 6.0],
                [self.width / 2.0 + self.width / 6.0, self.width / 6.0],
                [self.width / 2.0, self.width / 6.0],
                [self.width / 2.0, self.width / 2.0],
                [-self.width / 2.0, self.width / 2.0],
                [-self.width / 2.0, self.width / 6.0],
                [-self.width / 2.0 - self.width / 6.0, self.width / 6.0],
                [-self.width / 2.0 - self.width / 6.0, -self.width / 6.0],
                [-self.width / 2.0, -self.width / 6.0],
                [-self.width / 2.0, -self.width / 2.0],
            ]
        ).T
        # define points for the satellite panels
        self.panel_pts = np.array([[-self.length, self.length], [0.0, 0.0]])

    def setup_changing_objects(self, x0, r0):
        theta0, phi0 = x0[:2]

        # create the base of the satellite
        self.base = mpatches.Polygon(
            xy=self.get_base_xy(theta0),
            facecolor="blue",
            edgecolor="black",
        )
        self.ax.add_patch(self.base)

        # create the satellite panels
        x, y = self.get_panel_data(phi0)
        (self.panel,) = self.ax.plot(x, y, lw=4, c="green")

    def update_changing_objects(self, x, r):
        theta, phi = x[:2]
        self.base.set_xy(self.get_base_xy(theta))
        self.panel.set_data(self.get_panel_data(phi))
        return (self.base, self.panel)

    def get_base_xy(self, theta):
        """
        Return the base coordinates of the satellite in the rotated frame.
        """
        R = self.get_rotation_matrix(theta)
        rotated_base_pts = R @ self.base_pts
        return rotated_base_pts.T

    def get_panel_data(self, phi):
        """
        Return the panel coordinates of the satellite in the rotated frame.
        """
        R = self.get_rotation_matrix(phi)
        rotated_panel_pts = R @ self.panel_pts
        x, y = rotated_panel_pts
        return x, y

    def get_rotation_matrix(self, angle):
        """
        Return the 2D rotation matrix for a given angle in radians. Note that
        the minus sign is used on the lower left sin term because theta and phi
        are defined as positive angles in the clockwise direction.
        """
        c = np.cos(angle)
        s = np.sin(angle)
        return np.array([[c, s], [-s, c]])
