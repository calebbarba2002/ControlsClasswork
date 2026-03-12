# 3rd-party
import numpy as np
from matplotlib.patches import Rectangle, Ellipse

# local (controlbook)
from . import params as P
from ..common.animator import MatplotlibAxisAnimator


class VTOLAnimator(MatplotlibAxisAnimator):
    def setup_background(self):
        self.body_width = P.d / 2
        self.body_height = self.body_width * 0.8
        self.rotor_width = P.d / 2.5
        self.rotor_height = P.d / 6

        span = 10.0
        buffer = span * 0.1
        self.ax.set_xlim(-buffer, span + buffer)
        self.ax.set_ylim(-buffer, span + buffer)
        self.ax.set_xlabel("z")
        self.ax.set_ylabel("h")
        self.ax.set_aspect("equal", "box")

        (groundline,) = self.ax.plot([0.0, span], [0.0, 0.0], "k")

    def setup_changing_objects(self, x0, r0):
        z0, h0, theta0 = x0[:3]
        theta0_deg = np.rad2deg(theta0)
        ztarget0 = r0[0] if r0 is not None else 0.0

        # draw target first so it is underneath the VTOL
        x, y = self.get_target_data(ztarget0)
        (self.target,) = self.ax.plot(x, y, "gs", markersize=10)

        # draw arms before body and rotors so they are underneath
        rotor_centers = self.get_rotor_centers(z0, h0, theta0)
        x, y = rotor_centers
        (self.arms,) = self.ax.plot(x, y, "b")

        patch_params = {"angle": theta0_deg, "facecolor": "blue", "edgecolor": "black"}

        self.body = Rectangle(
            self.get_body_xy(z0, h0, theta0),
            self.body_width,
            self.body_height,
            **patch_params,
        )
        self.ax.add_patch(self.body)

        left_center = rotor_centers[:, 0]
        self.left_rotor = Ellipse(
            tuple(left_center),
            width=self.rotor_width,
            height=self.rotor_height,
            **patch_params,
        )
        self.ax.add_patch(self.left_rotor)

        right_center = rotor_centers[:, 1]
        self.right_rotor = Ellipse(
            tuple(right_center),
            width=self.rotor_width,
            height=self.rotor_height,
            **patch_params,
        )
        self.ax.add_patch(self.right_rotor)

    def update_changing_objects(self, x, r):
        z, h, theta = x[:3]
        theta_deg = np.rad2deg(theta)
        ztarget = r[0] if r is not None else 0.0

        self.target.set_data(self.get_target_data(ztarget))

        self.body.set_xy(self.get_body_xy(z, h, theta))
        self.body.set_angle(theta_deg)

        rotor_centers = self.get_rotor_centers(z, h, theta)
        left_center = rotor_centers[:, 0]
        right_center = rotor_centers[:, 1]
        self.arms.set_data(rotor_centers)
        self.left_rotor.set_center(tuple(left_center))
        self.left_rotor.set_angle(theta_deg)
        self.right_rotor.set_center(tuple(right_center))
        self.right_rotor.set_angle(theta_deg)

        # return target and arms first so they are drawn underneath the rest
        return self.target, self.arms, self.body, self.left_rotor, self.right_rotor

    def get_body_xy(self, z, h, theta):
        bottom_left_corner = np.array([-0.5 * self.body_width, -0.5 * self.body_height])
        R = self.get_rotation_matrix(theta)
        rotated_corner = R @ bottom_left_corner
        center = np.array([z, h])
        x, y = center + rotated_corner
        return x, y

    def get_rotor_centers(self, z, h, theta):
        body_center = np.array([[z, h]]).T
        rotor_pts = np.array([[-P.d, P.d], [0, 0]])
        R = self.get_rotation_matrix(theta)
        rotated_rotor_centers = body_center + R @ rotor_pts
        return rotated_rotor_centers

    def get_target_data(self, ztarget):
        # drawing target using a square marker on a line object with 1 point
        return [ztarget], [0.0]

    def get_rotation_matrix(self, theta):
        c = np.cos(theta)
        s = np.sin(theta)
        return np.array([[c, -s], [s, c]])
