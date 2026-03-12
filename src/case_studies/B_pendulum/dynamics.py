# 3rd-party
import numpy as np

# local (controlbook)
from . import params as P
from ..common.dynamics_base import DynamicsBase


class CartPendulumDynamics(DynamicsBase):
    def __init__(self, alpha=0.0):
        super().__init__(
            # Initial state conditions
            state0=np.array([P.z0, P.theta0, P.zdot0, P.thetadot0]),
            # Input force limits
            u_max=P.force_max,
            u_min=-P.force_max,
            # Time step for integration
            dt=P.ts,
        )
        # see params.py/textbook for details on these parameters

        # parameter randomization is used after Chapter TODO:
        # you do not need to include it before then
        self.m1 = self.randomize_parameter(P.m1, alpha)  # pendulum
        self.m2 = self.randomize_parameter(P.m2, alpha)  # cart
        self.ell = self.randomize_parameter(P.ell, alpha)
        self.b = self.randomize_parameter(P.b, alpha)
        self.g = P.g  # gravity constant is well known, so not randomized

    def f(self, x, u):
        """
        Return xdot = f(x,u).
        """
        # re-label states and inputs for readability
        z, theta, zdot, thetadot = x
        force = u[0]

        # TODO: decide whether to label terms like this
        moment_arm = self.m1 * self.ell / 2.0
        mass = self.m1 + self.m2
        rotational_inertia = self.m1 * self.ell**2 / 3

        M12 = moment_arm * np.cos(theta)
        M = np.array([[mass, M12], # fmt: skip
                      [M12, rotational_inertia]]) # fmt: skip

        friction = self.b * zdot
        weight_moment = moment_arm * self.g * np.sin(theta)
        tmp = moment_arm * thetadot**2 * np.sin(theta)
        c = np.array([tmp + force - friction, weight_moment])

        zddot, thetaddot = np.linalg.solve(M, c)

        xdot = np.array([zdot, thetadot, zddot, thetaddot])
        return xdot

    def h(self):
        """
        Return the output y = h(x).
        """
        # y = P.Cm @ self.state
        y = self.state[:2].copy()  # measure position states only
        return y
