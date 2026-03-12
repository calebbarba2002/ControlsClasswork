# 3rd-party
import numpy as np

# local (controlbook)
from . import params as P
from ..common.dynamics_base import DynamicsBase


class SatelliteDynamics(DynamicsBase):
    def __init__(self, alpha=0.0):
        super().__init__(
            # Initial state conditions
            state0=np.array([P.theta0, P.phi0, P.thetadot0, P.phidot0]),
            # Input limits
            u_max=P.torque_max,
            u_min=-P.torque_max,
            # Time step for integration
            dt=P.ts,
        )
        # see params.py/textbook for details on these parameters

        # parameter randomization is used after Chapter TODO:
        # you do not need to include it before then
        self.Js = self.randomize_parameter(P.Js, alpha)
        self.Jp = self.randomize_parameter(P.Jp, alpha)
        self.k = self.randomize_parameter(P.k, alpha)
        self.b = self.randomize_parameter(P.b, alpha)

    def f(self, x, u):
        """
        Return xdot = f(x,u).
        """
        # re-label states and inputs for readability
        theta, phi, thetadot, phidot = x
        tau = u[0]

        friction = self.b * (thetadot - phidot)
        spring_torque = self.k * (theta - phi)

        M = np.diag([self.Js, self.Jp])
        c = np.array([tau - friction - spring_torque, friction + spring_torque])

        thetaddot, phiddot = np.linalg.solve(M, c)

        xdot = np.array([thetadot, phidot, thetaddot, phiddot])
        return xdot

    def h(self):
        """
        Return the output y = h(x).
        """
        theta, phi = self.state[:2]
        y = np.array([theta, phi])  # measure position states only
        return y
