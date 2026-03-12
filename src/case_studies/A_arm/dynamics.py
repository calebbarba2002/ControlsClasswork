# 3rd-party
import numpy as np

# local (controlbook)
from . import params as P
from ..common.dynamics_base import DynamicsBase


class ArmDynamics(DynamicsBase):
    def __init__(self, alpha=0.0):
        super().__init__(
            # Initial state conditions
            state0=np.array([P.theta0, P.thetadot0]),
            # Input torque limits
            u_max=P.tau_max,
            u_min=-P.tau_max,
            # Time step for integration
            dt=P.ts,
        )
        # see params.py/textbook for details on these parameters
        self.m = self.randomize_parameter(P.m, alpha)
        self.ell = self.randomize_parameter(P.ell, alpha)
        self.b = self.randomize_parameter(P.b, alpha)
        self.g = P.g  # gravity constant is well known, so not randomized

    def f(self, x, u):
        # Return xdot = f(x,u), the system state update equations
        # re-label states for readability
        theta, thetadot = x
        torque = u[0]

        inertia = self.m * self.ell**2 / 3
        friction = self.b * thetadot
        weight_moment = self.m * self.g * self.ell / 2.0 * np.cos(theta)

        thetaddot = (torque - friction - weight_moment) / inertia
        xdot = np.array([thetadot, thetaddot])
        return xdot

    def h(self):
        # return the output equations
        # could also use input u if needed
        theta = self.state[0]
        y = np.array([theta])
        return y
