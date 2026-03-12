# 3rd-party
import numpy as np
# local (controlbook)
from ..common.dynamics_base import DynamicsBase
from . import params as P

class MassDynamics(DynamicsBase):
    def __init__(self, alpha=0.0):
        super().__init__(
            state0=np.array([P.z0, P.zdot0]),
            u_max=P.force_max,
            u_min=-P.force_max,
            dt=P.ts,
        )
        self.m = self.randomize_parameter(P.m, alpha)
        self.k = self.randomize_parameter(P.k, alpha)
        self.b = self.randomize_parameter(P.b, alpha)

    def f(self, x, u):
        z, zdot = x.flatten()
        force = u.flatten()[0]

        zddot = (force - self.k * z - self.b * zdot) / self.m

        xdot = np.array([zdot, zddot])
        return xdot

    def h(self):
        y = self.state[:,1]
        return y