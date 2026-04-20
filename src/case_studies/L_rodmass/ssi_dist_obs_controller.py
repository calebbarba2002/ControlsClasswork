# 3rd-party
import numpy as np
import control as cnt

# local (controlbook)
from . import params as P
from ..common import ControllerBase


class RodMassSSIDOController(ControllerBase):
    def __init__(self):

        # =========================================
        # CONTROLLER (same as before)
        # =========================================
        tr = 0.5
        zeta = 0.9
        integrator_pole = -10.0

        A1 = np.block([
            [P.A, np.zeros((2,1))],
            [-P.Cr, np.zeros((1,1))]
        ])
        B1 = np.vstack((P.B, 0))

        wn = 2.2 / tr
        poles = np.roots([1, 2*zeta*wn, wn**2])
        des_poles = np.hstack([poles, integrator_pole])

        K1 = cnt.place(A1, B1, des_poles)
        self.K = K1[:, :2]
        self.ki = K1[:, 2]

        # =========================================
        # OBSERVER
        # =========================================
        obs_poles = 5 * poles   # 5x faster
        self.L = cnt.place(P.A.T, P.Cm.T, obs_poles).T

        # =========================================
        # DISTURBANCE OBSERVER
        # =========================================
        self.p_dist = -1.0

        A2 = np.block([
            [P.A, P.B],
            [np.zeros((1,2)), self.p_dist]
        ])
        C2 = np.hstack([P.Cm, np.zeros((1,1))])

        obs_poles2 = np.hstack([obs_poles, self.p_dist])
        self.L2 = cnt.place(A2.T, C2.T, obs_poles2).T

        print("L =", self.L)
        print("L2 =", self.L2)

        # =========================================
        # STATES
        # =========================================
        self.xhat = np.zeros(2)
        self.dhat = 0.0
        self.integrator = 0.0
        self.error_prev = 0.0

    def update_with_measurement(self, r, y):

        # =========================================
        # OBSERVER UPDATE
        # =========================================
        y_error = y - P.Cm @ self.xhat
        xhat_dot = P.A @ self.xhat + P.B.flatten()*0 + self.L @ y_error
        self.xhat += P.ts * xhat_dot

        # =========================================
        # DISTURBANCE UPDATE
        # =========================================
        self.dhat += P.ts * self.p_dist * (y - P.Cm @ self.xhat)

        # =========================================
        # CONTROL LAW
        # =========================================
        error = r - self.xhat[0]
        self.integrator += P.ts * (error + self.error_prev)/2
        self.error_prev = error

        u = -self.K @ self.xhat - self.ki*self.integrator - self.dhat

        return u, self.xhat

