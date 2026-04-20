# 3rd-party
import numpy as np
import control as cnt

# local (controlbook)
from . import params as P
from ..common import ControllerBase


class RodMassLQRController(ControllerBase):
    def __init__(self):

        # =========================================
        # 1. AUGMENT SYSTEM
        # =========================================
        A1 = np.block([
            [P.A, np.zeros((2,1))],
            [-P.Cr, np.zeros((1,1))]
        ])
        B1 = np.vstack((P.B, 0))

        # =========================================
        # 2. CHOOSE Q AND R
        # =========================================
        Q = np.diag([10, 1, 50])   # state + integrator
        R = np.array([[0.1]])

        print("Q =", Q)
        print("R =", R)

        # =========================================
        # 3. LQR SOLUTION
        # =========================================
        K1, _, eigVals = cnt.lqr(A1, B1, Q, R)

        self.K = K1[:, :2]
        self.ki = K1[:, 2]

        print("LQR poles =", eigVals)

        # =========================================
        # OBSERVER (ONLY for 2-state system)
        # =========================================

        # Use ONLY the first 2 poles (system poles, not integrator)
        obs_poles = 5 * np.real(eigVals[:2])   # take 2 poles only

        self.L = cnt.place(P.A.T, P.Cm.T, obs_poles).T


        # =========================================
        # STATES
        # =========================================
        self.xhat = np.zeros(2)
        self.integrator = 0.0
        self.error_prev = 0.0

    def update_with_measurement(self, r, y):

        # observer
        u_prev = getattr(self, "u_prev", np.array([0.0]))
        y_error = y - P.Cm @ self.xhat

        self.xhat += P.ts * (
            P.A @ self.xhat + P.B @ u_prev + self.L @ y_error
        )

        # integrator
        error = r - self.xhat[0]
        self.integrator += P.ts * (error + self.error_prev)/2
        self.error_prev = error

        # control
        u = -self.K @ self.xhat - self.ki * self.integrator

        # save control for observer
        self.u_prev = u

        return np.array([u.item()]), self.xhat



