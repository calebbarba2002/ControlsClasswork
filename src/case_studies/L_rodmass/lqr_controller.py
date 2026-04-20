# 3rd-party
import numpy as np
import control as cnt

# local (controlbook)
from . import params as P
from ..common import ControllerBase


class RodMassLQRController(ControllerBase):
    def __init__(self):

        # =========================================
        # AUGMENT SYSTEM
        # =========================================
        A1 = np.block([
            [P.A, np.zeros((2,1))],
            [-P.Cr, np.zeros((1,1))]
        ])
        B1 = np.vstack((P.B, 0))

        # =========================================
        # LQR WEIGHTS
        # =========================================
        Q = np.diag([200, 1, 100])   # heavier penalty on theta tracking
        R = np.array([[5.0]])         # heavier penalty on control effort

        print("Q =", Q)
        print("R =", R)

        # =========================================
        # LQR SOLUTION
        # =========================================
        K1, _, eigVals = cnt.lqr(A1, B1, Q, R)

        self.K = K1[:, :2].flatten()       # 1D (2,)
        self.ki = float(K1[0, 2])          # scalar

        print("LQR poles =", eigVals)

        # =========================================
        # DISTURBANCE OBSERVER (AUGMENTED)
        # =========================================
        p_dist = -1.0

        obs_poles = 5 * eigVals[1:]        # complex conjugate pair

        self.A2 = np.block([
            [P.A,              P.B                        ],
            [np.zeros((1, 2)), np.array([[p_dist]])]
        ])
        self.B2 = np.vstack([P.B, np.zeros((1, 1))])
        C2 = np.hstack([P.Cm, np.zeros((1, 1))])

        obs_poles2 = np.hstack([obs_poles, p_dist])
        self.L2 = cnt.place(self.A2.T, C2.T, obs_poles2).T

        print("L2 =", self.L2)

        # =========================================
        # STATES - all 1D or scalar from the start
        # =========================================
        self.xhat = np.zeros(2)            # 1D (2,)
        self.dhat = 0.0
        self.integrator = 0.0
        self.u_prev = 0.0                  # scalar

        self.x_eq = P.x_eq.flatten()      # 1D
        self.u_eq = float(P.u_eq.item())   # scalar
        self.r_eq = float((P.Cr @ P.x_eq).item())  # scalar

    def update_with_measurement(self, r, y):

        # =========================================
        # AUGMENTED OBSERVER UPDATE
        # =========================================
        xhat_aug = np.append(self.xhat, self.dhat)    # 1D (3,)

        y_error = y - P.Cm @ self.xhat                # innovation

        xhat_aug += P.ts * (
            self.A2 @ xhat_aug
            + self.B2.flatten() * self.u_prev
            + self.L2 @ y_error
        ).flatten()

        # Unpack
        self.xhat = xhat_aug[:2]
        self.dhat = float(xhat_aug[2])

        # =========================================
        # TILDE VARIABLES
        # =========================================
        x_tilde = self.xhat - self.x_eq               # 1D (2,)
        r_tilde = float(r.item()) - self.r_eq
        y_tilde = float((P.Cr @ x_tilde).item())

        # =========================================
        # INTEGRATOR
        # =========================================
        error = r_tilde - y_tilde
        self.integrator += P.ts * error

        # =========================================
        # CONTROL LAW
        # =========================================
        u_tilde = float(-self.K @ x_tilde) - self.ki * self.integrator - self.dhat
        u = u_tilde + self.u_eq

        self.u_prev = u

        return np.array([u]), self.xhat.copy()