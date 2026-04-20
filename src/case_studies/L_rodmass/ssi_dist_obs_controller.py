# 3rd-party
import numpy as np
import control as cnt

# local (controlbook)
from . import params as P
from ..common import ControllerBase


class RodMassSSIDOController(ControllerBase):
    def __init__(self):

        # =========================================
        # CONTROLLER DESIGN
        # =========================================
        tr = 0.15
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
        self.K = K1[:, :2].flatten()       # force to 1D (2,)
        self.ki = float(K1[0, 2])          # index specific element instead of slicing

        # =========================================
        # OBSERVER
        # =========================================
        obs_poles = 5 * poles
        self.L = cnt.place(P.A.T, P.Cm.T, obs_poles).T

        # =========================================
        # DISTURBANCE OBSERVER (AUGMENTED)
        # =========================================
        p_dist = -1.0

        self.A2 = np.block([
            [P.A,              P.B                       ],
            [np.zeros((1, 2)), np.array([[p_dist]])]
        ])
        self.B2 = np.vstack([P.B, np.zeros((1, 1))])
        C2 = np.hstack([P.Cm, np.zeros((1, 1))])

        obs_poles2 = np.hstack([obs_poles, p_dist])
        self.L2 = cnt.place(self.A2.T, C2.T, obs_poles2).T

        print("L =", self.L)
        print("L2 =", self.L2)

        # =========================================
        # STATES
        # =========================================
        self.xhat = np.zeros(2)            # 1D (2,)
        self.dhat = 0.0
        self.integrator = 0.0
        self.u_prev = 0.0                  # plain scalar

        self.x_eq = P.x_eq.flatten()      # force 1D
        self.u_eq = float(P.u_eq.item())   # .item() extracts scalar from any shape array
        self.r_eq = float((P.Cr @ self.x_eq).item())

    def update_with_measurement(self, r, y):

        # =========================================
        # AUGMENTED OBSERVER UPDATE
        # =========================================
        xhat_aug = np.append(self.xhat, self.dhat)   # 1D (3,)

        y_error = y - P.Cm @ self.xhat               # innovation

        xhat_aug += P.ts * (
            self.A2 @ xhat_aug
            + self.B2.flatten() * self.u_prev
            + self.L2 @ y_error
        ).flatten()

        # Unpack updated estimates
        self.xhat = xhat_aug[:2]
        self.dhat = float(xhat_aug[2])

        # =========================================
        # TILDE VARIABLES
        # =========================================
        x_tilde = self.xhat - self.x_eq             # 1D (2,)
        r_tilde = float(r.item()) - self.r_eq   # r is 1D (1,), so extract scalar with .item()
        y_tilde = float((P.Cr @ x_tilde).item())


        # =========================================
        # INTEGRATOR
        # =========================================
        error = r_tilde - y_tilde
        self.integrator += P.ts * error

        # =========================================
        # CONTROL LAW
        # =========================================
        u_tilde = float((-self.K @ x_tilde).item()) - self.ki * self.integrator - self.dhat
        u = u_tilde + self.u_eq

        self.u_prev = u

        return np.array([u]), self.xhat.copy()