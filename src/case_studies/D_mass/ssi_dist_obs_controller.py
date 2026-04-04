# 3rd-party
import numpy as np

# local (controlbook)
from . import params as P
from ..common import ControllerBase
from ..control import utils_design
from ..control.observer import Observer


class MassSSIDOController(ControllerBase):
    def __init__(self):
        # -------------------------------------------------------
        # Same controller design as D.13 — manual SSI
        # -------------------------------------------------------
        tr_z = 1.0
        zeta_z = 0.707
        integrator_poles = [-2.0]

        import control as cnt
        sys_poles = utils_design.get_2nd_order_poles(tr_z, zeta_z)
        des_poles = np.hstack([sys_poles, integrator_poles])

        # augmented system for integral control
        A1 = np.block([[P.A, np.zeros((2, 1))], [-P.Cr, np.zeros(1)]])
        B1 = np.vstack((P.B, 0))
        self.K1 = cnt.place(A1, B1, des_poles)
        self.K = self.K1[:, :2]
        self.ki = self.K1[:, 2:]

        # integrator state
        self.error_integral = 0.0
        self.error_prev = 0.0

        # -------------------------------------------------------
        # State observer — same as D.13
        # -------------------------------------------------------
        tr_z_obs = tr_z / 10
        zeta_z_obs = 0.9
        obs_poles = utils_design.get_2nd_order_poles(tr_z_obs, zeta_z_obs)
        self.observer = Observer(
            P.A, P.B, P.Cm, obs_poles, P.ts, print_gain=True
        )

        # -------------------------------------------------------
        # Disturbance observer — augmented system
        # -------------------------------------------------------
        n = P.A.shape[0]

        A_aug = np.block([
            [P.A,               P.B              ],
            [np.zeros((1, n)),  np.zeros((1, 1)) ]
        ])
        B_aug = np.vstack([P.B, np.zeros((1, 1))])
        C_aug = np.hstack([P.Cm, np.zeros((P.Cm.shape[0], 1))])

        tr_d_obs = tr_z / 15
        zeta_d_obs = 0.9
        dist_obs_poles_2 = utils_design.get_2nd_order_poles(tr_d_obs, zeta_d_obs)
        dist_obs_poles = np.hstack([dist_obs_poles_2, [-tr_z / 15 * 3]])

        self.dist_observer = Observer(
            A_aug, B_aug, C_aug, dist_obs_poles, P.ts, print_gain=True
        )

        # save linearization info
        self.x_eq = P.x_eq
        self.r_eq = P.Cr @ self.x_eq
        self.y_eq = P.Cm @ self.x_eq
        self.u_eq = P.u_eq
        self.u_tilde_prev = np.zeros(1)
        self.dhat = np.zeros(1)

    def update_with_state(self, r, x):
        raise NotImplementedError("This controller uses measurement feedback only.")

    def update_with_measurement(self, r, y):
        # update state observer
        y_tilde = y - self.y_eq
        xhat_tilde = self.observer.update(y_tilde, self.u_tilde_prev)
        xhat = xhat_tilde + self.x_eq

        # update disturbance observer
        xhat_aug_tilde = self.dist_observer.update(y_tilde, self.u_tilde_prev)
        self.dhat = xhat_aug_tilde[2:3]

        # integrate error manually
        x_tilde = xhat - self.x_eq
        r_tilde = r - self.r_eq
        error = r_tilde - P.Cr @ x_tilde
        self.error_integral += P.ts * (error + self.error_prev) / 2
        self.error_prev = error

        # control with disturbance feedforward
        x1_tilde = np.hstack((x_tilde, self.error_integral))
        u_tilde = -self.K1 @ x1_tilde - self.dhat

        u_unsat = u_tilde + self.u_eq
        u = self.saturate(u_unsat, u_max=P.force_max)
        self.u_tilde_prev = u - self.u_eq

        return u, xhat