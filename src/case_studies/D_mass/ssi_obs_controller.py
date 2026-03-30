# 3rd-party
import numpy as np

# local (controlbook)
import case_studies.D_mass.params as P
from case_studies.common import ControllerBase
from case_studies.control import utils_design
from case_studies.control.state_space_integral_control import (
    StateSpaceIntegralController,
)
from case_studies.control.observer import Observer

class MassSSIOController(ControllerBase):
    def __init__(self):
        # controller design parameters
        tr_z = 1.0
        zeta_z = 0.707
        integrator_poles = [-2.0]

        # create controller
        sys_poles = utils_design.get_2nd_order_poles(tr_z, zeta_z)
        des_poles = np.hstack([sys_poles, integrator_poles])
        self.ssi_ctrl = StateSpaceIntegralController(P.A, P.B, P.Cr, des_poles, P.ts)

        # observer design parameters
        tr_z_obs = tr_z / 10
        zeta_z_obs = 0.9

        # create observer
        obs_poles = utils_design.get_2nd_order_poles(tr_z_obs, zeta_z_obs)
        self.observer = Observer(P.A, P.B, P.Cm, obs_poles, P.ts, print_gain=True)

        # save linearized system information
        self.x_eq = P.x_eq
        self.r_eq = P.Cr @ self.x_eq
        self.y_eq = P.Cm @ self.x_eq
        self.u_eq = P.u_eq
        self.u_tilde_prev = np.zeros(1)

    def update_with_state(self, r, x):
        raise NotImplementedError("This controller only uses measurement feedback.")

    def update_with_measurement(self, r, y):
        # update the observer with the measurement
        y_tilde = y - self.y_eq
        xhat_tilde = self.observer.update(y_tilde, self.u_tilde_prev)
        xhat = xhat_tilde + self.x_eq

        # calculate full-state feedback control
        x_tilde = xhat - self.x_eq
        r_tilde = r - self.r_eq
        u_tilde = self.ssi_ctrl.update(r_tilde, x_tilde)
        u_unsat = u_tilde + self.u_eq
        u = self.saturate(u_unsat, u_max=P.force_max)
        
        # does not saturate, so anti-windup doesn't actually do anything
        self.ssi_ctrl.add_anti_windup_saturation(u, u_unsat)
        # save the previous control input for the observer
        self.u_tilde_prev = u - self.u_eq

        return u, xhat





# # 3rd-party
# import numpy as np
# import control as cnt

# # local (controlbook)
# from . import params as P
# from ..common import ControllerBase


# class MassSSIOController(ControllerBase):
#     def __init__(self):
#         # --- Controller tuning ---
#         tr_z = 2.8   # increase this to slow down response and reduce overshoot
#         zeta_z = 0.90  # increase toward 1.0 for less overshoot (max 1.0 = no overshoot)
#         integrator_pole = [-0.9]  # move closer to 0 (e.g. -0.8) to slow integrator

#         # --- Augmented system (for integral control) ---
#         A1 = np.block([[P.A, np.zeros((2, 1))], [-P.Cr, np.zeros(1)]])
#         B1 = np.vstack((P.B, 0))

#         # check controllability
#         if np.linalg.matrix_rank(cnt.ctrb(A1, B1)) != 3:
#             raise ValueError("System not controllable")

#         # compute controller poles
#         wn_z = 2.2 / tr_z
#         z_char_poly = [1, 2 * zeta_z * wn_z, wn_z**2]
#         z_poles = np.roots(z_char_poly)
#         des_poles = np.hstack([z_poles, integrator_pole])

#         # compute gains
#         self.K1 = cnt.place(A1, B1, des_poles)
#         self.K = self.K1[:, :2]
#         self.ki = self.K1[:, 2:]
#         print("K:", self.K, "ki:", self.ki)

#         # linearization point
#         self.x_eq = P.x_eq
#         self.r_eq = P.Cr @ self.x_eq
#         self.u_eq = P.u_eq

#         # integrator variables
#         self.error_prev = 0.0
#         self.error_integral = 0.0

#         # --- Observer tuning (M times faster than controller) ---
#         M_obs = 5.0
#         tr_z_obs = tr_z / M_obs
#         zeta_z_obs = 0.95

#         # check observability
#         if np.linalg.matrix_rank(cnt.ctrb(P.A.T, P.Cm.T)) != 2:
#             raise ValueError("System not observable")

#         # compute observer gain L
#         wn_z_obs = 2.2 / tr_z_obs
#         obs_char_poly = [1, 2 * zeta_z_obs * wn_z_obs, wn_z_obs**2]
#         obs_poles = np.roots(obs_char_poly)

#         self.L = cnt.place(P.A.T, P.Cm.T, obs_poles).T
#         print("L^T:", self.L.T)

#         # observer state
#         self.xhat_tilde = np.zeros(2)
#         self.u_prev = np.zeros(1)

#     def update_with_measurement(self, r, y):
#         # update observer
#         xhat = self.observer_rk4_step(y)

#         x_tilde = xhat - self.x_eq
#         r_tilde = r - self.r_eq

#         # integrate error
#         error = r - P.Cr @ xhat
#         self.error_integral += P.ts * (error + self.error_prev) / 2
#         self.error_prev = error

#         # compute control using estimated state
#         x1_tilde = np.hstack((x_tilde, self.error_integral))
#         u_tilde = -self.K1 @ x1_tilde

#         # saturate
#         u_unsat = u_tilde + self.u_eq
#         u = self.saturate(u_unsat, u_max=P.force_max)

#         # save u for observer
#         self.u_prev = u

#         return u, xhat

#     def observer_f(self, xhat, y):
#         y_error = y - P.Cm @ xhat
#         xhat_tilde = xhat - self.x_eq
#         u_tilde = self.u_prev - self.u_eq
#         xhat_dot = P.A @ xhat_tilde + P.B @ u_tilde + self.L @ y_error
#         return xhat_dot

#     def observer_rk4_step(self, y):
#         k1 = self.observer_f(self.xhat_tilde, y)
#         k2 = self.observer_f(self.xhat_tilde + P.ts / 2 * k1, y)
#         k3 = self.observer_f(self.xhat_tilde + P.ts / 2 * k2, y)
#         k4 = self.observer_f(self.xhat_tilde + P.ts * k3, y)
#         xhat_tilde_dot = (k1 + 2 * k2 + 2 * k3 + k4) / 6
#         self.xhat_tilde += xhat_tilde_dot * P.ts
#         xhat = self.xhat_tilde + self.x_eq
#         return xhat

#     def update_with_state(self, r, x):
#         raise NotImplementedError("Use update_with_measurement for observer control.")