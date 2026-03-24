# 3rd-party
import numpy as np

# local (controlbook)
from . import params as P
from ..common import ControllerBase
from ..control import utils_design
from ..control.state_space_control import StateSpaceController


class VTOLControllerSSI(ControllerBase):
    def __init__(self, use_single_system: bool = False):
        # tuning parameters
        if use_single_system:
            tr_h = 5.0
            zeta_h = 0.9
            tr_theta = 0.5
            zeta_theta = 0.9
            M = 10.0
            tr_z = tr_theta * M
            zeta_z = 0.9
        else:
            tr_h = 2.0
            zeta_h = 0.95
            tr_theta = 0.4
            zeta_theta = 0.9
            M = 10.0
            tr_z = tr_theta * M
            zeta_z = 0.9

        # desired poles
        des_h_poles = utils_design.get_2nd_order_poles(tr_h, zeta_h)
        des_theta_poles = utils_design.get_2nd_order_poles(tr_theta, zeta_theta)
        des_z_poles = utils_design.get_2nd_order_poles(tr_z, zeta_z)

        des_poles_lat = np.hstack([des_theta_poles, des_z_poles])
        des_poles = np.hstack([des_h_poles, des_poles_lat])

        # controllers
        self.lon_ss_ctrl = StateSpaceController(
            P.A_lon, P.B_lon, P.Cr_lon, des_h_poles, use_kr=True
        )
        self.lat_ss_ctrl = StateSpaceController(
            P.A_lat, P.B_lat, P.Cr_lat, des_poles_lat, use_kr=True
        )
        self.full_ss_ctrl = StateSpaceController(
            P.A, P.B, P.Cr, des_poles, use_kr=True
        )

        # equilibrium points
        self.x_lon_eq = P.x_lon_eq
        self.r_lon_eq = P.Cr_lon @ self.x_lon_eq

        self.x_lat_eq = P.x_lat_eq
        self.r_lat_eq = P.Cr_lat @ self.x_lat_eq

        self.x_eq = P.x_eq
        self.r_eq = P.Cr @ self.x_eq
        self.u_eq = P.u_eq

        self.use_single_system = use_single_system

        # NEW: integrators
        self.integrator_h = 0.0
        self.integrator_z = 0.0

        self.ki_h = 0.1   # tune this
        self.ki_z = 0.05   # tune this

    def update_with_state(self, r, x):

        if self.use_single_system:
            x_tilde = x - self.x_eq
            r_tilde = r - self.r_eq
            u_tilde = self.full_ss_ctrl.update(r_tilde, x_tilde)

        else:
            # LONGITUDINAL (h control)
            x_lon = x[[1, 4]]
            r_lon = r[[1]]

            x_lon_tilde = x_lon - self.x_lon_eq
            r_lon_tilde = r_lon - self.r_lon_eq

            # integrator (h)
            error_h = r[1] - x[1]
            self.integrator_h += P.ts * error_h

            F_tilde = self.lon_ss_ctrl.update(r_lon_tilde, x_lon_tilde) \
                      + self.ki_h * self.integrator_h

            # LATERAL (z control)
            x_lat = x[[0, 2, 3, 5]]
            r_lat = r[[0]]

            x_lat_tilde = x_lat - self.x_lat_eq
            r_lat_tilde = r_lat - self.r_lat_eq

            # integrator (z)
            error_z = r[0] - x[0]
            self.integrator_z += P.ts * error_z

            tau_tilde = self.lat_ss_ctrl.update(r_lat_tilde, x_lat_tilde) \
                        + self.ki_z * self.integrator_z

            # combine
            u_FT_tilde = np.hstack([F_tilde, tau_tilde])
            u_tilde = P.mixer @ u_FT_tilde

        # SATURATION + ANTI-WINDUP
        u_unsat = u_tilde + self.u_eq
        u = self.saturate(u_unsat, u_max=P.fr_max, u_min=0.0)

        # anti-windup (simple)
        u_error = u - u_unsat
        self.integrator_h += P.ts * u_error[0]
        self.integrator_z += P.ts * u_error[1]

        return u

    def update_with_measurement(self, r, y):
        raise NotImplementedError("This controller only uses state feedback.")
