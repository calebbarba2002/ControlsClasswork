# 3rd-party
import numpy as np

# local (controlbook)
from . import params as P
from ..common import ControllerBase
from ..control import utils_design
from ..control.state_space_control import StateSpaceController

class VTOLControllerSS(ControllerBase):
    def __init__(self, use_single_system: bool = False):
        # tuning parameters
        # NOTE: when using a single system, you can't control which poles go where
        # i.e., what you labeled as h_poles might end up controlling z instead.
        # This makes it different to tune.
        if use_single_system:
            tr_h = 5.0
            zeta_h = 0.9
            tr_theta = 0.5
            zeta_theta = 0.9
            M = 10.0 # time scale separation between inner and outer loop
            tr_z = tr_theta * M
            zeta_z = 0.9
        else:
            tr_h = 2.0
            zeta_h = 0.95
            tr_theta = 0.4
            zeta_theta = 0.9
            M = 10.0 # time scale separation between inner and outer loop
            tr_z = tr_theta * M
            zeta_z = 0.9

        # desired closed-loop poles
        des_h_poles = utils_design.get_2nd_order_poles(tr_h, zeta_h)
        des_theta_poles = utils_design.get_2nd_order_poles(tr_theta, zeta_theta)
        des_z_poles = utils_design.get_2nd_order_poles(tr_z, zeta_z)

        des_poles_lat = np.hstack([des_theta_poles, des_z_poles])
        des_poles = np.hstack([des_h_poles, des_poles_lat])

        if not use_single_system:
            print("Longitudinal gain matrices:")
        self.lon_ss_ctrl = StateSpaceController(
            P.A_lon,
            P.B_lon,
            P.Cr_lon,
            des_h_poles,
            use_kr=True,
            print_gains=not use_single_system,
        )
        if not use_single_system:
            print("Lateral gain matrices:")
        self.lat_ss_ctrl = StateSpaceController(
            P.A_lat,
            P.B_lat,
            P.Cr_lat,
            des_poles_lat,
            use_kr=True,
            print_gains=not use_single_system,
        )
        if use_single_system:
            print("Combined (full) gain matrices:")
        self.full_ss_ctrl = StateSpaceController(
            P.A, P.B, P.Cr, des_poles, use_kr=True, print_gains=use_single_system
        )

        # linearization point
        self.x_lon_eq = P.x_lon_eq
        self.r_lon_eq = P.Cr_lon @ self.x_lon_eq

        self.x_lat_eq = P.x_lat_eq
        self.r_lat_eq = P.Cr_lat @ self.x_lat_eq

        self.x_eq = P.x_eq
        self.r_eq = P.Cr @ self.x_eq
        self.u_eq = P.u_eq

        self.use_single_system = use_single_system

    def update_with_state(self, r, x):
        if self.use_single_system:
            x_tilde = x - self.x_eq
            r_tilde = r - self.r_eq
            u_tilde = self.full_ss_ctrl.update(r_tilde, x_tilde)
        else:
        # longitudinal (altitude) control
            x_lon = x[[1, 4]]
            r_lon = r[[1]]
            x_lon_tilde = x_lon - self.x_lon_eq
            r_lon_tilde = r_lon - self.r_lon_eq
            F_tilde = self.lon_ss_ctrl.update(r_lon_tilde, x_lon_tilde)

            # lateral (side-to-side) control
            x_lat = x[[0, 2, 3, 5]]
            r_lat = r[[0]]
            x_lat_tilde = x_lat - self.x_lat_eq
            r_lat_tilde = r_lat - self.r_lat_eq
            tau_tilde = self.lat_ss_ctrl.update(r_lat_tilde, x_lat_tilde)

            # combine
            u_FT_tilde = np.hstack([F_tilde, tau_tilde])

            # convert [F, tau] to motor forces
            u_tilde = P.mixer @ u_FT_tilde

        u_unsat = u_tilde + self.u_eq
        u = self.saturate(u_unsat, u_max=P.fr_max, u_min=0.0)
        return u

    # TODO: book doesn't mention controlling w/ measurement, but mass does it
    def update_with_measurement(self, r, y):
        raise NotImplementedError("This controller only uses state feedback.")