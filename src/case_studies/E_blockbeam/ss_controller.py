# 3rd-party
import numpy as np

# local (controlbook)
from . import params as P
from ..common import ControllerBase
from ..control import utils_design
from ..control.state_space_control import StateSpaceController

class BlockbeamSSController(ControllerBase):
    def __init__(self):
        # controller design parameters
        tr_theta = 0.18 # tuned value from chapter 8
        zeta_theta = 0.707
        tr_z = 1.2 # tuned value
        zeta_z = 0.707

        # create controller
        des_theta_poles = utils_design.get_2nd_order_poles(tr_theta, zeta_theta)
        des_z_poles = utils_design.get_2nd_order_poles(tr_z, zeta_z)
        des_poles = np.hstack([des_theta_poles, des_z_poles])
        self.ss_ctrl = StateSpaceController(
            P.A, P.B, P.Cr, des_poles, use_kr=True, print_gains=True
        )

        # linearization point
        self.x_eq = P.x_eq
        self.r_eq = P.Cr @ self.x_eq
        self.u_eq = P.u_eq

    def update_with_state(self, r, x):
        x_tilde = x - self.x_eq
        r_tilde = r - self.r_eq
        u_tilde = self.ss_ctrl.update(r_tilde, x_tilde)
        u_unsat = u_tilde + self.u_eq
        u = self.saturate(u_unsat, u_max=P.force_max)
        return u
   
    # TODO: book doesn't mention controlling w/ measurement, but mass does it
    def update_with_measurement(self, r, y):
        raise NotImplementedError("This controller only uses state feedback.")