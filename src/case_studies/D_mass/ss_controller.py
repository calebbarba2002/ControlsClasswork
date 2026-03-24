# 3rd-party
import numpy as np

# local (controlbook)
from . import params as P
from ..common import ControllerBase
from ..control import utils_design
from ..control.state_space_control import StateSpaceController

class MassSSController(ControllerBase): #updated with integral so SSI controleler
    def __init__(self):
        # controller design parameters
        tr_z = 2.0
        tr = 1.65   # tuned to barely saturate on first step only from chapter 8
        zeta_z = 0.707

        # create controller
        des_poles = utils_design.get_2nd_order_poles(tr_z, zeta_z)
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
    
    def update_with_measurement(self, r, y):
        raise NotImplementedError("This controller only uses state feedback.")