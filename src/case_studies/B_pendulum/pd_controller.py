# 3rd-party
import numpy as np

# local (controlbook)
# TODO: decide whether to use global vs relative imports for public case studies
# (it might depend on what you want to show up in the book)
from case_studies import common
from case_studies.B_pendulum import params as P

# from .. import common
# from . import params as P


class CartPendulumControllerPD(common.ControllerBase):
    def __init__(self):
        # tuning parameters
        tr_theta = 0.5  # original
        tr_theta = 0.15  # tuned to saturate for single time step
        zeta_theta = 0.707
        M = 10  # time separation factor between inner and outer loop
        tr_z = tr_theta * M
        zeta_z = 0.707

        # system parameters
        a1_inner, a0_inner = P.tf_inner_den[-2:]
        b0_inner = P.tf_inner_num[-1]
        b2_outer, _, b0_outer = P.tf_outer_num

        # Inner loop
        wn_theta = 2.2 / tr_theta
        alpha0_inner = wn_theta**2
        alpha1_inner = 2 * zeta_theta * wn_theta
        self.kp_theta = (alpha0_inner - a0_inner) / b0_inner
        self.kd_theta = (alpha1_inner - a1_inner) / b0_inner
        print(f"Inner loop (theta): {self.kp_theta = :.3f}, {self.kd_theta = :.3f}")

        # DC gain of inner loop
        DC_gain = (b0_inner * self.kp_theta) / (a0_inner + b0_inner * self.kp_theta)
        print(f"{DC_gain = :.3f}")

        # Outer loop
        wn_z = 2.2 / tr_z
        zero_LHP = -np.sqrt(b0_outer / -b2_outer)
        a = wn_z**2 / zero_LHP  # eq 8.12 from controlbook
        b = (a - 2 * zeta_z * wn_z) / -zero_LHP  # eq 8.13 from controlbook
        self.kd_z = b / (1 - b)
        self.kp_z = a * (1 + self.kd_z)
        print(f"Outer loop (z): {self.kp_z = :.3f}, {self.kd_z = :.3f}")

        self.filter = ZeroCancelingFilter(zero_LHP, DC_gain)

    def update_with_state(self, r, x):
        z_ref = r[0]
        z, theta, zdot, thetadot = x

        # outer loop control
        error_z = z_ref - z
        theta_ref_pre = self.kp_z * error_z - self.kd_z * zdot

        # low-pass filter the reference angle to cancel the zero and DC gain
        theta_ref = self.filter.update(theta_ref_pre)
        r[1] = theta_ref  # if you want to visualize the "reference" angle

        # inner loop control
        error_theta = theta_ref - theta
        F = self.kp_theta * error_theta - self.kd_theta * thetadot
        u_unsat = np.array([F])
        u = self.saturate(u_unsat, u_max=P.force_max)
        return u


class ZeroCancelingFilter:
    def __init__(self, zero_LHP, DC_gain):
        # TODO: book uses b for numerator coeffcients and a for denominator
        # coefficients for all of the system transfer functions, so why does it
        # switch here? (I switched them so b is on top...is that OK?)

        # system parameters
        sys_b2 = P.tf_outer_num[0]

        # low-pass filter parameters
        self.b0 = 1 / (sys_b2 * DC_gain)
        self.a0 = -zero_LHP
        self.state = 0.0

    def update(self, input_):
        # Euler (RK1) integration
        state_dot = -self.a0 * self.state + self.b0 * input_
        self.state = self.state + state_dot * P.ts
        return self.state
