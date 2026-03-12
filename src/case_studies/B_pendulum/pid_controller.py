# 3rd-party
import numpy as np

# local (controlbook)
from case_studies import common
from case_studies.B_pendulum import params as P


class CartPendulumControllerPID(common.ControllerBase):
    def __init__(self):
        ## control tuning parameters
        tr_theta = 0.2
        tr_theta = 0.15
        zeta_theta = 0.707
        M = 10  # time separation factor between inner and outer loop
        tr_z = tr_theta * M
        zeta_z = 0.707
        self.ki_z = -0.1  # TODO: works better with 0 ... or -0.01

        ## system parameters
        a1_inner, a0_inner = P.tf_inner_den[-2:]
        b0_inner = P.tf_inner_num[-1]
        b2_outer, _, b0_outer = P.tf_outer_num

        ## Inner loop
        wn_theta = 2.2 / tr_theta
        alpha0_inner = wn_theta**2
        alpha1_inner = 2 * zeta_theta * wn_theta
        self.kp_theta = (alpha0_inner - a0_inner) / b0_inner
        self.kd_theta = (alpha1_inner - a1_inner) / b0_inner
        print(
            f"Inner loop (theta): kp_theta = {self.kp_theta:.3f}, "
            f"kd_theta = {self.kd_theta:.3f}"
        )

        # DC gain of inner loop
        DC_gain = (b0_inner * self.kp_theta) / (a0_inner + b0_inner * self.kp_theta)
        print(f"{DC_gain = :.3f}")

        ## Outer loop
        wn_z = 2.2 / tr_z
        zero_LHP = -np.sqrt(b0_outer / -b2_outer)
        a = wn_z**2 / zero_LHP  # eq 8.12 from controlbook
        b = (a - 2 * zeta_z * wn_z) / -zero_LHP  # eq 8.13 from controlbook
        self.kd_z = b / (1 - b)
        self.kp_z = a * (1 + self.kd_z)
        print(
            f"Outer loop (z): kp_z = {self.kp_z:.3f}, ki_z = {self.ki_z}, "
            f"kd_z = {self.kd_z:.3f}"
        )

        self.filter = ZeroCancelingFilter(zero_LHP, DC_gain)

        # dirty derivative variables
        self.sigma = 0.05
        self.beta = (2 * self.sigma - P.ts) / (2 * self.sigma + P.ts)
        self.thetadot_hat = P.thetadot0
        self.theta_prev = P.theta0
        self.zdot_hat = P.zdot0
        self.z_prev = P.z0

        # integrator variables
        self.error_z_prev = 0.0
        self.integral_z_error = 0.0

    def update_with_measurement(self, r, y):
        z, theta = y
        z_ref = r[0]

        # dirty derivative to estimate zdot
        z_diff = (z - self.z_prev) / P.ts
        self.zdot_hat = self.beta * self.zdot_hat + (1 - self.beta) * z_diff
        self.z_prev = z

        # dirty derivative to estimate thetadot
        theta_diff = (theta - self.theta_prev) / P.ts
        self.thetadot_hat = self.beta * self.thetadot_hat + (1 - self.beta) * theta_diff
        self.theta_prev = theta

        # compute input from partially estimated state
        xhat = np.array([z, theta, self.zdot_hat, self.thetadot_hat])

        # outer loop control
        error_z = z_ref - z
        if abs(self.zdot_hat) < 0.1:  # anti-windup: only integrate if zdot is small
            self.integral_z_error += P.ts * (error_z + self.error_z_prev) / 2
        self.error_z_prev = error_z
        theta_ref_pre = (
            self.kp_z * error_z
            + self.ki_z * self.integral_z_error
            - self.kd_z * self.zdot_hat
        )

        # low-pass filter the reference angle to cancel the zero and DC gain
        theta_ref = self.filter.update(theta_ref_pre)
        r[1] = theta_ref  # if you want to visualize the "reference" angle

        # inner loop control
        error_theta = theta_ref - theta
        F = self.kp_theta * error_theta - self.kd_theta * self.thetadot_hat
        u_unsat = np.array([F])
        u = self.saturate(u_unsat, u_max=P.force_max)

        return u, xhat


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
