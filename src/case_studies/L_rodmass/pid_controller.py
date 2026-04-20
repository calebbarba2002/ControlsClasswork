# 3rd-party
import numpy as np

# local
from . import params as P
from ..common import ControllerBase


class RodMassControllerPID(ControllerBase):
    """
    PID controller for rod-mass system.

    Works as:
    - PD if ki = 0
    - PID if ki > 0
    """

    def __init__(self, kp=0.0, kd=0.0, ki=0.0):
        # =========================
        # Gains
        # =========================
        self.kp = kp
        self.kd = kd
        self.ki = ki

        # =========================
        # Dirty derivative params
        # =========================
        self.sigma = 0.05  # tuning parameter (small = noisy, large = smooth)
        self.beta = (2*self.sigma - P.ts) / (2*self.sigma + P.ts)

        # =========================
        # Memory (for derivative + integrator)
        # =========================
        self.theta_d1 = P.theta0     # previous theta
        self.thetadot = 0.0          # estimated derivative

        self.error_d1 = 0.0
        self.integrator = 0.0

    def update_with_measurement(self, r, y):
        """
        r = reference (theta_r)
        y = measurement (theta)
        """

        theta = y[0]
        theta_r = r[0]

        # =========================
        # Error
        # =========================
        error = theta_r - theta

        # =========================
        # Dirty derivative of theta (NOT error!)
        # =========================
        self.thetadot = self.beta * self.thetadot + \
            (1 - self.beta) * ((theta - self.theta_d1) / P.ts)

        self.theta_d1 = theta

        # =========================
        # Integrator (trapezoidal)
        # =========================
        self.integrator += (P.ts / 2) * (error + self.error_d1)
        self.error_d1 = error

        # =========================
        # PID Control Law
        # =========================
        tau = self.kp * error \
              - self.kd * self.thetadot \
              + self.ki * self.integrator

        # =========================
        # Saturation (VERY IMPORTANT)
        # =========================
        tau = self.saturate(tau, u_max=P.tau_max)

        # =========================
        # MUST return (u, xhat)
        # =========================
        xhat = np.array([theta, self.thetadot])
        return np.array([tau]), xhat



# # 3rd-party
# import numpy as np

# # local (controlbook)
# from . import params as P
# from ..common import ControllerBase


# class RodMassControllerPID(ControllerBase):
#     """
#     PID controller for the rod-mass system.
    
#     Can be used as PD controller (ki=0) or full PID controller.
#     """

#     def __init__(self, kp=0., kd=0., ki=0.):
#         """
#         Initialize PID controller with specified gains.
        
#         Args:
#             kp: Proportional gain
#             kd: Derivative gain 
#             ki: Integral gain
#         """

#         # TODO: initialize necessary variables for your controller here ...

#     # TODO: implement functions needed for your controller here ...


