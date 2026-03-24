import numpy as np
from .. import common
from . import params as P

class LongitudinalPDController(common.ControllerBase):
    def __init__(self):
        self.kp = 2.0
        self.kd = 0.7

    def update_with_state(self, r, x):
        #initialize once
        # hasattr(self, "theta_d1") returns True if the attribute already exists.
        #used so initialization only happens on the first update cell, and not every time update is called.
        if not hasattr(self, "theta_d1"):
            self.theta_d1 = 0.0
            self.theta_dot = 0.0
            self.sigma = 0.05
            self.beta = (2*self.sigma - P.ts) / (2*self.sigma + P.ts)

        theta_ref = r[1]
        theta = x[1]

        #dirty derivative
        self.theta_dot = self.beta * self.theta_dot + (1 - self.beta) * ((theta - self.theta_d1) / P.ts)
        self.theta_d1 = theta

        error = theta_ref - theta
        F_tilde = self.kp * error - self.kd * self.theta_dot

        #gravity compensation
        F_fl = (P.m1 * P.ell1 + P.m2 * P.ell2) * P.g / P.ellT * np.cos(theta)

        F = F_fl + F_tilde

        tau = 0.0 #as specified in lab manual, we are only controlling altitude, so no torque

        # convert to motor forces
        f = P.mixer @ np.array([F, tau])
        # convert forces to PWM
        km = P.g * (P.m1 * P.ell1 + P.m2 * P.ell2) / P.ellT
        u = f / km

        return u

#increasing kp will increase the speed of response, but can lead to overshoot and oscillations if too high
#increasing kd will increase damping, which can reduce overshoot and oscillations, but can also slow down the response if too high. 
#Tuning these gains is often a trade off between speed
#km is the system gain, which converts from force to PWM. 
#If km is too low, the controller will not be able to generate enough
#force to track the reference, leading to steady state error. 
# If km is too high, the controller will generate too much force, 
#which can lead to instability and oscillations.