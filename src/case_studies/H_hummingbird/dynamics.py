# 3rd-party
import numpy as np

# local (controlbook)
from . import params as P
from ..common.dynamics_base import DynamicsBase
from . import eom_generated


class HummingbirdDynamics(DynamicsBase):
    def __init__(self, alpha=0.0):
        super().__init__(
            # Initial state conditions
            state0=np.array(
                [P.phi0, P.theta0, P.psi0, P.phidot0, P.thetadot0, P.psidot0]
            ),
            # Input limits [f_l, f_r]
            u_max=np.inf,  # we will saturate later based on input limits
            u_min=0.0,  # no negative forces
            # Time step for integration
            dt=P.ts,
        )
        # See params.py/textbook for details on these parameters

        # Parameter randomization is introduced in Chapter 10, you do not need
        # to include it before then
        # NOTE: Keys must match the parameter names in eom_generated.py functions
        self.eom_params = {
            "m1": self.randomize_parameter(P.m1, alpha),
            "m2": self.randomize_parameter(P.m2, alpha),
            "m3": self.randomize_parameter(P.m3, alpha),

            "J1_x": self.randomize_parameter(P.J1x, alpha),
            "J1_y": self.randomize_parameter(P.J1y, alpha),
            "J1_z": self.randomize_parameter(P.J1z, alpha),

            "J2_x": self.randomize_parameter(P.J2x, alpha),
            "J2_y": self.randomize_parameter(P.J2y, alpha),
            "J2_z": self.randomize_parameter(P.J2z, alpha),

            "J3_x": self.randomize_parameter(P.J3x, alpha),
            "J3_y": self.randomize_parameter(P.J3y, alpha),
            "J3_z": self.randomize_parameter(P.J3z, alpha),

            "ell1": self.randomize_parameter(P.ell1, alpha),
            "ell2": self.randomize_parameter(P.ell2, alpha),
            "ell3_x": self.randomize_parameter(P.ell3x, alpha),
            "ell3_y": self.randomize_parameter(P.ell3y, alpha),
            "ell3_z": self.randomize_parameter(P.ell3z, alpha),

            "ell_T": self.randomize_parameter(P.ellT, alpha),

            "d": self.randomize_parameter(P.d, alpha),
            "g": P.g,
        }
        # TODO: should km be randomized, or should we use the already randomized
        # parameters to compute it?
        # self.km = self.randomize_parameter(P.km, alpha)
        p = self.eom_params
        self.km = P.g * (p["m1"] * p["ell1"] + p["m2"] * p["ell2"]) / p["ell_T"]
        self.B = P.beta * np.eye(3)  # friction-based terms

    ############################################################################
    # The following 4 methods are where we use the generated functions from
    # SymPy to calculate the terms in the equations of motion: the mass matrix
    # M, the coriolis vector C, the gravity terms dP_dq, and the generalized
    # force vector tau. Each of these methods are just wrappers to call the
    # generated functions with the appropriate parameters.
    # See the end of h3_generate_EL.py for an example of how to call the
    # generated functions.

    # NOTE: IF YOU DID NOT GENERATE THESE FUNCTIONS, YOU WOULD NEED TO
    # IMPLEMENT THE CALCULATION OF THESE TERMS BY HAND.

    def calculate_M(self, x):
        M = eom_generated.calculate_M(x, **self.eom_params)
        return M

    def calculate_C(self, x):
        C = eom_generated.calculate_C(x, **self.eom_params)
        return C

    def calculate_dP_dq(self, x):
        dP_dq = eom_generated.calculate_dP_dq(x, **self.eom_params)
        return dP_dq

    def calculate_tau(self, x, u):
        tau = eom_generated.calculate_tau(x, u, **self.eom_params)
        return tau

    ############################################################################

    def f(self, x, u):
        """
        Full nonlinear dynamic model `xdot = f(x,u)`.

        Args:
            x (1D numpy array): state vector [q, qdot] or
                [phi, theta, psi, phidot, thetadot, psidot]
            u (1D numpy array): input vector [f_l, f_r]
        Returns:
            xdot (1D numpy array): time derivative of state vector
                [phidot, thetadot, psidot, phiddot, thetaddot, psiddot]
        """
        # Ensure inputs are flattened for consistent indexing
        x = x.flatten()
        u = u.flatten()
        
        # Re-label terms for readability
        qdot = x[3:6]
        # phidot, thetadot, psidot = qdot  # TODO: use this?

        # Evaluate equations of motion
        M = self.calculate_M(x)
        C = self.calculate_C(x)
        dP_dq = self.calculate_dP_dq(x)
        tau = self.calculate_tau(x, u)
        friction = self.B @ qdot

        qddot = np.linalg.solve(M, tau - C - dP_dq - friction)
        # phiddot, thetaddot, psiddot = qddot  # TODO: use this?

        # Package and return xdot
        # TODO: use this?
        # xdot = np.array([phidot, thetadot, psidot, phiddot, thetaddot, psiddot])
        xdot = np.concatenate((qdot, qddot))
        return xdot

    def h(self):
        """
        Output function `y = h(x)`, where x is self.state and does not need to
        be passed in. Here we assume we can measure all the generalized
        coordinates, i.e., position states.

        Returns:
            y (1D numpy array): output vector [phi, theta, psi]
        """
        y = self.state[:3]
        return y

    def update(self, pwm):
        """
        Integrates the system dynamics forward one time step using RK4.
        This version accepts PWM inputs and converts them to forces.

        Args:
            pwm (NDArray[np.float64]): input vector [pwm_l, pwm_r].
        Returns:
            y (NDArray[np.float64]): measured output vector [phi, theta, psi].
        """
        # saturate pwm
        pwm = np.clip(pwm, 0.0, 1.0)

        # convert pwm to motor forces [f_l, f_r]
        u = pwm * self.km

        # call update from parent class (DynamicsBase)
        y = super().update(u)
        return y