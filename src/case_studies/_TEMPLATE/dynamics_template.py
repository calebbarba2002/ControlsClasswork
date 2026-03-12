"""
═══════════════════════════════════════════════════════════════════════════════
DYNAMICS TEMPLATE FOR STUDENTS
═══════════════════════════════════════════════════════════════════════════════

INSTRUCTIONS:
1. Copy this file to your case study folder (e.g., D_mass/dynamics.py)
2. Rename "YourSystemDynamics" to match your system (e.g., "MassDynamics")
3. Fill in the TODOs below
4. Refer to A_arm/dynamics.py or B_pendulum/dynamics.py for working examples

WHAT THIS FILE DOES:
This file describes HOW your physical system behaves - its dynamics.
Think of it as teaching the computer the physics equations from your textbook.

═══════════════════════════════════════════════════════════════════════════════
"""

# 3rd-party or pip installed python libraries
import numpy as np

# local (controlbook) modules
from ..common.dynamics_base import DynamicsBase
from . import params as P


class YourSystemDynamics(DynamicsBase):
    """
    Dynamics model for [YOUR SYSTEM NAME HERE].

    This class implements the differential equations that describe how
    your system changes over time.
    """

    def __init__(self, alpha=0.0):
        """
        Initialize the dynamics with starting conditions and parameters.

        Args:
            alpha: Parameter uncertainty (0.0 = perfect knowledge, 0.2 = ±20% error)
                   Used to test if your controller is robust to model uncertainty.
        """
        # ═══════════════════════════════════════════════════════════════
        # TODO: Set initial conditions from params.py
        # ═══════════════════════════════════════════════════════════════
        # EXAMPLE: For a mass-spring system with position and velocity:
        #   state0=np.array([P.z0, P.zdot0])
        #
        # EXAMPLE: For a pendulum with angle and angular velocity:
        #   state0=np.array([P.theta0, P.thetadot0])

        # using "super()" is required when we have a parent class (DynamicsBase)
        super().__init__(
            state0=np.array([P.STATE1_INITIAL, P.STATE2_INITIAL]),  # CHANGE THIS!
            u_max=P.INPUT_MAX,  # CHANGE THIS! Maximum control input
            u_min=-P.INPUT_MAX,  # Usually negative of max (symmetric limits)
            dt=P.ts,  # Time step - leave this as is
        )

        # ═══════════════════════════════════════════════════════════════
        # TODO: Load physical parameters from params.py
        # ═══════════════════════════════════════════════════════════════
        # The randomize_parameter() method adds uncertainty for testing robustness.
        #
        # EXAMPLE: For a mass-spring-damper system:
        #   self.m = self.randomize_parameter(P.m, alpha)  # mass
        #   self.k = self.randomize_parameter(P.k, alpha)  # spring constant
        #   self.b = self.randomize_parameter(P.b, alpha)  # damping coefficient

        self.param1 = self.randomize_parameter(P.PARAM1, alpha)  # CHANGE THIS!
        self.param2 = self.randomize_parameter(P.PARAM2, alpha)  # CHANGE THIS!
        # ADD MORE PARAMETERS AS NEEDED

    def f(self, x, u):
        """
        State derivative function: xdot = f(x, u)

        This is the HEART of your dynamics - the differential equations in state
        variable form.

        Args:
            x: State vector [state1, state2, ...] (position, velocity, angle, etc.)
            u: Control input vector [input1, input2, ...] (force, torque, etc.)

        Returns:
            xdot: Time derivative of state [state1_dot, state2_dot, ...]

        IMPORTANT NOTES:
        - Do NOT use "self" in this function! Use x and u only.
        - Why? Because it will be called many times when we perform numeric integration,
          and it needs to work for ANY x and u values, not just the current state.
        - You CAN use self.param1, self.param2, etc. (physical parameters)

        COMMON MISTAKES:
        ❌ Using self.state instead of x
        ❌ Forgetting to return np.array([...])
        ❌ Using degrees instead of radians (numpy trig functions use radians!)
        ❌ Wrong order in state vector
        """
        # ═══════════════════════════════════════════════════════════════
        # STEP 1: Unpack the state vector x into named variables if helpful
        # ═══════════════════════════════════════════════════════════════
        # This makes your code more readable than using x[0], x[1], etc.
        #
        # EXAMPLE for mass-spring system:
        #   position = x[0]
        #   velocity = x[1]

        state1 = x[0]  # RENAME THIS to something meaningful (e.g., "position", "theta")
        state2 = x[
            1
        ]  # RENAME THIS to something meaningful (e.g., "velocity", "thetadot")
        # ADD MORE IF NEEDED (x[2], x[3], etc.)

        # ═══════════════════════════════════════════════════════════════
        # STEP 2: Unpack the control input vector u
        # ═══════════════════════════════════════════════════════════════
        # EXAMPLE:
        #   force = u[0]
        #   torque = u[0]

        control_input = u[0]  # RENAME THIS (e.g., "force", "torque")
        # ADD MORE IF NEEDED (u[1], u[2], etc.)

        # ═══════════════════════════════════════════════════════════════
        # STEP 3: Calculate state derivatives using physics equations
        # ═══════════════════════════════════════════════════════════════
        # Get these equations from your textbook!
        #
        # EXAMPLE for simple pendulum:
        #   theta_dot = thetadot
        #   thetadot_dot = (torque - self.b*thetadot - self.m*self.g*self.ell*np.sin(theta)) / self.J

        # TODO: Replace these with YOUR equations!
        state1_dot = None  # REPLACE with your equation
        state2_dot = None  # REPLACE with your equation
        # ADD MORE IF NEEDED (state3_dot, state4_dot, etc.)

        # ═══════════════════════════════════════════════════════════════
        # STEP 4: Return derivatives as a numpy array
        # ═══════════════════════════════════════════════════════════════
        xdot = np.array([state1_dot, state2_dot])  # Must match order of state vector!
        return xdot

    def h(self):
        """
        Output equation: y = h(x)

        This defines what you can MEASURE from the system.
        In real systems, you can't usually measure everything!

        Returns:
            y: Measurement vector (what sensors can meaure about your states)

        This should most often be accomplished by using the output matrix Cm from
        your params.py file, but you can also define custom measurements here.

        IMPORTANT:
        - Use self.state here (unlike in f() where you use x)
        - Return as numpy array, even if it's just one value
        """
        # ═══════════════════════════════════════════════════════════════
        # TODO: Define what you can measure
        # ═══════════════════════════════════════════════════════════════
        # Most common case: Only measure position (not velocity) and use Cm
        Cm = P.Cm  # Measurement matrix from params.py
        measurement = Cm @ self.state  # Use output matrix to get measurements

        # Alternative: Measure all states directly
        # measurement = self.state.copy()
        # OR
        # Cm = np.eye(len(self.state))
        # measurement = Cm @ self.state

        return measurement
