# 3rd-party
import numpy as np
from numpy.typing import NDArray

# local (controlbook)
from .numeric_integration import rk4_step


class DynamicsBase:
    """
    ═══════════════════════════════════════════════════════════════════════════\n
    # BASE CLASS FOR ALL PHYSICAL SYSTEM DYNAMICS
    ═══════════════════════════════════════════════════════════════════════════\n
    This is a "template" or "blueprint" that every physical system
    (arm, pendulum, mass, etc.) will use. Think of it like a requirements list
    that says "every dynamics model must have certain methods and properties."
    Our simulation framework will call these methods to simulate how the system
    behaves over time. YOU will implement these methods in your specific
    system's dynamics file (e.g., D_mass/dynamics.py).

    YOU ONLY NEED TO IMPLEMENT TWO METHODS IN YOUR SUBCLASS:
    1. f(x, u)  - The dynamics equation (how does the system change over time?)
    2. h()      - The output equation (what can we measure?)

    See _TEMPLATE/dynamics_template.py for detailed instructions on how to
    implement your own dynamics class.

    ═══════════════════════════════════════════════════════════════════════════\n
    ## What to implement in your subclass:
    ═══════════════════════════════════════════════════════════════════════════\n
    Methods you must implement in your subclass:
        f: state evolution equation xdot = f(x,u). See textbook Chapters 2-6.
        h: output equation y = h(x). See textbook Chapter 6.

    Methods provided for you:
        update: method runs the simulation step (uses Runge-Kutta integration)
        randomize_parameter: adds uncertainty to parameters (when unsure of the
                             model).
    """

    def __init__(
        self,
        state0: NDArray[np.float64],
        u_min: NDArray[np.float64] | float,
        u_max: NDArray[np.float64] | float,
        dt: float = 0.01,
    ):
        """
        Initializes the DynamicsBase class.

        Args:
            state0: initial state vector of the system.
            u_min: minimum input vector to the system.
            u_max: maximum input vector to the system.
            dt: time step for numerical integration.
        """
        self.state = state0.copy()
        self.u_min = u_min
        self.u_max = u_max
        self.dt = dt
        self.rng = np.random.default_rng()  # random number generator

    def update(self, u: NDArray[np.float64]) -> NDArray[np.float64]:
        """
        Propagates the system dynamics forward one time step using RK4.

        Args:
            u: input vector to apply to the system.
        Returns:
            y: output vector of the system.
        """
        u_sat = np.clip(
            u, self.u_min, self.u_max
        )  # saturate input to enforce physical limits
        self.state = rk4_step(self.f, self.state, u_sat, self.dt)
        y = self.h()
        return y.copy()

    def f(self, x: NDArray[np.float64], u: NDArray[np.float64]) -> NDArray[np.float64]:
        """
        ═══════════════════════════════════════════════════════════════════════\n
        ### DYNAMICS EQUATION: xdot = f(x, u)
        ═══════════════════════════════════════════════════════════════════════\n
        This is how the state changes over time. See chapters 2-6 of the textbook.
        We use the differential equations derived in earlier chapters, and put in state
        variable form.

        Args:
            x: State vector (position, velocity, angle, etc.) - can be ANY values
            u: Control input (force, torque, etc.) - can be ANY values

        Returns:
            xdot: Time derivative of state vector

        #### IMPORTANT NOTES:
        - Do NOT use "self" to access state or parameters! Use x and u only.
           WHY? Because the RK4 integrator calls this function with multiple
           intermediate x values)
        - You CAN use self.param1, self.param2, etc. for physical parameters
          (mass, spring constant, damping, etc.) - these don't change.

        ### COMMON STUDENT MISTAKES:
        - Using self.state instead of x → will break RK4 integration!
        - Forgetting to return np.array([...]) → must be numpy array
        - Using degrees instead of radians → numpy uses radians
        - Wrong sign on equations → check your work carefully!

        See examples in case_studies/A_arm/dynamics.py, etc.

        See also textbook Section 6.1 or _TEMPLATE/dynamics_template.py for details.
        ═══════════════════════════════════════════════════════════════════════
        """

        raise NotImplementedError(
            f"\n\n"
            f"════════════════════════════════════════════════════════════════\n"
            f"TODO: Implement f(x, u) in your dynamics file!\n"
            f"════════════════════════════════════════════════════════════════\n"
            f"This method should contain the differential equations for your system.\n"
            f"Reference: case_studies/A_arm/dynamics.py (working example)\n"
            f"════════════════════════════════════════════════════════════════\n"
        )

    def h(self) -> NDArray[np.float64]:
        """
        ═══════════════════════════════════════════════════════════════════════\n
        ### OUTPUT EQUATION: y = h(x)
        ═══════════════════════════════════════════════════════════════════════\n
        This defines what you can MEASURE from your system, or "y".

        In real life, you can't always measure everything! For example:
        - You might measure position but not velocity (need to estimate velocity)
        - You might measure anglular velocity, but not angle

        Args:
            None (we will access the state from self.state)

        Returns:
            y: Measurement vector (what your sensors can measure about your states)

        ### IMPORTANT NOTES:
        - Use self.state here (UNLIKE f() where you use the x parameter)
          WHY? Because we're measuring the CURRENT state of the system
        - Return as numpy array, even if measuring just one thing

        ### COMMON CASES:
        - Measure position only:         return self.state[0:1]
        - Measure entire state:   return self.state.copy()
        - Measure first n states:        return self.state[:n]

        See examples in case_studies/A_arm/dynamics.py, etc.

        For all controlbook case studies, all of our output equations are linear,
        so we can represent them as y = C*x. However, this is not always the case.
        """

        raise NotImplementedError(
            f"\n\n"
            f"════════════════════════════════════════════════════════════════\n"
            f"TODO: Implement h(x) in your dynamics file!\n"
            f"════════════════════════════════════════════════════════════════\n"
            f"This method must contain the output equations for your system.\n"
            f"Reference: case_studies/A_arm/dynamics.py (working example)\n"
            f"════════════════════════════════════════════════════════════════\n"
        )

    def randomize_parameter(self, param: float, alpha: float) -> float:
        """
        Randomizes a parameter by a percentage defined by alpha. For example, if
        alpha=0.1, the parameter will be randomized by +/- 10%.

        Args:
            param (float): parameter to randomize.
            alpha (float): percentage to randomize the parameter by (0 to 1).
        Returns:
            randomized_param (float): randomized parameter to use in control or simulation.
        """
        if not 0.0 <= alpha <= 1.0:
            raise ValueError(f"'alpha' ({alpha}) must be between 0 and 1")
        percent = self.rng.uniform(low=-alpha, high=alpha)
        return param * (1 + percent)
