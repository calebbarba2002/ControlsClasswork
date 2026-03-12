# 3rd-party
import numpy as np
from numpy.typing import NDArray


class ControllerBase:
    """
    ═══════════════════════════════════════════════════════════════════════════\n
    # BASE CLASS FOR ALL CONTROLLERS
    ═══════════════════════════════════════════════════════════════════════════\n

    This is a template that all controllers (PD, PID, state-space, etc.)
    will use. Think of it as a requirement that says "every controller must
    know how to compute a control input." Our simulation framework will call the
    controller's update method at each time step to get the control input based
    on the current state or measurement.

    YOU MUST IMPLEMENT ONE OF THESE METHODS FOR MOST CONTROLLERS:
    1. update_with_state(r, x)        - When full state is available
    2. update_with_measurement(r, y)  - When measurements of some states are available

    ═══════════════════════════════════════════════════════════════════════════\n
    ## WHICH ONE TO IMPLEMENT? (Three Approaches)
    ═══════════════════════════════════════════════════════════════════════════\n

    APPROACH 1 - PD control and Full State Feedback (Chapters 7, 8, 11, and 12):
        Implement: update_with_state(r, x)
        Use when: You assume you know ALL states (positions AND velocities)
        Example: Perfect sensors or simulation with full state access

    APPROACH 2 - Digital PID Control with Measurement + Numerical Derivative (Chapter 10):
        Implement: update_with_measurement(r, y)
        - update_with_measurement(r, y): Computes velocities via "dirty derivative"
        Use when: Only measure position, estimate velocity numerically
        Limitation: Noisy! Amplifies sensor noise.

    APPROACH 3 - Full State Feedback + Observer using Measurement (Chapters 12-14):
        Implement: update_with_measurement(r, y)
        Use when: Can only measure some states, and can use observer to estimate others
        Advantage: Can estimate unknown states and disturbances
        Example: Measure position, observer estimates velocity using dynamics
        Limitation: Effect of noise depends on observer design (for example Kalman
                    filters for observers work better, but are out of the scope of this
                    class)

    ═══════════════════════════════════════════════════════════════════════════\n
    ## What to implement in your subclass:
    ═══════════════════════════════════════════════════════════════════════════\n
    Methods you must implement in your subclass (one or the other, depending on which homework):
        update_with_state: Control law when you know the full state.
        update_with_measurement: Control law when you only have sensor measurements.

    Methods provided for you in this parent class:
        saturate: Clips control inputs to maximum allowed values.

    See _TEMPLATE/controller_template.py for a complete example.
    ═══════════════════════════════════════════════════════════════════════════
    """

    def update_with_state(
        self,
        r: NDArray[np.float64],
        x: NDArray[np.float64],
    ) -> NDArray[np.float64]:
        """
        ═══════════════════════════════════════════════════════════════════════\n
        ### PLACE HOLDER FOR CONTROL LAW WITH FULL STATE (or ESTIMATED STATE) FEEDBACK
        ═══════════════════════════════════════════════════════════════════════\n
        This method/function will be implemented in your controller subclass, where
        you'll include this base class through inheritance. Then you'll define the
        specific control law (e.g., PD control) based on the full state vector x.

        This will be used in chapters 7, 8, 11, and 12.
        - Use self.saturate() to limit control to realistic values

        ═══════════════════════════════════════════════════════════════════════
        Args:
            r: Reference vector (what you WANT the output to be)\n
               Example: [desired_position]
               NOTE: Usually same size as output y, not state x

            x: State vector (actual state)\n
               Example: [position, velocity]

        Returns:
            u: Control input vector (what to DO or apply to the system as control)\n
               Example: [force]
        """

        msg = (
            f"\n\n"
            f"════════════════════════════════════════════════════════════════\n"
            f"TODO: Implement update_with_state() in your controller!\n"
            f"════════════════════════════════════════════════════════════════\n"
            f"This method implements feedback control assuming access to full state (Ch 7, 8, 11, 12).\n"
            f"Reference: case_studies/A_arm/pd_controller.py\n"
            f"Template:  case_studies/_TEMPLATE/controller_template.py\n"
            f"════════════════════════════════════════════════════════════════\n"
        )

        raise NotImplementedError(msg)

    def update_with_measurement(
        self,
        r: NDArray[np.float64],
        y: NDArray[np.float64],
    ) -> tuple[NDArray[np.float64], ...]:
        """
        ═══════════════════════════════════════════════════════════════════════\n
        ### PLACE HOLDER FOR CONTROL LAW WITH MEASUREMENT FEEDBACK (OUTPUT FEEDBACK)
        ═══════════════════════════════════════════════════════════════════════\n
        This method/function will be implemented in your controller subclass, where
        you'll include this base class through inheritance. Then you'll define the
        specific control law (e.g., digital PID control) based on the measurement
        vector y.

        This will be used in chapters 10, 13, and 14.
        - Use self.saturate() to limit control to realistic values

        ═══════════════════════════════════════════════════════════════════════\n
        #### Two common approaches for output feedback controllers:
        ═══════════════════════════════════════════════════════════════════════\n
        APPROACH 1 - Numerical Derivative (Chapter 10):
            Use "dirty derivative" to estimate velocity from position

            PROS: Simple, no model needed
            CONS: Amplifies sensor noise, not great for noisy measurements

        APPROACH 2 - Observer-Based (Chapters 13-14):
            Use proper observer to estimate unmeasured states

            PROS: Uses dynamics model to improve estimates, can also estimate disturbance forces
            CONS: More complex, requires accurate model
        ═══════════════════════════════════════════════════════════════════════\n
        Args:
            r: reference vector of the system. r should be
                a vector of the same size as the output vector y.
            y: measurement vector of the system.
        Returns:
            tuple:
                - u: control input vector to apply to the system.
                - xhat: estimated state vector (if applicable).
                - dhat: estimated disturbance vector (if applicable).
        ═══════════════════════════════════════════════════════════════════════
        """

        msg = (
            f"\n\n"
            f"════════════════════════════════════════════════════════════════\n"
            f"TODO: Implement update_with_measurement() in your controller!\n"
            f"════════════════════════════════════════════════════════════════\n"
            f"This method implements output feedback with access to measurements only (Ch 10, 13, and 14).\n"
            f"Reference: case_studies/A_arm/ssi_obs_controller.py\n"
            f"Template:  case_studies/_TEMPLATE/controller_template.py\n"
            f"════════════════════════════════════════════════════════════════\n"
        )

        raise NotImplementedError(msg)

    def saturate(
        self,
        u: NDArray[np.float64] | float,
        u_max: NDArray[np.float64] | float,
        u_min: NDArray[np.float64] | float | None = None,
    ):
        """
        Saturate input signal u to be within u_min and u_max.

        Args:
            u: input vector to saturate.
            u_max: upper bound on u. If a numpy array,
                it must be the same size as u. If a float, this value will be used
                as the limit for all values of u.
            u_min: lower bound on u. If None,
                then it is set to the negative of u_max. If a numpy array, it
                must be the same size as u. If a float, this value will be used
                as the limit for all values of u.
        Returns:
            u_saturated: the saturated input signal.
        """
        if u_min is None:
            u_min = -u_max
        # can use np.clip(u, u_min, u_max) if u is a float
        return np.clip(u, u_min, u_max)
