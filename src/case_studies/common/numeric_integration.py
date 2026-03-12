# standard library
from typing import Callable

# 3rd-party
import numpy as np
from numpy.typing import NDArray


def rk4_step(
    fn: Callable[[NDArray[np.float64], NDArray[np.float64]], NDArray[np.float64]],
    x: NDArray[np.float64],
    u: NDArray[np.float64],
    dt: float,
) -> NDArray[np.float64]:
    """
    Perform a numerical integration step using the 4th-order Runge-Kutta method
    (RK4).

    Args:
        fn: Function that computes the derivative of the state. It should be a
            function of the form fn(x, u) -> xdot.
        x: Current state vector of the system (time-varying over the step).
        u: Control input vector to the system (constant over the step).
            Hint: for observers, the input (labeled here as u) would be the
            measurement y.

    returns:
        x_next: State vector of the system after one time step.
    """
    k1 = fn(x, u)
    k2 = fn(x + k1 * dt / 2, u)
    k3 = fn(x + k2 * dt / 2, u)
    k4 = fn(x + k3 * dt, u)
    xdot = (k1 + 2 * k2 + 2 * k3 + k4) / 6
    x_next = x + xdot * dt
    return x_next
