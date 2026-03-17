# 3rd-party
import numpy as np
from numpy.typing import NDArray
def get_natural_frequency(tr: float, zeta: float = 0.707):
    """
    Compute natural frequency from rise time and damping ratio.

    Args:
        tr: Rise time (10-90%)
        zeta: Damping ratio (default 0.707 for critically damped)

    Returns:
        Natural frequency wn in rad/s
    """
    wn = 0.5 * np.pi / (tr * np.sqrt(1 - zeta**2))
    return float(wn)

def get_des_CE(tr: float, zeta: float):
    """
    Get desired characteristic equation coefficients from rise time and damping ratio.

    Returns coefficients for: s^2 + alpha1*s + alpha0
    Args:
        tr: Rise time (10-90%)
        zeta: Damping ratio
    Returns:
        Tuple of (alpha1, alpha0) coefficients
    """
    wn = get_natural_frequency(tr, zeta)
    alpha0 = wn**2
    alpha1 = 2 * zeta * wn
    return alpha1, alpha0

def get_pd_gains_from_des_CE(wn: float, zeta: float, b0: float):
    """
    Compute PD gains from desired closed-loop natural frequency and damping.

    Args:
        wn: Desired natural frequency
        zeta: Desired damping ratio
        b0: DC gain of plant (coefficient from transfer function)

    Returns:
        Tuple of (kp, kd) gains
    """
    kp = wn**2 / b0
    kd = 2 * zeta * wn / b0
    return kp, kd

def get_2nd_order_poles(tr: float, zeta: float):
    """
    Compute complex conjugate poles for 2nd order system from rise time and damping.

    Args:
        tr: Rise time (10-90%)
        zeta: Damping ratio
    Returns:
        Array of 2 poles (complex conjugates)
    """
    wn = get_natural_frequency(tr, zeta)
    char_poly = [1, 2 * zeta * wn, wn**2]
    poles = np.roots(char_poly)
    return poles