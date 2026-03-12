"""
D_mass case study - Mass-spring-damper system

STUDENT VERSION - Files released incrementally throughout the semester.
Import errors are gracefully handled for not-yet-released components.
"""

# Core components (always available for animation/visualization)
from .animator import MassAnimator as Animator
from .visualizer import MassVisualizer as Visualizer
from . import params

__all__ = ["Animator", "Visualizer", "params"]

# Optional imports - automatically available as files are released
# Using try/except allows the package to work even when files are missing

try:
    from .dynamics import MassDynamics as Dynamics

    __all__.append("Dynamics")
except ImportError:
    pass

try:
    from .pd_controller import MassControllerPD as ControllerPD

    __all__.append("ControllerPD")
except ImportError:
    pass

try:
    from .pid_controller import MassControllerPID as ControllerPID

    __all__.append("ControllerPID")
except ImportError:
    pass

try:
    from .ss_controller import MassSSController as ControllerSS

    __all__.append("ControllerSS")
except ImportError:
    pass

try:
    from .ssi_controller import MassSSIController as ControllerSSI

    __all__.append("ControllerSSI")
except ImportError:
    pass

try:
    from .ssi_obs_controller import MassSSIOController as ControllerSSIO

    __all__.append("ControllerSSIO")
except ImportError:
    pass

try:
    from .ssi_dist_obs_controller import MassSSIDOController as ControllerSSIDO

    __all__.append("ControllerSSIDO")
except ImportError:
    pass

try:
    from .lqr_controller import MassSSIDOController as ControllerLQRIDO

    __all__.append("ControllerLQRIDO")
except ImportError:
    pass

try:
    from .loopshaped_controller import MassControllerLoopshaped as ControllerLoopshaped

    __all__.append("ControllerLoopshaped")
except ImportError:
    pass

try:
    from .loopshaping import design_loopshaped_controller

    __all__.append("design_loopshaped_controller")
except ImportError:
    pass
