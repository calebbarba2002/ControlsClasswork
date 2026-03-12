# 3rd-party
import numpy as np
from numpy.typing import NDArray
from pyqtgraph.opengl import GLViewWidget

# local (controlbook)
from ..common import Visualizer
from .animator import HummingbirdAnimator


class HummingbirdVisualizer(Visualizer):
    def __init__(
        self,
        t_hist: NDArray[np.float64],
        x_hist: NDArray[np.float64],
        u_hist: NDArray[np.float64],
        r_hist: NDArray[np.float64] | None = None,
        xhat_hist: NDArray[np.float64] | None = None,
        d_hist: NDArray[np.float64] | None = None,
        dhat_hist: NDArray[np.float64] | None = None,
    ):
        # NOTE: base Visualizer class will convert data to degrees for plotting
        # if "deg" is in the label
        x_labels = [
            r"$\phi$ (deg)",
            r"$\theta$ (deg)",
            r"$\psi$ (deg)",
            r"$\dot{\phi}$ (rad/s)",
            r"$\dot{\theta}$ (rad/s)",
            r"$\dot{\psi}$ (rad/s)",
        ]
        u_labels = [r"$u_l$ (pwm)", r"$u_r$ (pwm)"]
        super().__init__(
            t_hist,
            x_hist,
            x_labels,
            u_hist,
            u_labels,
            r_hist,
            d_hist,
            xhat_hist,
            dhat_hist,
            mpl_axis=False,
        )

    def get_system_animator(
        self,
        viz_object: GLViewWidget,
        x_hist: NDArray[np.float64],
        r_hist: NDArray[np.float64] | None,
        blit=True,
    ):
        assert isinstance(viz_object, GLViewWidget)
        r_hist = None  # animation does not use reference
        return HummingbirdAnimator(viz_object, x_hist, r_hist)
