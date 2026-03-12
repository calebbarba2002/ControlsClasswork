# 3rd-party
import numpy as np
from numpy.typing import NDArray
from matplotlib.axes import Axes

# local (controlbook)
from ..common import Visualizer
from .animator import MassAnimator


class MassVisualizer(Visualizer):
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
        x_labels = ["z (m)", r"$\dot{z}$ (m/s)"]
        u_labels = ["F (N)"]
        # x_labels = [] # TODO: fill in x labels (a string for each state)
        # u_labels = [] # TODO: fill in u labels
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
        )

    def get_system_animator(
        self,
        viz_object: Axes,
        x_hist: NDArray[np.float64],
        r_hist: NDArray[np.float64] | None,
        blit=True,
    ):
        assert isinstance(viz_object, Axes)
        r_hist = None  # animation does not use reference
        ax = viz_object
        return MassAnimator(ax, x_hist, r_hist, blit)
